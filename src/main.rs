#![no_std]
#![no_main]

use bsp::entry;
use defmt_rtt as _;
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    prelude::*,
    text::{Baseline, Text, TextStyleBuilder},
};
use embedded_hal::delay::DelayNs;

use defmt::println;

use panic_probe as _;

use epd_waveshare::{epd2in13_v2::*, prelude::*};

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico::{self as bsp};
// use sparkfun_pro_micro_rp2040 as bsp;
//

use scd30_interface::data::{
    AmbientPressure, AmbientPressureCompensation, DataStatus, MeasurementInterval,
};
use scd30_interface::Scd30;

use bsp::hal::fugit::RateExtU32;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio, pac,
    sio::Sio,
    spi,
    watchdog::Watchdog,
    Timer, I2C,
};
use embedded_hal_bus::spi::ExclusiveDevice;
const CO2_OFFSET: f32 = 300.0;
const TEMP_OFFSET: f32 = -1.0;

#[entry]
#[allow(clippy::empty_loop)]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let pins = gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Define pins
    let pin_cs = pins.gpio14.into_push_pull_output();
    let pin_busy = pins.gpio2.into_floating_input();
    let pin_dc = pins.gpio3.into_push_pull_output();
    let pin_rst = pins.gpio4.into_push_pull_output();
    let spi_mosi = pins.gpio11.into_function::<gpio::FunctionSpi>();
    let spi_sclk = pins.gpio10.into_function::<gpio::FunctionSpi>();
    let i2c_sda: gpio::Pin<_, gpio::FunctionI2C, _> = pins.gpio18.reconfigure();
    let i2c_scl: gpio::Pin<_, gpio::FunctionI2C, _> = pins.gpio19.reconfigure();
    println!("Hi!");

    // Init SPI
    let spi = spi::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_sclk));
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16.MHz(),
        embedded_hal::spi::MODE_0,
    );
    let mut spi_device = ExclusiveDevice::new(spi, pin_cs, timer);

    // Setup screen
    let mut epd = Epd2in13::new(&mut spi_device, pin_busy, pin_dc, pin_rst, &mut timer, None)
        .expect("Epd should work");

    // Init i2c and co2 sensor
    let i2c = I2C::i2c1(
        pac.I2C1,
        i2c_sda,
        i2c_scl,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
    let mut sensor = Scd30::new(i2c);
    sensor
        .set_measurement_interval(MeasurementInterval::try_from(5).unwrap())
        .unwrap();
    sensor
        .set_automatic_self_calibration(scd30_interface::data::AutomaticSelfCalibration::Active)
        .unwrap();
    sensor
        .trigger_continuous_measurements(Some(AmbientPressureCompensation::CompensationPressure(
            AmbientPressure::try_from(1026).unwrap(),
        )))
        .unwrap();

    // Use display graphics from embedded-graphics and draw text
    let mut display = Display2in13::default();
    display.set_rotation(DisplayRotation::Rotate90);
    display.clear(Color::White).ok();

    draw_text(&mut display, "Hello from Rust!", 5, 0);
    epd.update_and_display_frame(&mut spi_device, display.buffer(), &mut timer)
        .unwrap();
    epd.set_refresh(&mut spi_device, &mut timer, RefreshLut::Quick)
        .unwrap();

    // Read out firmware version
    let firmware_version = sensor.read_firmware_version().unwrap();
    println!("Firmware version: {}", firmware_version);

    loop {
        timer.delay_ms(5_000);
        while sensor.is_data_ready().unwrap() != DataStatus::Ready {}

        let sample = sensor.read_measurement().expect("Should meassure co2");
        println!(
            "Temp:{}, co2:{}\r\n",
            sample.temperature + TEMP_OFFSET,
            sample.co2_concentration + CO2_OFFSET
        );
        let mut buf = [0u8; 64];
        let text = format_no_std::show(
            &mut buf,
            format_args!(
                "Temp: {:.1}, co2: {:.0}      ",
                sample.temperature + TEMP_OFFSET,
                sample.co2_concentration + CO2_OFFSET
            ),
        )
        .unwrap();

        draw_text(&mut display, text, 5, 20);
        epd.update_and_display_frame(&mut spi_device, display.buffer(), &mut timer)
            .unwrap();
    }
}

fn draw_text(display: &mut Display2in13, text: &str, x: i32, y: i32) {
    let style = MonoTextStyleBuilder::new()
        .font(&embedded_graphics::mono_font::ascii::FONT_10X20)
        .background_color(Color::White)
        .text_color(Color::Black)
        .build();

    let text_style = TextStyleBuilder::new().baseline(Baseline::Top).build();

    let _ = Text::with_text_style(text, Point::new(x, y), style, text_style).draw(display);
}

#[defmt::panic_handler]
fn panic() -> ! {
    bsp::hal::reset();
}
