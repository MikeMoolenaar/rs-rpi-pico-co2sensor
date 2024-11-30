#![no_std]
#![no_main]

use cortex_m as _;
use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Input, Level, Output, Pull},
    i2c::{I2c, InterruptHandler},
    peripherals::I2C1,
    spi::{Config, Spi},
};
use embassy_time::{Delay, Timer};

use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    prelude::*,
    text::{Baseline, Text, TextStyleBuilder},
};
use embedded_hal_bus::spi::ExclusiveDevice;
use epd_waveshare::{epd2in13_v2::*, prelude::*};
use scd30_interface::data::{
    AmbientPressure, AmbientPressureCompensation, DataStatus, MeasurementInterval,
};
use scd30_interface::Scd30;

const CO2_OFFSET: f32 = 300.0;
const TEMP_OFFSET: f32 = -1.0;

embassy_rp::bind_interrupts!(
    struct Irqs {
        I2C1_IRQ => InterruptHandler<I2C1>;
    }
);

#[allow(clippy::empty_loop)]
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Define pins
    let _pin_cs = Output::new(p.PIN_14, Level::Low);
    let pin_busy = Input::new(p.PIN_2, Pull::Down);
    let pin_dc = Output::new(p.PIN_3, Level::Low);
    let pin_rst = Output::new(p.PIN_4, Level::Low);
    info!("Hi!");

    let mut config = Config::default();
    config.frequency = 16_000_000;
    //                                    CLK       MOSI
    let spi = Spi::new_txonly(p.SPI1, p.PIN_10, p.PIN_11, p.DMA_CH0, Config::default());
    let mut spi_delay = Delay;
    let mut spi_device =
        ExclusiveDevice::new(spi, Output::new(p.PIN_5, Level::Low), &mut spi_delay);

    // Setup screen
    let mut epd = Epd2in13::new(&mut spi_device, pin_busy, pin_dc, pin_rst, &mut Delay, None)
        .expect("Epd should work");

    // Init i2c and co2 sensor
    let i2c = I2c::new_async(
        p.I2C1,
        p.PIN_19, // scl
        p.PIN_18, // sda
        Irqs,
        embassy_rp::i2c::Config::default(),
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
    epd.update_and_display_frame(&mut spi_device, display.buffer(), &mut Delay)
        .unwrap();
    epd.set_refresh(&mut spi_device, &mut Delay, RefreshLut::Quick)
        .unwrap();

    // Read out firmware version
    let firmware_version = sensor.read_firmware_version().unwrap();
    info!("Firmware version: {}", firmware_version);

    loop {
        Timer::after_secs(5).await;
        while sensor.is_data_ready().unwrap() != DataStatus::Ready {}

        let sample = sensor.read_measurement().expect("Should meassure co2");
        info!(
            "Temp:{}, co2:{}",
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
        epd.update_and_display_frame(&mut spi_device, display.buffer(), &mut Delay)
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
    cortex_m::peripheral::SCB::sys_reset();
}
