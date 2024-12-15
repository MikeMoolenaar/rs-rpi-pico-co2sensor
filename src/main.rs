#![no_std]
#![no_main]

use cortex_m as _;
use libscd::asynchronous::scd30::Scd30;
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

const CO2_OFFSET: u16 = 400;
const TEMP_OFFSET: f32 = -2.0;

embassy_rp::bind_interrupts!(
    struct Irqs {
        I2C1_IRQ => InterruptHandler<I2C1>;
    }
);

#[allow(clippy::empty_loop)]
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Booting...");

    // Define pins
    let pin_busy = Input::new(p.PIN_2, Pull::Down);
    let pin_dc = Output::new(p.PIN_3, Level::Low);
    let pin_rst = Output::new(p.PIN_4, Level::Low);

    // Define SPI
    let mut config = Config::default();
    config.frequency = 16_000_000;
    //                                CLK       MOSI
    let spi = Spi::new_txonly(p.SPI1, p.PIN_10, p.PIN_11, p.DMA_CH0, Config::default());
    let mut spi_delay = Delay;
    let mut spi_device =
        ExclusiveDevice::new(spi, Output::new(p.PIN_5, Level::Low), &mut spi_delay);

    // Setup screena (max_x = 250, max_y = 122)
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
    let mut sensor = Scd30::new(i2c, Delay);
    sensor.set_measurement_interval(5).await.unwrap();
    sensor.stop_continuous_measurement().await.unwrap();
    sensor.start_continuous_measurement(1026).await.unwrap();
    sensor
        .enable_automatic_self_calibration(true)
        .await
        .unwrap();

    // Use display graphics from embedded-graphics and draw text
    let mut display = Display2in13::default();
    display.set_rotation(DisplayRotation::Rotate90);
    display.clear(Color::White).ok();

    draw_text(&mut display, "Booting...", 5, 0);
    epd.update_and_display_frame(&mut spi_device, display.buffer(), &mut Delay)
        .unwrap();
    epd.set_refresh(&mut spi_device, &mut Delay, RefreshLut::Quick)
        .unwrap();

    // Read out firmware version
    let firmware_version = sensor.read_firmware_version().await.unwrap();
    info!(
        "Scp30 connected with firmware version: {}.{}",
        firmware_version.0, firmware_version.1
    );

    loop {
        Timer::after_secs(5).await;

        while !sensor.data_ready().await.unwrap() {
            Timer::after_millis(250).await;
        }

        let sample = sensor.measurement().await.expect("Should meassure co2");

        let mut buf = [0u8; 64];
        let temp_text = format_no_std::show(
            &mut buf,
            format_args!("{:.1}°C     ", sample.temperature + TEMP_OFFSET),
        )
        .unwrap();
        draw_text(&mut display, temp_text, 5, 0);

        let humidity_text =
            format_no_std::show(&mut buf, format_args!("{:.0}%H", sample.humidity)).unwrap();
        draw_text(&mut display, humidity_text, 210, 0);

        let co2 = sample.co2 + CO2_OFFSET;
        let text = format_no_std::show(&mut buf, format_args!("  {:.1}  ", co2)).unwrap();
        let style = MonoTextStyleBuilder::new()
            .font(&profont::PROFONT_24_POINT)
            .background_color(Color::White)
            .text_color(Color::Black)
            .build();
        let text_style = TextStyleBuilder::new()
            .baseline(Baseline::Middle)
            .alignment(embedded_graphics::text::Alignment::Center)
            .build();
        let _ =
            Text::with_text_style(text, Point::new(125, 61), style, text_style).draw(&mut display);

        let text = match co2 {
            0..=999 => ":)",
            1000..=1499 => ":|",
            _ => ":)",
        };
        let _ =
            Text::with_text_style(text, Point::new(125, 90), style, text_style).draw(&mut display);
        epd.update_and_display_frame(&mut spi_device, display.buffer(), &mut Delay)
            .unwrap();
    }
}

fn draw_text(display: &mut Display2in13, text: &str, x: i32, y: i32) {
    let style = MonoTextStyleBuilder::new()
        .font(&profont::PROFONT_18_POINT)
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
