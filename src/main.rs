#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    prelude::*,
    text::{Baseline, Text, TextStyleBuilder},
};
use panic_probe as _;

use epd_waveshare::{epd2in13_v2::*, prelude::*};

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::fugit::RateExtU32;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio, pac,
    sio::Sio,
    spi,
    watchdog::Watchdog,
    Timer,
};
use embedded_hal_bus::spi::ExclusiveDevice;

#[entry]
#[allow(clippy::empty_loop)]
fn main() -> ! {
    info!("Program start");
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
    let pin_dc = pins.gpio1.into_push_pull_output();
    let pin_rst = pins.gpio0.into_push_pull_output();
    let spi_mosi = pins.gpio11.into_function::<gpio::FunctionSpi>();
    let spi_sclk = pins.gpio10.into_function::<gpio::FunctionSpi>();
    let spi = spi::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_sclk));

    // Init SPI
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

    // Use display graphics from embedded-graphics and draw text
    let mut display = Display2in13::default();
    display.clear(Color::White).ok();
    draw_text(&mut display, "Hello from Rust", 5, 50);

    // Update screen, then sleep
    epd.update_frame(&mut spi_device, display.buffer(), &mut timer)
        .unwrap();
    epd.display_frame(&mut spi_device, &mut timer).unwrap();
    epd.sleep(&mut spi_device, &mut timer).unwrap();

    // TODO: use Wifi here
    loop {}
}

fn draw_text(display: &mut Display2in13, text: &str, x: i32, y: i32) {
    let style = MonoTextStyleBuilder::new()
        .font(&embedded_graphics::mono_font::ascii::FONT_6X10)
        .text_color(Color::Black)
        .build();

    let text_style = TextStyleBuilder::new().baseline(Baseline::Top).build();

    let _ = Text::with_text_style(text, Point::new(x, y), style, text_style).draw(display);
}
