# Raspberry Pi Pico powered CO2 sensor
CO2 sensor with E-ink display and bluetoth on the Raspberry Pi Pico W. Using Rust and the Embassy embedded framework.


## TODO
- [x] Test E-ink display
- [x] Test cheap CO2 sensor
- [x] Test more expensive CO2 sensor
- [x] Rewrite to [embasy-rs](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/i2c_async.rs)
- [ ] Try [Bluetooth](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/bluetooth.rs) or WIFI 

## Install
This assumes you have the Raspbery Pi debug probe or another Pico with raspberrypi/debugprobe.
```sh
cargo install --locked probe-rs-tools

rustup target install thumbv6m-none-eabi
cargo install flip-link
```


## Run
```sh
cargo run --release
```
