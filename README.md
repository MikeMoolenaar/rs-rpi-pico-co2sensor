# Project template for rp2040-hal

## TODO
- [x] Test E-ink display
- [x] Test cheap CO2 sensor
- [x] Test more expensive CO2 sensor
- [ ] Rewrite to [embasy-rs](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/i2c_async.rs)
- [ ] Try [Bluetooth](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/bluetooth.rs) or WIFI 

## Install
```sh
# Install picotool using your favo package manager, example for arch:
paru -S picotool

rustup target install thumbv6m-none-eabi
cargo install flip-link
```


## Run
Disconnect PICO, hold BOOTSTEL button while reconnecting again via USB.
```sh
cargo run --release
```
