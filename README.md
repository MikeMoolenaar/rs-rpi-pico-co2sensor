# Project template for rp2040-hal

## TODO
- [x] Test E-ink display
- [ ] Test cheap CO2 sensor
- [ ] Try Bluetooth or WIFI 

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
