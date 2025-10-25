ESPump
=======

ESPump is an ESP32-based project that utilizes a soil sensor, and a water pump for automatic irrigation with posting to MQTT.
- [Adafruit HUZZAH32](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-huzzah32-esp32-feather.pdf)
- [Adafruit STEMMA Soil Sensor](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-stemma-soil-sensor-i2c-capacitive-moisture-sensor.pdf)
- [Adafruit MOSFET Driver](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-mosfet-driver.pdf) for the [Submersible 3V DC Water Pump](https://www.adafruit.com/product/4546)


# Setting up environment
- Install [espup](https://github.com/esp-rs/espup)
  - `cargo install espup --locked`
- Move `.cargo/config.toml.example` to `.cargo/config.toml`, and check over/change the environment variables, including:

# Flashing the ESP32
- ESPup makes it pretty easy, just run `cargo run --release` with the ESP32 connected.

# Logging
ESPump uses defmt, so accessing the logs post-flash can be done via:
- `espflash monitor -L defmt --elf ./target/xtensa-esp32-none-elf/release/main`
