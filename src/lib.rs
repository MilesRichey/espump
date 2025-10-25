#![no_std]

extern crate alloc;

use alloc::format;

use defmt::{Debug2Format, error, info};
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use rand_core::RngCore;
use rust_mqtt::client::client::MqttClient;

//
// Error Types
//
#[derive(Debug, Clone, Copy)]
pub enum ESPumpError {
    SensorI2c,
    SensorTimeout,
    NetworkDisconnected,
    MqttPublishFailed,
    InvalidConfig,
}

//
// MQTT Consts
//
pub const MOISTURE_TOPIC: &str = "homeassistant/sensor/plant1/moisture/state";
pub const TEMPERATURE_TOPIC: &str = "homeassistant/sensor/plant1_temperature/state";
pub const PUMP_COMMAND_TOPIC: &str = "homeassistant/switch/pump/set";
pub const PUMP_STATE_TOPIC: &str = "homeassistant/switch/pump/state";
pub const AVAILABILITY_TOPIC: &str = "homeassistant/sensor/plant1/availability";
pub const MOISTURE_CONFIG_TOPIC: &str = "homeassistant/sensor/plant1_moisture/config";
pub const TEMPERATURE_CONFIG_TOPIC: &str = "homeassistant/sensor/plant1_temperature/config";
pub const PUMP_CONFIG_TOPIC: &str = "homeassistant/switch/plant1_pump/config";

///
/// ESPump project configuration
/// Currently just WiFi
///
pub struct ESPumpConfig {
    pub wifi_ssdid: &'static str,
    pub wifi_password: &'static str,
}

pub enum PumpCmd {
    On,
    Off,
}

pub static SENSOR_TO_MQTT: Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    u16,
    4,
> = Channel::new();
pub static TEMPERATURE_TO_MQTT: Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    f32,
    4,
> = Channel::new();
pub static MQTT_TO_PUMP: Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    PumpCmd,
    4,
> = Channel::new();
pub static PUMP_STATE_TO_MQTT: Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    bool,
    4,
> = Channel::new();

///
/// Publishes discovery for moisture/pump to MQTT
///
pub async fn publish_discovery<T, const MAX_PROPERTIES: usize, R>(
    client: &mut MqttClient<'_, T, MAX_PROPERTIES, R>,
) where
    T: embedded_io_async::Read + embedded_io_async::Write,
    R: RngCore,
{
    let moisture_config = br#"{"name":"Plant Moisture","unique_id":"plant1_moisture","state_topic":"homeassistant/sensor/plant1/moisture/state","availability_topic":"homeassistant/sensor/plant1/availability","unit_of_measurement":"%","device_class":"moisture","icon":"mdi:water-percent"}"#;

    if let Err(e) = client
        .send_message(
            MOISTURE_CONFIG_TOPIC,
            moisture_config,
            rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS0,
            true,
        )
        .await
    {
        error!(
            "Failed to publish moisture discovery: {:?}",
            Debug2Format(&e)
        );
    }

    let temperature_config = r#"{"name":"Plant Temperature","unique_id":"plant1_temperature","state_topic":"homeassistant/sensor/plant1_temperature/state","availability_topic":"homeassistant/sensor/plant1/availability","unit_of_measurement":"°C","device_class":"temperature","icon":"mdi:thermometer"}"#;

    if let Err(e) = client
        .send_message(
            TEMPERATURE_CONFIG_TOPIC,
            temperature_config.as_bytes(),
            rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS0,
            true,
        )
        .await
    {
        error!(
            "Failed to publish temperature discovery: {:?}",
            Debug2Format(&e)
        );
    }

    let pump_config = br#"{"name":"Water Pump","unique_id":"plant1_pump","command_topic":"homeassistant/switch/pump/set","state_topic":"homeassistant/switch/pump/state","availability_topic":"homeassistant/sensor/plant1/availability","icon":"mdi:water-pump"}"#;

    if let Err(e) = client
        .send_message(
            PUMP_CONFIG_TOPIC,
            pump_config,
            rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS0,
            true,
        )
        .await
    {
        error!("Failed to publish pump discovery: {:?}", Debug2Format(&e));
    }

    // Publish online status
    if let Err(e) = client
        .send_message(
            AVAILABILITY_TOPIC,
            b"online",
            rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS0,
            true,
        )
        .await
    {
        error!("Failed to publish availability: {:?}", Debug2Format(&e));
    }

    info!("Published HA discovery");
}

///
/// Publishes a given moisture to MQTT
///
pub async fn publish_moisture<T, const MAX_PROPERTIES: usize, R>(
    client: &mut MqttClient<'_, T, MAX_PROPERTIES, R>,
    moisture: u16,
) -> bool
where
    T: embedded_io_async::Read + embedded_io_async::Write,
    R: RngCore,
{
    let moisture_str = format!("{}", moisture);

    match client
        .send_message(
            MOISTURE_TOPIC,
            moisture_str.as_bytes(),
            rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS0,
            false,
        )
        .await
    {
        Ok(_) => {
            info!("Moisture ({}) published successfully", moisture);
            true
        }
        Err(e) => {
            error!("Failed to publish moisture: {:?}", Debug2Format(&e));
            false
        }
    }
}

///
/// Publishes a given temperature to MQTT
///
pub async fn publish_temperature<T, const MAX_PROPERTIES: usize, R>(
    client: &mut MqttClient<'_, T, MAX_PROPERTIES, R>,
    temperature: f32,
) -> bool
where
    T: embedded_io_async::Read + embedded_io_async::Write,
    R: RngCore,
{
    let temp_str = format!("{:.2}", temperature);

    match client
        .send_message(
            TEMPERATURE_TOPIC,
            temp_str.as_bytes(),
            rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS0,
            false,
        )
        .await
    {
        Ok(_) => {
            info!("Temperature ({}°C) published successfully", temperature);
            true
        }
        Err(e) => {
            error!("Failed to publish temperature: {:?}", Debug2Format(&e));
            false
        }
    }
}

///
/// Converts raw moisture reading to percentage (0-100)
///
pub fn moisture_to_percentage(raw: u16) -> u16 {
    // Calibration values based on Adafruit Stemma sensor:
    // Bone dry in air: ~200-400
    // Saturated in water: ~1000-2000
    const DRY_VALUE: u16 = 350; // 0% moisture (completely dry)
    const WET_VALUE: u16 = 1000; // 100% moisture (saturated)

    // Notes:
    // Adafruit docs say: 200 (very dry) -> 2000 (very wet)
    // Bone dry soil ~= 376
    // After watering, the reading seemed a bit sporadic
    // Directly after watering: 65535 -> 28 -> 551 -> 1016 -> 595 -> 600 (take average, or normalize somehow?)

    if raw <= DRY_VALUE {
        return 0;
    }
    if raw >= WET_VALUE {
        return 100;
    }

    // Linear interpolation: (raw - dry) / (wet - dry) * 100
    let percentage = ((raw - DRY_VALUE) as u32 * 100) / ((WET_VALUE - DRY_VALUE) as u32);
    percentage as u16
}

///
/// Reads moisture from the I2C sensor and returns percentage (0-100)
///
pub async fn read_moisture<I>(soil_sens: &mut I) -> Result<u16, I::Error>
where
    I: embedded_hal::i2c::I2c,
{
    const STEMMA_ADDR: u8 = 0x36;

    // Seesaw protocol constants
    const SEESAW_TOUCH_BASE: u8 = 0x0F;
    const SEESAW_TOUCH_CHANNEL_OFFSET: u8 = 0x10;

    let mut data = [0u8; 2];

    match soil_sens.write(
        STEMMA_ADDR,
        &[SEESAW_TOUCH_BASE, SEESAW_TOUCH_CHANNEL_OFFSET],
    ) {
        Ok(_) => {
            Timer::after(Duration::from_millis(10)).await;

            match soil_sens.read(STEMMA_ADDR, &mut data) {
                Ok(_) => {
                    let raw_moisture = u16::from_be_bytes(data);
                    Ok(raw_moisture)
                }
                Err(e) => Err(e),
            }
        }
        Err(e) => Err(e),
    }
}

///
/// Reads temperature from the I2C sensor
/// Returns temperature in Celsius
///
pub async fn read_temperature<I>(soil_sens: &mut I) -> Result<f32, I::Error>
where
    I: embedded_hal::i2c::I2c,
{
    const STEMMA_ADDR: u8 = 0x36;

    // Seesaw protocol constants for temperature
    const SEESAW_STATUS_BASE: u8 = 0x00;
    const SEESAW_STATUS_TEMP: u8 = 0x04;

    let mut data = [0u8; 4];

    match soil_sens.write(STEMMA_ADDR, &[SEESAW_STATUS_BASE, SEESAW_STATUS_TEMP]) {
        Ok(_) => {
            // Wait for sensor to process the request
            Timer::after(Duration::from_millis(10)).await;

            match soil_sens.read(STEMMA_ADDR, &mut data) {
                Ok(_) => {
                    // Temperature is returned as a 32-bit big-endian fixed-point value
                    let raw_temp = u32::from_be_bytes(data);
                    // Convert to Celsius by dividing by 65536.0
                    let temp_celsius = (raw_temp as f32) / 65536.0;
                    let temp_celsius = temp_celsius - 3.0; // Sensor seems high by ~3 C
                    Ok(temp_celsius)
                }
                Err(e) => Err(e),
            }
        }
        Err(e) => Err(e),
    }
}
