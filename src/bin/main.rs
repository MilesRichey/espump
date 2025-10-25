#![no_std]
#![no_main]

extern crate alloc;

use esp_backtrace as _;
use esp_println as _;

use core::{cell::RefCell, net::Ipv4Addr};

use critical_section::Mutex;
use defmt::{Debug2Format, debug, error, info, warn};
use embassy_executor::Spawner;
use embassy_net::{Runner, StackResources, tcp::TcpSocket};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal::i2c::I2c as hali2c;
use embedded_hal_bus::i2c::CriticalSectionDevice;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    Blocking,
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig, Pull},
    i2c::master::{Config, I2c},
    ram,
    rng::Rng,
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_radio::{
    Controller,
    wifi::{
        ClientConfig, ModeConfig, ScanConfig, WifiController, WifiDevice, WifiEvent, WifiStaState,
    },
};
use espump::{
    SENSOR_TO_MQTT, TEMPERATURE_TO_MQTT, publish_discovery, publish_moisture, publish_temperature,
};
use rust_mqtt::{client::client::MqttClient, utils::rng_generator::CountingRng};

esp_bootloader_esp_idf::esp_app_desc!();

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const MQTT_BROKER: &str = env!("MQTT_BROKER");
const MQTT_PORT: u16 = 1883;
const MQTT_USERNAME: &str = env!("MQTT_USERNAME");
const MQTT_PASSWORD: &str = env!("MQTT_PASSWORD");

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    let esp_radio_ctrl = &*mk_static!(
        Controller<'static>,
        esp_radio::init().expect("Failed to initialize ESP radio - hardware issue")
    );

    let (controller, interfaces) =
        esp_radio::wifi::new(esp_radio_ctrl, peripherals.WIFI, Default::default())
            .expect("Failed to initialize WiFi - hardware issue");

    let wifi_interface = interfaces.sta;

    let config = embassy_net::Config::dhcpv4(Default::default());

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Setup Red LED - GPIO #13
    let led_pin = Output::new(
        peripherals.GPIO13,
        Level::Low,
        OutputConfig::default().with_pull(Pull::Down),
    );
    let led_pin = mk_static!(Output<'static>, led_pin);

    //
    // Setup I2C Bus (SDA:23, SCL: 22) @ 300kHz for:
    // - Adafruit stemma soil sensor with 300kHz frequency
    //
    let i2c_config = Config::default().with_frequency(Rate::from_khz(300));
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .expect("Failed to initialize I2C - check hardware configuration")
        .with_sda(peripherals.GPIO23)
        .with_scl(peripherals.GPIO22);
    let i2c_bus = mk_static!(
        Mutex<RefCell<I2c<'static, Blocking>>>,
        Mutex::new(RefCell::new(i2c))
    );

    let soil_sens = mk_static!(
        CriticalSectionDevice<'static, I2c<'static, Blocking>>,
        CriticalSectionDevice::new(i2c_bus)
    );

    //
    // Setup water pump on pin GPIO14
    //
    let pump_pin = Output::new(
        peripherals.GPIO14,
        Level::Low,
        OutputConfig::default().with_pull(Pull::Down),
    );
    let pump_pin = mk_static!(Output<'static>, pump_pin);

    //
    // Init network stack
    //
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    let stack = mk_static!(embassy_net::Stack<'static>, stack);

    spawner
        .spawn(connection(controller))
        .expect("Failed to spawn connection task - critical error");
    spawner
        .spawn(net_task(runner))
        .expect("Failed to spawn net_task - critical error");
    spawner
        .spawn(sensor_task(soil_sens, led_pin))
        .expect("Failed to spawn sensor_task - critical error");
    spawner
        .spawn(pump_task(pump_pin))
        .expect("Failed to spawn pump_task - critical error");
    spawner
        .spawn(mqtt_task(stack))
        .expect("Failed to spawn mqtt_task - critical error");

    info!("All tasks spawned successfully");

    // Wait for network to be ready
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {:?}", Debug2Format(&config.address));
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    // Validate configuration
    info!("Validating configuration...");
    let _broker_addr: Ipv4Addr = MQTT_BROKER
        .parse()
        .expect("MQTT_BROKER environment variable must be a valid IPv4 address");
    info!("Configuration validated successfully");

    // Main task just waits - sensor_task handles everything
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("start connection task");
    info!("Device capabilities: {:?}", controller.capabilities());
    loop {
        if esp_radio::wifi::sta_state() == WifiStaState::Connected {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            Timer::after(Duration::from_millis(5000)).await;
        }

        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(SSID.into())
                    .with_password(PASSWORD.into()),
            );

            if let Err(e) = controller.set_config(&client_config) {
                error!("Failed to set WiFi config: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await;
                continue;
            }

            info!("Starting wifi");
            if let Err(e) = controller.start_async().await {
                error!("Failed to start WiFi: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await;
                continue;
            }
            info!("Wifi started!");

            info!("Scan");
            let scan_config = ScanConfig::default().with_max(10);
            match controller.scan_with_config_async(scan_config).await {
                Ok(result) => {
                    for ap in result {
                        debug!("{:?}", ap);
                    }
                }
                Err(e) => {
                    warn!("WiFi scan failed: {:?}", e);
                    // Continue anyway, scan is optional
                }
            }
        }
        info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                error!("Failed to connect to wifi: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

#[embassy_executor::task]
async fn sensor_task(
    i2c: &'static mut CriticalSectionDevice<'static, I2c<'static, Blocking>>,
    led_pin: &'static mut Output<'static>,
) {
    // Stemma soil sensor I2C address is 0x36
    const STEMMA_ADDR: u8 = 0x36;

    // Seesaw protocol constants
    const SEESAW_TOUCH_BASE: u8 = 0x0F;
    const SEESAW_TOUCH_CHANNEL_OFFSET: u8 = 0x10;

    Timer::after(Duration::from_secs(2)).await; // Wait for sensor to power up

    // Test I2C communication first
    let mut data = [0u8; 2];
    info!("Testing I2C connection to sensor...");

    match i2c.write_read(
        STEMMA_ADDR,
        &[SEESAW_TOUCH_BASE, SEESAW_TOUCH_CHANNEL_OFFSET],
        &mut data,
    ) {
        Ok(_) => {
            let moisture = u16::from_be_bytes(data);
            info!("Sensor connected! Initial moisture reading: {}", moisture);
        }
        Err(e) => {
            error!("ERROR: Failed to communicate with sensor: {:?}", e);
            error!("Check wiring: SDA=GPIO23, SCL=GPIO22, Address=0x36");
        }
    }

    let mut ticker = Ticker::every(Duration::from_secs(15));
    let mut consecutive_failures = 0u8;
    const MAX_CONSECUTIVE_FAILURES: u8 = 5;

    loop {
        ticker.next().await;
        led_pin.set_high(); // Indicate reading

        // Read moisture from sensor
        match espump::read_moisture(i2c).await {
            Ok(moisture) => {
                consecutive_failures = 0; // Reset on success
                if !(200..=20000).contains(&moisture) {
                    warn!("Invalid moisture read, skipping... {}", moisture);
                    continue;
                }
                let moisture_percent = espump::moisture_to_percentage(moisture);
                info!(
                    "Moisture: (raw: {}, percent: {}%)",
                    moisture, moisture_percent
                );
                // Publish raw value for now to get more data
                if SENSOR_TO_MQTT.try_send(moisture).is_err() {
                    warn!("MQTT queue full, dropping moisture reading");
                }
            }
            Err(e) => {
                consecutive_failures += 1;
                error!(
                    "Failed to read moisture (failure {}/{}): {:?}",
                    consecutive_failures, MAX_CONSECUTIVE_FAILURES, e
                );

                if consecutive_failures >= MAX_CONSECUTIVE_FAILURES {
                    warn!(
                        "Sensor appears offline after {} consecutive failures, will keep retrying...",
                        MAX_CONSECUTIVE_FAILURES
                    );
                }
            }
        }

        Timer::after(Duration::from_millis(500)).await;

        // Read temperature from sensor
        match espump::read_temperature(i2c).await {
            Ok(temperature) => {
                info!("Temperature reading: {} °C", temperature);
                // Send to MQTT task
                if TEMPERATURE_TO_MQTT.try_send(temperature).is_err() {
                    warn!("MQTT queue full, dropping temperature reading");
                }
            }
            Err(e) => {
                error!("Failed to read temperature from sensor: {:?}", e);
            }
        }

        led_pin.set_low(); // Turn off indicator
    }
}

#[embassy_executor::task]
async fn pump_task(pump_pin: &'static mut Output<'static>) {
    loop {
        // Check for pump commands from MQTT
        let cmd = espump::MQTT_TO_PUMP.receive().await;
        match cmd {
            espump::PumpCmd::On => {
                pump_pin.set_high();
                info!("Pump turned ON");
                // Send state to MQTT channel
                if espump::PUMP_STATE_TO_MQTT.try_send(true).is_err() {
                    warn!("MQTT queue full, dropping pump state update");
                }
            }
            espump::PumpCmd::Off => {
                pump_pin.set_low();
                info!("Pump turned OFF");
                // Send state to MQTT channel
                if espump::PUMP_STATE_TO_MQTT.try_send(false).is_err() {
                    warn!("MQTT queue full, dropping pump state update");
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn mqtt_task(stack: &'static embassy_net::Stack<'static>) {
    // Wait for network
    loop {
        if stack.is_link_up()
            && let Some(_config) = stack.config_v4()
        {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("MQTT task: Network ready");

    let mut rx_buffer = [0; 2048];
    let mut tx_buffer = [0; 2048];

    loop {
        // Connect to MQTT broker
        let mut socket = TcpSocket::new(*stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(30)));

        let broker_addr: Ipv4Addr = match MQTT_BROKER.parse() {
            Ok(addr) => addr,
            Err(e) => {
                error!(
                    "Invalid MQTT_BROKER address '{}': {:?}",
                    MQTT_BROKER,
                    Debug2Format(&e)
                );
                Timer::after(Duration::from_secs(30)).await;
                continue;
            }
        };
        let remote_endpoint = (broker_addr, MQTT_PORT);

        info!("Connecting to MQTT broker...");
        if socket.connect(remote_endpoint).await.is_err() {
            error!("Failed to connect to MQTT broker");
            Timer::after(Duration::from_secs(5)).await;
            continue;
        }
        info!("MQTT connected!");

        let mut client_config: rust_mqtt::client::client_config::ClientConfig<'_, 5, CountingRng> =
            rust_mqtt::client::client_config::ClientConfig::new(
                rust_mqtt::client::client_config::MqttVersion::MQTTv5,
                CountingRng(20000),
            );
        client_config
            .add_max_subscribe_qos(rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1);
        client_config.add_client_id("esp32_plant");
        client_config.add_username(MQTT_USERNAME);
        client_config.add_password(MQTT_PASSWORD);
        client_config.max_packet_size = 1024;
        client_config.keep_alive = 60;

        let mut recv_buffer = [0; 1024];
        let mut write_buffer = [0; 1024];

        let mut client = MqttClient::<_, 5, _>::new(
            socket,
            &mut write_buffer,
            1024,
            &mut recv_buffer,
            1024,
            client_config,
        );

        match client.connect_to_broker().await {
            Ok(_) => info!("MQTT handshake successful!"),
            Err(e) => {
                error!("Failed MQTT handshake: {:?}", Debug2Format(&e));
                Timer::after(Duration::from_secs(5)).await;
                continue;
            }
        }

        // Publish HA discovery
        publish_discovery(&mut client).await;

        // Subscribe to pump command topic
        if let Err(e) = client.subscribe_to_topic(espump::PUMP_COMMAND_TOPIC).await {
            error!(
                "Failed to subscribe to pump command topic: {:?}",
                Debug2Format(&e)
            );
        }

        // Track last heartbeat time
        let mut last_heartbeat = embassy_time::Instant::now();
        const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(60);

        // Main MQTT loop
        'inner: loop {
            // Send heartbeat/availability if needed
            if last_heartbeat.elapsed() >= HEARTBEAT_INTERVAL {
                if let Err(e) = client
                    .send_message(
                        espump::AVAILABILITY_TOPIC,
                        b"online",
                        rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS0,
                        true,
                    )
                    .await
                {
                    warn!(
                        "Failed to publish heartbeat: {:?}, reconnecting...",
                        Debug2Format(&e)
                    );
                    break 'inner;
                }
                last_heartbeat = embassy_time::Instant::now();
            }

            // 1) Check for moisture readings from sensor task
            if let Ok(moisture) =
                embassy_time::with_timeout(Duration::from_millis(10), SENSOR_TO_MQTT.receive())
                    .await
                && !publish_moisture(&mut client, moisture).await
            {
                warn!("Failed to publish moisture, reconnecting...");
                break 'inner;
            }

            // 2) Check for temperature readings from sensor task
            if let Ok(temperature) =
                embassy_time::with_timeout(Duration::from_millis(10), TEMPERATURE_TO_MQTT.receive())
                    .await
                && !publish_temperature(&mut client, temperature).await
            {
                warn!("Failed to publish temperature, reconnecting...");
                break 'inner;
            }

            // 3) Check for pump state updates from pump task
            if let Ok(state) = embassy_time::with_timeout(
                Duration::from_millis(10),
                espump::PUMP_STATE_TO_MQTT.receive(),
            )
            .await
            {
                let payload = if state { &b"ON"[..] } else { &b"OFF"[..] };
                if let Err(e) = client
                    .send_message(
                        espump::PUMP_STATE_TOPIC,
                        payload,
                        rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS0,
                        true,
                    )
                    .await
                {
                    warn!(
                        "Failed to publish pump state: {:?}, reconnecting...",
                        Debug2Format(&e)
                    );
                    break 'inner;
                }
            }

            // 4) Poll the MQTT client for any incoming messages
            match embassy_time::with_timeout(Duration::from_millis(100), client.receive_message())
                .await
            {
                Ok(Ok((topic, payload))) => {
                    info!("Received MQTT message on topic: {}", topic);
                    // Handle pump command topic
                    if topic == espump::PUMP_COMMAND_TOPIC {
                        let s = core::str::from_utf8(payload)
                            .unwrap_or_default()
                            .trim()
                            .to_ascii_uppercase();

                        if s == "ON" || s == "1" {
                            if espump::MQTT_TO_PUMP.try_send(espump::PumpCmd::On).is_err() {
                                warn!("Pump command queue full, dropping ON command");
                            }
                            // Optimistic retained publish so HA sees immediate change
                            if let Err(e) = client
                                .send_message(
                                    espump::PUMP_STATE_TOPIC,
                                    b"ON",
                                    rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS0,
                                    true,
                                )
                                .await
                            {
                                warn!("Failed to publish pump state: {:?}", Debug2Format(&e));
                            }
                        } else {
                            if espump::MQTT_TO_PUMP.try_send(espump::PumpCmd::Off).is_err() {
                                warn!("Pump command queue full, dropping OFF command");
                            }
                            if let Err(e) = client
                                .send_message(
                                    espump::PUMP_STATE_TOPIC,
                                    b"OFF",
                                    rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS0,
                                    true,
                                )
                                .await
                            {
                                warn!("Failed to publish pump state: {:?}", Debug2Format(&e));
                            }
                        }
                    }
                }
                Ok(Err(e)) => {
                    warn!("MQTT receive error, reconnecting: {:?}", Debug2Format(&e));
                    break 'inner;
                }
                _ => {} // Timeout, continue
            }
        }
    }
}
