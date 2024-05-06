#![no_std]
#![no_main]

use core::str::FromStr;

use cyw43_pio::PioSpi;
use defmt::{debug, trace, unwrap, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_futures::select::Either::{First, Second};
use embassy_net::dns::DnsQueryType;
use embassy_net::tcp;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, DhcpConfig, Stack, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_25, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{with_timeout, Duration, Timer};
use heapless::String;
use immerbell::led;
use panic_probe as _;
use rand::RngCore;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<
        'static,
        Output<'static, PIN_23>,
        PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>,
    >,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::task]
async fn led_task(blinker: led::Blinker<'static>) -> ! {
    blinker.run().await
}

#[embassy_executor::task]
async fn notify_task(worker: &'static mut Worker<'static>) -> ! {
    worker.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut rng = RoscRng;

    let fw = include_bytes!("/opt/embassy/cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("/opt/embassy/cyw43-firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmware independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download 43439A0_clm.bin --format bin --chip RP2040 --base-address 0x10140000
    // let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    // let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_driver, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(wifi_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let mut config = DhcpConfig::default();
    let client_host = env!("CLIENT_HOST");
    config.hostname = Some(unwrap!(String::from_str(client_host)));

    let config = Config::dhcpv4(config);
    let seed = rng.next_u64();

    static STACK: StaticCell<Stack<cyw43::NetDriver<'static>>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let resources = RESOURCES.init(StackResources::new());
    let stack = STACK.init(Stack::new(net_driver, config, resources, seed));
    unwrap!(spawner.spawn(net_task(stack)));

    let ssid = unwrap!(option_env!("WIFI_NETWORK"));
    let pass = unwrap!(option_env!("WIFI_PASSWORD"));
    while let Err(e) = control.join_wpa2(ssid, pass).await {
        warn!("join failed with status={}", e.status);
    }

    static LED_SIGNAL: Signal<CriticalSectionRawMutex, led::Mode> = Signal::new();
    let blinker = led::Blinker::new(&LED_SIGNAL, control);
    unwrap!(spawner.spawn(led_task(blinker)));

    static WORKER: StaticCell<Worker> = StaticCell::new();
    let input = Input::new(AnyPin::from(p.PIN_2), Pull::Up);
    let server_host = env!("SERVER_HOST");
    let port = unwrap!(env!("PORT").parse().ok());
    let worker = WORKER.init(Worker::new(stack, input, server_host, port, &LED_SIGNAL));
    unwrap!(spawner.spawn(notify_task(worker)));
}

struct Worker<'a> {
    stack: &'a Stack<cyw43::NetDriver<'a>>,
    input: Input<'a, AnyPin>,
    server_host: &'a str,
    port: u16,
    led: &'a Signal<CriticalSectionRawMutex, led::Mode>,
}

impl<'a> Worker<'a> {
    pub fn new(
        stack: &'a Stack<cyw43::NetDriver<'a>>,
        input: Input<'a, AnyPin>,
        server_host: &'a str,
        port: u16,
        led: &'a Signal<CriticalSectionRawMutex, led::Mode>,
    ) -> Self {
        Self {
            stack,
            input,
            server_host,
            port,
            led,
        }
    }

    pub async fn run(&mut self) -> ! {
        loop {
            use led::Mode::{Off, On, OnOff};

            trace!("waiting for IP configuration...");
            self.led.signal(On);

            let future = self.stack.wait_config_up();
            let client_address = match with_timeout(Duration::from_secs(10), future).await {
                Ok(_) => unwrap!(self.stack.config_v4()).address.address(),
                Err(_) => {
                    self.led.signal(Off);
                    warn!("timed out");
                    continue;
                }
            };

            debug!("client IP address is {:?}", client_address);

            trace!("resolving host {}...", self.server_host);
            self.led.signal(OnOff(
                Duration::from_millis(150),
                Duration::from_millis(350),
            ));

            let future = self.stack.dns_query(self.server_host, DnsQueryType::A);
            let server_address = match with_timeout(Duration::from_secs(10), future).await {
                Ok(Ok(mut r)) => unwrap!(r.pop()),
                Ok(Err(e)) => {
                    warn!("query error: {:?}", e);
                    self.led.signal(Off);
                    Timer::after_millis(500).await;
                    continue;
                }
                Err(_) => {
                    warn!("timed out");
                    continue;
                }
            };

            debug!("server IP address is {:?}", server_address);

            trace!("waiting for low on GPIO pin...");
            self.led.signal(OnOff(
                Duration::from_millis(600),
                Duration::from_millis(2400),
            ));

            match select(self.input.wait_for_low(), Timer::after_secs(1200)).await {
                First(_) => (),
                Second(_) => {
                    trace!("worker is going to refresh");
                    continue;
                }
            }

            trace!("button is now pushed");
            self.led.signal(On);

            let mut rx_buffer = [0; 256];
            let mut tx_buffer = [0; 256];

            let mut socket = TcpSocket::new(self.stack, &mut rx_buffer, &mut tx_buffer);

            trace!("connecting...");
            self.led.signal(OnOff(
                Duration::from_millis(150),
                Duration::from_millis(350),
            ));

            let future = socket.connect((server_address, self.port));
            let remote_endpoint = match with_timeout(Duration::from_secs(10), future).await {
                Ok(Ok(_)) => unwrap!(socket.remote_endpoint()),
                Ok(Err(e)) => {
                    warn!("connect error: {:?}", e);
                    self.led.signal(Off);
                    Timer::after_millis(500).await;
                    continue;
                }
                Err(_) => {
                    warn!("timed out");
                    continue;
                }
            };

            trace!("connected to {:?}", remote_endpoint);
            self.led.signal(On);

            match self.begin_activity(&mut socket).await {
                Ok(_) => {
                    trace!("closing connection...");
                    socket.close();
                }
                Err(e) => {
                    warn!("write error: {:?}", e);
                    self.led.signal(Off);
                    Timer::after_millis(500).await;
                    continue;
                }
            };

            trace!("connection closed");
            self.led.signal(Off);
        }
    }

    async fn begin_activity(&mut self, socket: &mut TcpSocket<'_>) -> Result<(), tcp::Error> {
        loop {
            use led::Mode::{Off, OffOn, On, OnOff};

            trace!("sending command for ringing...");
            while socket.write(&[0x01]).await? != 1 {
                warn!("write failed");
            }

            trace!("waiting for high on GPIO pin...");
            self.led
                .signal(OffOn(Duration::from_hz(50), Duration::from_hz(50)));

            let future = self.input.wait_for_high();
            if with_timeout(Duration::from_secs(10), future).await.is_err() {
                trace!("ending session due to inactivity...");
                while socket.write(&[0xFF]).await? != 1 {
                    warn!("write failed");
                }
                Timer::after_millis(75).await;
                return Ok(());
            }

            trace!("button is no longer pushed");
            self.led.signal(Off);

            trace!("sending command to stop ringing...");
            while socket.write(&[0x00]).await? != 1 {
                warn!("write failed");
            }

            trace!("waiting for low on GPIO pin...");
            self.led.signal(OnOff(
                Duration::from_millis(150),
                Duration::from_millis(350),
            ));

            let future = self.input.wait_for_low();
            if with_timeout(Duration::from_secs(10), future).await.is_err() {
                trace!("ending session due to inactivity...");
                while socket.write(&[0xFF]).await? != 1 {
                    warn!("write failed");
                }
                Timer::after_millis(75).await;
                return Ok(());
            }

            trace!("button is now pushed again");
            self.led.signal(On);
        }
    }
}
