#![no_std]
#![no_main]
extern crate panic_halt;


extern crate riscv;
extern crate riscv_rt;
use litex_pac;
use riscv_rt::entry;

use vexriscv::register::{vdci, vmim, vmip, vsim, vsip};
use riscv::register::{mstatus, mcause, mie};


mod ethernet;
mod print;
mod timer;
mod leds;
mod adc;
mod gpio;

use crate::ethernet::Eth;
use timer::Timer;
use leds::Leds;
use leds::Leds2;
use adc::Adc;
use gpio::Gpio;

use managed::ManagedSlice;
use smoltcp::iface::{EthernetInterfaceBuilder, NeighborCache};
use smoltcp::socket::{
    SocketSet, TcpSocket, TcpSocketBuffer, UdpPacketMetadata, UdpSocket, UdpSocketBuffer,
};
use smoltcp::time::Duration;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr};

use crate::print::UartLogger;
use log::{debug, info, trace};



const SYSTEM_CLOCK_FREQUENCY: u32 = 49_600_000;



mod mock {
    use core::cell::Cell;
    use smoltcp::time::{Duration, Instant};

    #[derive(Debug)]
    pub struct Clock(Cell<Instant>);

    impl Clock {
        pub fn new() -> Clock {
            Clock(Cell::new(Instant::from_millis(0)))
        }

        pub fn advance(&self, duration: Duration) {
            self.0.set(self.0.get() + duration)
        }

        pub fn elapsed(&self) -> Instant {
            self.0.get()
        }
    }
}

// This is the entry point for the application.
// It is not allowed to return.
#[entry]
fn main() -> ! {
    let peripherals = litex_pac::Peripherals::take().unwrap();

    print::print_hardware::set_hardware(peripherals.UART);

    UartLogger::init().unwrap();

    info!("Logger initialized");

    let mut timer = Timer::new(peripherals.TIMER0);
    let mut timer2 = Timer::new(peripherals.TIMER2);

    let mut leds = Leds::new(peripherals.LEDS);

    let mut adc = Adc::new(peripherals.ADC);

    let mut gpio = Gpio::new(peripherals.GPIO);

    let clock = mock::Clock::new();
    let device = Eth::new(peripherals.ETHMAC, peripherals.ETHMEM);

    let mut neighbor_cache_entries = [None; 8];
    let neighbor_cache = NeighborCache::new(&mut neighbor_cache_entries[..]);
    let mut ip_addr = [IpCidr::new(IpAddress::v4(192, 168, 1, 50), 24)];
    let ip_addrs = ManagedSlice::Borrowed(&mut ip_addr);
    let mut iface = EthernetInterfaceBuilder::new(device)
        .ethernet_addr(EthernetAddress::from_bytes(&[
            0xF6, 0x48, 0x74, 0xC8, 0xC4, 0x83,
        ]))
        .neighbor_cache(neighbor_cache)
        .ip_addrs(ip_addrs)
        .finalize();

    let tcp_server_socket = {
        // It is not strictly necessary to use a `static mut` and unsafe code here, but
        // on embedded systems that smoltcp targets it is far better to allocate the data
        // statically to verify that it fits into RAM rather than get undefined behavior
        // when stack overflows.
        #[link_section = ".main_ram"]
        static mut TCP_SERVER_RX_DATA: [u8; 8192] = [0; 8192];
        #[link_section = ".main_ram"]
        static mut TCP_SERVER_TX_DATA: [u8; 8192] = [0; 8192];
        let tcp_rx_buffer = TcpSocketBuffer::new(unsafe { &mut TCP_SERVER_RX_DATA[..] });
        let tcp_tx_buffer = TcpSocketBuffer::new(unsafe { &mut TCP_SERVER_TX_DATA[..] });
        TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer)
    };

    let udp_server_socket = {
        // It is not strictly necessary to use a `static mut` and unsafe code here, but
        // on embedded systems that smoltcp targets it is far better to allocate the data
        // statically to verify that it fits into RAM rather than get undefined behavior
        // when stack overflows.
        #[link_section = ".main_ram"]
        static mut UDP_SERVER_RX_METADATA: [UdpPacketMetadata; 32] = [UdpPacketMetadata::EMPTY; 32];
        #[link_section = ".main_ram"]
        static mut UDP_SERVER_TX_METADATA: [UdpPacketMetadata; 32] = [UdpPacketMetadata::EMPTY; 32];
        #[link_section = ".main_ram"]
        static mut UDP_SERVER_RX_DATA: [u8; 8192] = [0; 8192];
        #[link_section = ".main_ram"]
        static mut UDP_SERVER_TX_DATA: [u8; 8192] = [0; 8192];
        let udp_rx_buffer =
            UdpSocketBuffer::new(unsafe { &mut UDP_SERVER_RX_METADATA[..] }, unsafe {
                &mut UDP_SERVER_RX_DATA[..]
            });
        let udp_tx_buffer =
            UdpSocketBuffer::new(unsafe { &mut UDP_SERVER_TX_METADATA[..] }, unsafe {
                &mut UDP_SERVER_TX_DATA[..]
            });
        UdpSocket::new(udp_rx_buffer, udp_tx_buffer)
    };

    info!("Add sockets");

    let mut socket_set_entries: [_; 2] = Default::default();
    let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let tcp_server_handle = socket_set.add(tcp_server_socket);
    let udp_server_handle = socket_set.add(udp_server_socket);

    timer2.load(SYSTEM_CLOCK_FREQUENCY / 1_000 * 4000);

    timer2.enable();

    timer2.en_interrupt();


    info!("Main loop...");

    loop {
        info!("{}", timer2.value());
    }
}


#[no_mangle]
fn MachineExternal() {
    let mc = mcause::read();
    let irqs_pending = vmip::read();
    if mc.is_exception() {};
}

// fn msleep(timer: &mut Timer, leds : &mut Leds, ms: u32) {
//     timer.disable();
//
//     timer.reload(0);
//     timer.load(SYSTEM_CLOCK_FREQUENCY / 1_000 * ms);
//
//     timer.enable();
//
//     // Wait until the time has elapsed
//     while timer.value() > 0 {
//     }
//     timer.disable();
// }
