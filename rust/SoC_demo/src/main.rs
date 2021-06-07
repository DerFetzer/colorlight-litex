#![no_std]
#![no_main]
extern crate panic_halt;

extern crate riscv;
extern crate riscv_rt;
use litex_pac;
use riscv_rt::entry;

use riscv::register::{mcause, mie, mstatus};
use vexriscv::register::{vdci, vmim, vmip, vsim, vsip};

use riscv::interrupt::{self};
use core::cell::Cell;

mod ethernet;
mod gpio;
mod leds;
mod print;
mod pwm;
mod timer;

use crate::ethernet::Eth;
use gpio::Gpio;
use leds::{Leds, Leds2};
use pwm::Pwm;
use timer::{Timer, Timer2};


use managed::ManagedSlice;
use smoltcp::iface::{EthernetInterfaceBuilder, NeighborCache};
use smoltcp::socket::{
    SocketSet, TcpSocket, TcpSocketBuffer, UdpPacketMetadata, UdpSocket, UdpSocketBuffer,
};
use smoltcp::time::Duration;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr};

use crate::print::UartLogger;
use log::{debug, info, trace};

const SYSTEM_CLOCK_FREQUENCY: u32 = 60_000_000;

static mut interrupted:bool = false;
// state of the button interrupt. static mut allows (unsafe) communication between main and irq.


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
    let mut timer2 = Timer2::new(peripherals.TIMER2);

    let mut leds = Leds::new(peripherals.LEDS);
    let mut leds2 = Leds2::new(peripherals.LEDS2);

    let mut gpio = Gpio::new(peripherals.GPIO);

    let mut pwm = Pwm::new(peripherals.PWM);

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

    let mut data = [0;128];

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

    timer2.load(0);
    timer2.reload(SYSTEM_CLOCK_FREQUENCY / 2);
    timer2.enable();
    timer2.en_interrupt();

    pwm.set_period(1 << 15);
    pwm.set_value(1 << 13);

    gpio.set_interrupt_polarity(true);
    gpio.en_interrupt();


    unsafe {
        vmim::write(0xFFFF_FFFF); // 1010 for timer and gpio
        mstatus::set_mie();
        mie::set_mext();
    }

    info!("Main loop...");



    loop {
        match iface.poll(&mut socket_set, clock.elapsed()) {
            Ok(_) => {}
            Err(e) => {
                debug!("Poll error: {}", e);
            }
        }

        {
            let mut socket = socket_set.get::<TcpSocket>(tcp_server_handle);
            let mut yay = false;

            if !socket.is_active() && !socket.is_listening() {
                info!("Start listen...");
                socket.listen(1234).unwrap();
            }

            else if socket.can_send() && socket.can_recv() {
                let bytes_rcvd = socket.recv_slice(&mut data);
                info!("bytes rcvd: {:?}", bytes_rcvd.unwrap());
                socket.send_slice(&data).unwrap();

                if(&data[0..3] == "hi\n".as_bytes()) {socket.send_slice(b"Horrayy! :)\r\n").unwrap();}

                else if(&data[0..7] == "led on\n".as_bytes()) {
                    leds.on();
                    info!("turned led on");
                }
                else if(&data[0..8] == "led off\n".as_bytes()) {
                    leds.off();
                    info!("turned led off");
                }
                else if(&data[0..15] == "tell me a joke\n".as_bytes()) {
                    socket.send_slice(b"Was machen die Pilze auf der Pizza? Als Belag funghieren. Ha.\r\n").unwrap();
                }
                else if(&data[0..7] == "set pwm".as_bytes()) {
                    let value = as_u32_be(&data[9..13]);
                    pwm.set_value(value);
                    socket.send_slice(b"set pwm value.\r\n").unwrap();
                    info!("set pwm value to {}", value);
                }
                else {
                    socket.send_slice(b"I didn't understand, sorry...\r\n").unwrap();
                    info!("wtf is this other computer talking about??");
                }

                for elem in data.iter_mut() { *elem = 0; }
            }

            if socket.can_send() {
                unsafe {
                    if interrupted {
                        socket.send_slice(b"Someone pressed my Button!\r\n").unwrap();
                        info!("interrupt noticed");
                        interrupted=false;
                    }
                }
            }
        }

        {
            let mut socket = socket_set.get::<UdpSocket>(udp_server_handle);
            if !socket.is_open() {
                socket.bind(5678).unwrap()
            }

            match socket.recv() {
                Ok((data, endpoint)) => {
                    info!("Hello World");
                }
                Err(_) => debug!("Err"),
            };
        }

        match iface.poll_delay(&socket_set, clock.elapsed()) {
            Some(Duration { millis: 0 }) => {}
            Some(delay) => {
                trace!("sleeping for {} ms", delay);
                msleep(&mut timer, delay.total_millis() as u32);
                clock.advance(delay)
            }
            None => clock.advance(Duration::from_millis(1)),
        }
        trace!("Clock elapsed: {}", clock.elapsed());
    }
}

#[no_mangle]
fn MachineExternal() {
    let mc = mcause::read();
    let irqs_pending = vmip::read();
    if mc.is_exception() {};
    for irq_no in 0..32 {
        if irqs_pending & (1 << irq_no) != 0 {
            match isr_to_interrupt(irq_no) {
                Some(0) => system_tick(),
                Some(1) => button_irq(),
                _ => unsafe{vmim::write(vmim::read() ^ (1<<irq_no))} // disable non-impl. interrupt
            }
        }
    }
}

pub fn isr_to_interrupt(isr: u8) -> Option<u8> {
    if isr == (litex_pac::Interrupt::TIMER2 as u8) {return Some(0)}
    if isr == (litex_pac::Interrupt::GPIO as u8) {return Some(1)}

    None
}


// Main signal processing routine. Triggered by Timer2.
fn system_tick() {
    let peripherals = unsafe { litex_pac::Peripherals::steal() }; // steal all but only use the safe ones ;)
    let mut timer2 = Timer2::new(peripherals.TIMER2);
    let mut leds2 = Leds2::new(peripherals.LEDS2);
    leds2.toggle();
    timer2.clr_interrupt();
}

fn button_irq() {
    let peripherals = unsafe { litex_pac::Peripherals::steal() }; // steal all but only use the safe ones ;)

    let mut gpio = Gpio::new(peripherals.GPIO);
    gpio.clr_interrupt();
    unsafe {interrupted = true};
}


fn msleep(timer: &mut Timer, ms: u32) {
    timer.disable();

    timer.reload(0);
    timer.load(SYSTEM_CLOCK_FREQUENCY / 1_000 * ms);

    timer.enable();

    // Wait until the time has elapsed
    while timer.value() > 0 {
    }
    timer.disable();
}


fn as_u32_be(array: &[u8]) -> u32 {
    ((array[0] as u32) << 24) +
    ((array[1] as u32) << 16) +
    ((array[2] as u32) <<  8) +
    ((array[3] as u32) <<  0)
}

fn as_u32_le(array: &[u8]) -> u32 {
    ((array[0] as u32) <<  0) +
    ((array[1] as u32) <<  8) +
    ((array[2] as u32) << 16) +
    ((array[3] as u32) << 24)
}
