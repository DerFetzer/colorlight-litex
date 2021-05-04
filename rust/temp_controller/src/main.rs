#![no_std]
#![no_main]
extern crate panic_halt;

extern crate riscv;
extern crate riscv_rt;
use litex_pac;
use riscv_rt::entry;

use riscv::register::{mcause, mie, mstatus};
use vexriscv::register::{vdci, vmim, vmip, vsim, vsip};

use riscv::interrupt::{self, Mutex};
use core::cell::Cell;

mod adc;
mod ethernet;
mod gpio;
mod leds;
mod print;
mod pwm;
mod timer;
mod iir;
mod dac;

use crate::ethernet::Eth;
use adc::Adc;
use gpio::Gpio;
use leds::{Leds, Leds2};
use pwm::Pwm;
use timer::{Timer, Timer2};
use dac::Dac;

use iir::Iir;

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


// static IIR: Mutex<Cell<Iir>> =  Mutex::new(Cell::new(Iir{
//     ba : [1<<30,1<<30,1<<30,0,0],
//     shift : 30,
//     xy : [0,0,0,0,0],
//     }));

static mut set_temp: i32 = 700_000_000;
static mut err: i32 = 0;
static mut drive: u32 = 0;

static mut iir: Iir = Iir{
    // ba : [12190614,-15880203,12190614,138662614,943580235],  // lowpass
    // shift : 31,

    // ba : [10926271,-19764559,10926271,67672592,1008157215],
    // shift : 31,
    // ba : [283852882,-553640919,283852882,142568885,190173743],
    // shift : 31,

    // ba : [138600,-270332,138600,69614,92858],       // highpass
    // shift : 20,

    // ba : [10691554,-21298801,10691554,13430401,1060395729],
    // shift : 31,
    // ba: [1<<12,0,0,0,0],
    // shift : 12,
    ba: [736909014,736184239,0,-1072668083,0],        // maybe PI controller?
    shift : 31,
    xy : [0,0,0,0,0],
    };




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

    let mut adc = Adc::new(peripherals.ADC);

    let mut gpio = Gpio::new(peripherals.GPIO);

    let mut pwm = Pwm::new(peripherals.PWM);

    let mut dac = Dac::new(peripherals.DAC);

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

    timer2.load(0);
    timer2.reload(SYSTEM_CLOCK_FREQUENCY / 5);
    timer2.enable();
    timer2.en_interrupt();

    pwm.set_period(1 << 15);
    pwm.set_value(1 << 13);

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

            if !socket.is_active() && !socket.is_listening() {
                info!("Start listen...");
                socket.listen(1234).unwrap();
            }

            if socket.can_send() {
                // info!("Can send...");
                unsafe{
                    info!("adc:\t{:?}", adc.read() as u32);
                    info!("error:\t{:?}", err as i32);
                    info!("drive:\t{:?}", drive as i32);
                }
                // socket.send_slice(b"Hello World!\r\n").unwrap();
                let adcval: u32 = adc.read();
                socket.send_slice(&adcval.to_be_bytes()).unwrap();
            }
        }

        {
            let mut socket = socket_set.get::<UdpSocket>(udp_server_handle);
            if !socket.is_open() {
                socket.bind(5678).unwrap()
            }

            match socket.recv() {
                Ok((data, endpoint)) => {

                    let data4 = [data[0]-48,data[1]-48,data[2]-48,data[3]-48];
                    let val = as_u32_be(&data4);
                    unsafe{ set_temp = val as i32;}
                    info!("udp:5678 data/value received: {:?}/{} from {}", data, val,endpoint);
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
        // unsafe {
        //     // let y = iir.tick(x as i32);
        //     // dac.set((1<<17)-1);
        //     info!("adc: {:?}", adc.read() as u32);
        //     // info!("ba: {:?}", iir.ba);
        //     // info!("xy: {:?}", iir.xy);
        // }
        // msleep(&mut timer, 1000 as u32);
        // leds.toggle_mask(0xf);
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
                Some(1) => system_tick(),
                _ => unsafe{vmim::write(vmim::read() ^ (1<<irq_no))} // disable non-impl. interrupt
            }
        }
    }
}

pub fn isr_to_interrupt(isr: u8) -> Option<u8> {
    if isr == (litex_pac::Interrupt::TIMER2 as u8) {
        return Some(1);
    }

    None
}


// Main signal processing routine. Triggered by Timer2.
fn system_tick() {
    let peripherals = unsafe { litex_pac::Peripherals::steal() }; // steal all but only use the safe ones ;)
    let mut timer2 = Timer2::new(peripherals.TIMER2);
    let mut leds2 = Leds2::new(peripherals.LEDS2);
    let mut dac = Dac::new(peripherals.DAC);
    let mut adc = Adc::new(peripherals.ADC);
    leds2.on();
    unsafe {
        err = set_temp - (adc.read() as i32);
        let y = iir.tick(err);
        if y < 0{
            drive = 0;
        } else {
            drive = (y as u32 >> 9);
        }
        // drive = (y as u32 >> 15);
        if drive > (1<<16) {
            drive = (1 << 16) - 1;
        }
        dac.set(drive);
        // dac.set(adc.read() as u32 >> 15 );
    }
    leds2.off();
    timer2.clr_interrupt();
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

fn as_u32_be(array: &[u8; 4]) -> u32 {
    ((array[0] as u32) << 24) +
    ((array[1] as u32) << 16) +
    ((array[2] as u32) <<  8) +
    ((array[3] as u32) <<  0)
}

fn as_u32_le(array: &[u8; 4]) -> u32 {
    ((array[0] as u32) <<  0) +
    ((array[1] as u32) <<  8) +
    ((array[2] as u32) << 16) +
    ((array[3] as u32) << 24)
}
