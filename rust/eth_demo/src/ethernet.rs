use litex_pac::{ETHMAC, ETH_BUFFERS};

use smoltcp::phy::{self, DeviceCapabilities};
use smoltcp::time::Instant;
use smoltcp::{Error, Result};

use log::trace;

// TODO: Fix super ugly and unsafe device implementation

pub struct Eth {}

impl Eth {
    pub fn new() -> Self {
        let ethmac = ETHMAC::ptr();
        unsafe {
            (*ethmac).sram_writer_ev_pending.write(|w| w.bits(1));
            (*ethmac).sram_reader_ev_pending.write(|w| w.bits(1));
            (*ethmac).sram_reader_slot.write(|w| w.bits(0));
        };

        Eth {}
    }
}

impl<'a> phy::Device<'a> for Eth {
    type RxToken = EthRxToken;
    type TxToken = EthTxToken;

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        let ethmac = ETHMAC::ptr();
        unsafe {
            if (*ethmac).sram_writer_ev_pending.read().bits() == 0 {
                return None;
            }
        }
        Some((Self::RxToken {}, Self::TxToken {}))
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        Some(Self::TxToken {})
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 2048;
        caps.max_burst_size = Some(1);
        caps
    }
}

pub struct EthRxToken {}

impl phy::RxToken for EthRxToken {
    fn consume<R, F>(self, _timestamp: Instant, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> Result<R>,
    {
        #[link_section = ".main_ram"]
        static mut RX_BUFFER: [u8; 2048] = [0; 2048];

        let ethmac = ETHMAC::ptr();
        let ethbuf = ETH_BUFFERS::ptr();
        unsafe {
            if (*ethmac).sram_writer_ev_pending.read().bits() == 0 {
                return Err(Error::Exhausted);
            }
            let slot = (*ethmac).sram_writer_slot.read().bits();
            let length = (*ethmac).sram_writer_length.read().bits();
            match slot {
                0 => {
                    for (i, elem) in (*ethbuf).rx_buffer_0.iter().enumerate() {
                        if i > length as usize {
                            break;
                        }
                        RX_BUFFER[i] = elem.read().bits();
                    }
                }
                1 => {
                    for (i, elem) in (*ethbuf).rx_buffer_1.iter().enumerate() {
                        if i > length as usize {
                            break;
                        }
                        RX_BUFFER[i] = elem.read().bits();
                    }
                }
                _ => return Err(Error::Exhausted),
            };

            trace!("Rx slot: {}", slot);
            trace!("Rx length: {}", length);
            trace!("Rx bytes: {:?}", &RX_BUFFER[..length as usize]);
            let result = f(&mut RX_BUFFER[..length as usize]);
            (*ethmac).sram_writer_ev_pending.write(|w| w.bits(1));
            result
        }
    }
}

pub struct EthTxToken {}

impl phy::TxToken for EthTxToken {
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> Result<R>,
    {
        #[link_section = ".main_ram"]
        static mut TX_BUFFER: [u8; 2048] = [0; 2048];
        static mut SLOT: u8 = 0;

        let ethmac = ETHMAC::ptr();
        unsafe {
            trace!("Wait for tx ready...");
            while (*ethmac).sram_reader_ready.read().bits() == 0 {}
            trace!("Tx ready...");
            let ethbuf = ETH_BUFFERS::ptr();
            let result = f(&mut TX_BUFFER[..len]);
            trace!("Tx slot: {}", SLOT);
            trace!("Tx length: {}", len);
            trace!("Tx bytes: {:?}", &TX_BUFFER[..len as usize]);
            match SLOT {
                0 => {
                    for (i, elem) in (*ethbuf).tx_buffer_0.iter().enumerate() {
                        if i > len {
                            break;
                        }
                        elem.write(|w| w.bits(TX_BUFFER[i]));
                    }
                }
                1 => {
                    for (i, elem) in (*ethbuf).tx_buffer_1.iter().enumerate() {
                        if i > len {
                            break;
                        }
                        elem.write(|w| w.bits(TX_BUFFER[i]));
                    }
                }
                _ => return Err(Error::Exhausted),
            };
            (*ethmac).sram_reader_slot.write(|w| w.bits(SLOT.into()));
            (*ethmac).sram_reader_length.write(|w| w.bits(len as u32));
            (*ethmac).sram_reader_start.write(|w| w.bits(1));
            SLOT = (SLOT + 1) % 2;
            result
        }
    }
}
