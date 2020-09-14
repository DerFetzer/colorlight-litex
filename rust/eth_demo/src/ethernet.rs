use litex_pac::{ETHMAC, ETH_BUFFERS};

use smoltcp::phy::{self, DeviceCapabilities};
use smoltcp::time::Instant;
use smoltcp::{Error, Result};

use log::trace;

pub struct Eth {
    ethmac: ETHMAC,
    ethbuf: ETH_BUFFERS,
}

impl Eth {
    pub fn new(ethmac: ETHMAC, ethbuf: ETH_BUFFERS) -> Self {
        ethmac.sram_writer_ev_pending.write(unsafe { |w| w.bits(1) });
        ethmac.sram_reader_ev_pending.write(unsafe { |w| w.bits(1) });
        ethmac.sram_reader_slot.write(unsafe { |w| w.bits(0) });

        Eth {
            ethmac,
            ethbuf,
        }
    }
}

impl<'a> phy::Device<'a> for Eth {
    type RxToken = EthRxToken<'a>;
    type TxToken = EthTxToken<'a>;

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        if self.ethmac.sram_writer_ev_pending.read().bits() == 0 {
            return None;
        }
        Some((Self::RxToken { ethmac: &self.ethmac, ethbuf: &self.ethbuf },
              Self::TxToken { ethmac: &self.ethmac, ethbuf: &self.ethbuf }))
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        Some(Self::TxToken { ethmac: &self.ethmac, ethbuf: &self.ethbuf })
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 2048;
        caps.max_burst_size = Some(1);
        caps
    }
}

pub struct EthRxToken<'a> {
    ethmac: &'a ETHMAC,
    ethbuf: &'a ETH_BUFFERS,
}

impl<'a> phy::RxToken for EthRxToken<'a> {
    fn consume<R, F>(self, _timestamp: Instant, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> Result<R>,
    {
        #[link_section = ".main_ram"]
        static mut RX_BUFFER: [u8; 2048] = [0; 2048];

        unsafe {
            if self.ethmac.sram_writer_ev_pending.read().bits() == 0 {
                return Err(Error::Exhausted);
            }
            let slot = self.ethmac.sram_writer_slot.read().bits();
            let length = self.ethmac.sram_writer_length.read().bits();
            match slot {
                0 => {
                    for (i, elem) in self.ethbuf.rx_buffer_0.iter().enumerate() {
                        if i > length as usize {
                            break;
                        }
                        RX_BUFFER[i] = elem.read().bits();
                    }
                }
                1 => {
                    for (i, elem) in self.ethbuf.rx_buffer_1.iter().enumerate() {
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
            self.ethmac.sram_writer_ev_pending.write(|w| w.bits(1));
            result
        }
    }
}

pub struct EthTxToken<'a> {
    ethmac: &'a ETHMAC,
    ethbuf: &'a ETH_BUFFERS,
}

impl<'a> phy::TxToken for EthTxToken<'a> {
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> Result<R>,
    {
        #[link_section = ".main_ram"]
        static mut TX_BUFFER: [u8; 2048] = [0; 2048];
        static mut SLOT: u8 = 0;

        trace!("Wait for tx ready...");
        while self.ethmac.sram_reader_ready.read().bits() == 0 {}
        trace!("Tx ready...");
        let result = f(unsafe { &mut TX_BUFFER[..len] });
        unsafe {
            trace!("Tx slot: {}", SLOT);
            trace!("Tx length: {}", len);
            trace!("Tx bytes: {:?}", &TX_BUFFER[..len as usize]);
        }
        let current_slot = unsafe { SLOT };
        match current_slot {
            0 => {
                for (i, elem) in self.ethbuf.tx_buffer_0.iter().enumerate() {
                    if i > len {
                        break;
                    }
                    elem.write(|w| unsafe { w.bits(TX_BUFFER[i]) });
                }
            }
            1 => {
                for (i, elem) in self.ethbuf.tx_buffer_1.iter().enumerate() {
                    if i > len {
                        break;
                    }
                    elem.write(|w| unsafe { w.bits(TX_BUFFER[i]) });
                }
            }
            _ => return Err(Error::Exhausted),
        };
        self.ethmac.sram_reader_slot.write(unsafe { |w| w.bits(current_slot.into()) });
        self.ethmac.sram_reader_length.write(unsafe { |w| w.bits(len as u32) });
        self.ethmac.sram_reader_start.write(unsafe { |w| w.bits(1) });
        unsafe {
            SLOT = (SLOT + 1) % 2;
        }
        result
    }
}
