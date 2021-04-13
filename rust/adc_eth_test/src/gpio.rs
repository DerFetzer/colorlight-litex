use litex_pac::GPIO;

pub struct Gpio {
    registers: GPIO,
}

#[allow(dead_code)]
impl Gpio {

    pub fn new(registers: GPIO) -> Self {
        Self { registers }
    }

    pub fn status(&mut self) -> u32 {
        self.registers.in_.read().bits()
    }

    pub fn set_interrupt_polarity(&mut self, pol: bool) {
        unsafe {
            if pol {
                self.registers.polarity.write(|w| w.bits(1));
            }else {
                self.registers.polarity.write(|w| w.bits(0));
            }
        }
    }

    pub fn en_interrupt(&mut self) {
        unsafe {
            self.registers.ev_enable.write(|w| w.bits(0xFFFF_FFFF));
        }
    }

    pub fn clr_interrupt(&mut self) {
        unsafe {
            self.registers.ev_pending.write(|w| w.bits(0xFFFF_FFFF));
        }
    }

    pub fn dis_interrupt(&mut self) {
        unsafe {
            self.registers.ev_enable.write(|w| w.bits(0));
        }
    }

}
