use litex_pac::{TIMER0, TIMER2};

pub struct Timer {
    registers: TIMER0,
}

impl Timer {
    pub fn new(registers: TIMER0) -> Self {
        Self { registers }
    }

    pub fn enable(&mut self) {
        unsafe {
            self.registers.en.write(|w| w.bits(1));
        }
    }

    pub fn disable(&mut self) {
        unsafe {
            self.registers.en.write(|w| w.bits(0));
        }
    }

    pub fn load(&mut self, value: u32) {
        unsafe {
            self.registers.load.write(|w| w.bits(value));
        }
    }

    pub fn reload(&mut self, value: u32) {
        unsafe {
            self.registers.reload.write(|w| w.bits(value));
        }
    }

    pub fn value(&mut self) -> u32 {
        unsafe {
            self.registers.update_value.write(|w| w.bits(1));
        }

        self.registers.value.read().bits()
    }

    pub fn en_interrupt(&mut self) {
        unsafe {
            self.registers.ev_enable.write(|w| w.bits(0x1));
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

    pub fn ev_pending(&mut self) -> u32 {
        self.registers.ev_pending.read().bits()
    }

    pub fn ev_status(&mut self) -> u32 {
        self.registers.ev_status.read().bits()
    }

    pub fn ev_enable(&mut self) -> u32 {
        self.registers.ev_enable.read().bits()
    }
}

pub struct Timer2 {
    registers: TIMER2,
}

impl Timer2 {
    pub fn new(registers: TIMER2) -> Self {
        Self { registers }
    }

    pub fn enable(&mut self) {
        unsafe {
            self.registers.en.write(|w| w.bits(1));
        }
    }

    pub fn disable(&mut self) {
        unsafe {
            self.registers.en.write(|w| w.bits(0));
        }
    }

    pub fn load(&mut self, value: u32) {
        unsafe {
            self.registers.load.write(|w| w.bits(value));
        }
    }

    pub fn reload(&mut self, value: u32) {
        unsafe {
            self.registers.reload.write(|w| w.bits(value));
        }
    }

    pub fn value(&mut self) -> u32 {
        unsafe {
            self.registers.update_value.write(|w| w.bits(1));
        }

        self.registers.value.read().bits()
    }

    pub fn en_interrupt(&mut self) {
        unsafe {
            self.registers.ev_enable.write(|w| w.bits(0x1));
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

    pub fn ev_pending(&mut self) -> u32 {
        self.registers.ev_pending.read().bits()
    }

    pub fn ev_status(&mut self) -> u32 {
        self.registers.ev_status.read().bits()
    }

    pub fn ev_enable(&mut self) -> u32 {
        self.registers.ev_enable.read().bits()
    }
}
