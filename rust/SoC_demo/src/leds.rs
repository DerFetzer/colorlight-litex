use litex_pac::{LEDS, LEDS2};

pub struct Leds {
    registers: LEDS,
}

#[allow(dead_code)]
impl Leds {
    pub fn new(registers: LEDS) -> Self {
        Self { registers }
    }

    pub fn set_single(&mut self, red: bool, yellow: bool, green: bool) {
        self.registers.out.write(|w| {
            w.r().bit(red);
            w.g().bit(green);
            w.y().bit(yellow)
        });
    }

    pub fn set(&mut self, leds: u32) {
        unsafe {
            self.registers.out.write(|w| w.bits(leds));
        }
    }

    pub fn off(&mut self) {
        unsafe {
            self.registers.out.write(|w| w.bits(0x0000_0000));
        }
    }

    pub fn on(&mut self) {
        unsafe {
            self.registers.out.write(|w| w.bits(0xFFFF_FFFF));
        }
    }

    pub fn toggle(&mut self) {
        self.toggle_mask(0xFFFF_FFFF);
    }

    pub fn toggle_mask(&mut self, mask: u32) {
        let val: u32 = self.registers.out.read().bits() ^ mask;
        unsafe {
            self.registers.out.write(|w| w.bits(val));
        }
    }
}

pub struct Leds2 {
    registers: LEDS2,
}

#[allow(dead_code)]
impl Leds2 {
    pub fn new(registers: LEDS2) -> Self {
        Self { registers }
    }

    pub fn set_single(&mut self, red: bool, yellow: bool, green: bool) {
        self.registers.out.write(|w| {
            w.r1().bit(red);
            w.g1().bit(green);
            w.y1().bit(yellow)
        });
    }

    pub fn set(&mut self, leds: u32) {
        unsafe {
            self.registers.out.write(|w| w.bits(leds));
        }
    }

    pub fn off(&mut self) {
        unsafe {
            self.registers.out.write(|w| w.bits(0x0000_0000));
        }
    }

    pub fn on(&mut self) {
        unsafe {
            self.registers.out.write(|w| w.bits(0xFFFF_FFFF));
        }
    }

    pub fn toggle(&mut self) {
        self.toggle_mask(0xFFFF_FFFF);
    }

    pub fn toggle_mask(&mut self, mask: u32) {
        let val: u32 = self.registers.out.read().bits() ^ mask;
        unsafe {
            self.registers.out.write(|w| w.bits(val));
        }
    }
}
