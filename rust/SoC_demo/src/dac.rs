use litex_pac::DAC;

pub struct Dac {
    registers: DAC,
}

#[allow(dead_code)]
impl Dac {
    pub fn new(registers: DAC) -> Self {
        Self { registers }
    }

    pub fn set(&mut self, value: u32) {
        unsafe{
            self.registers.val.write(|w| w.bits(value));
        }
    }
}
