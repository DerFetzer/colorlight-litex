use litex_pac::PWM;

pub struct Pwm {
    registers: PWM,
}

impl Pwm {
    pub fn new(registers: PWM) -> Self {
        Self { registers }
    }

    pub fn set_period(&mut self, period: u32) {
        unsafe {
            self.registers.period.write(|w| w.bits(period));
        }
    }

    pub fn set_value(&mut self, val: u32) {
        unsafe {
            self.registers.value.write(|w| w.bits(val));
        }
    }
}
