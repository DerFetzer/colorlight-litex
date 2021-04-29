use litex_pac::ADC;

pub struct Adc {
    registers: ADC,
}

#[allow(dead_code)]
impl Adc {
    pub fn new(registers: ADC) -> Self {
        Self { registers }
    }

    pub fn read(&mut self) -> u32 {
        self.registers.adc_value.read().bits()
    }

}
