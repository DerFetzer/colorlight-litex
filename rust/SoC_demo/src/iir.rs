

// Biquadratic (BiQuad) Infinite Impulse Response (IIR) Filter.


/// Generic vector for integer IIR filter.
/// This struct is used to hold the x/y input/output data vector or the b/a coefficient
/// vector.
pub type Vec5 = [i32; 5];


/// Main IIR struct holds coefficient vector and a shift value which defines the fixed point position
pub struct Iir {
    pub ba: Vec5,   // b and a coeffitients can be changed. [b0,b1,b2,a1,a2]
    pub shift: i32, // shift for fixed point pos
    pub xy: Vec5,   // x and y internal filter states       [x0,x1,y0,y1,y2]
}

impl Iir {
    /// Filter tick. Takes a new inout sample and returns a new output sample.
    pub fn tick(&mut self, x0: i32) -> i32 {

        // shift in x0
        self.xy.copy_within(0..4, 1);
        self.xy[0] = x0;

        let y0 = 1 << ((self.shift) - 1);
        let y = &self.xy
            .iter()
            .zip(&self.ba)
            .map(|(xi, ai)| *xi as i64 * *ai as i64)
            .fold(y0, |y, xa| y + xa);

        self.xy[2] = (y >> self.shift) as i32;
        self.xy[2]
    }
}
