pub mod biscuit2 {
    pub fn get_steps() -> u8 {
        6
    }
    pub fn get_max_temp(step: u8) -> f32 {
        match step {
            1 => 100.0,
            2 => 150.0,
            3 => 700.0,
            4 => 1100.0,
            5 => 1100.0,
            0_u8 | 6_u8..=u8::MAX => 0.0,
        }
    }
    pub fn get_duration_step(step: u8) -> f32 {
        match step {
            1 => 3.0,
            2 => 2.0,
            3 => 5.5,
            4 => 1.5,
            5 => 0.5,
            0_u8 | 6_u8..=u8::MAX => 0.0,
        }
    }
}
pub mod biscuit {
    pub fn get_steps() -> u8 {
        4
    }
    pub fn get_max_temp(step: u8) -> f32 {
        match step {
            1 => 860.0,
            2 => 1040.0,
            3 => 1040.0,
            0_u8 | 4_u8..=u8::MAX => 0.0,
        }
    }
    pub fn get_duration_step(step: u8) -> f32 {
        match step {
            1 => 5.0,
            2 => 0.5,
            3 => 0.5,
            0_u8 | 4_u8..=u8::MAX => 0.0,
        }
    }
}
