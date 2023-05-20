pub mod biscuit {
    pub fn get_steps() -> u8 {
        4
    }
    pub fn get_max_temp(step: u8) -> f32 {
        match step {
            1 => 600.0,
            2 => 1030.0,
            3 => 1030.0,
            0_u8 | 4_u8..=u8::MAX => 0.0,
        }
    }
    pub fn get_duration_step(step: u8) -> f32 {
        match step {
            1 => 10.0,
            2 => 4.3,
            3 => 0.08,
            0_u8 | 4_u8..=u8::MAX => 0.0,
        }
    }
}
//(programs::biscuit::get_max_temp(step) - programs::biscuit::get_max_temp(step - 1)) / (programs::biscuit::get_duration_step(step) * 60.0)
pub mod biscuit2 {
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
