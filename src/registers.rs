pub struct Registers;

impl Registers {
    pub const CHIP_ID: u8 = 0x00;
    pub const ERR_REG: u8 = 0x02;
}

pub struct ErrRegBits;
impl ErrRegBits {
    pub const AUX_ERR: u8 = 1 << 7;
    pub const FIFO_ERR: u8 = 1 << 6;
    pub const INTERNAL_ERR: u8 = 0b0001_1110;
    pub const FATAL_ERR: u8 = 1;
}
