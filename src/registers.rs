pub struct Registers;

impl Registers {
    pub const CHIP_ID: u8 = 0x00;
    pub const ERR_REG: u8 = 0x02;
    pub const STATUS: u8 = 0x03;
    pub const AUX_DATA_0: u8 = 0x04;
    pub const ACC_DATA_0: u8 = 0x0C;
    pub const GYR_DATA_0: u8 = 0x12;
    pub const SENSORTIME_0: u8 = 0x18;
}

pub struct ErrRegBits;
impl ErrRegBits {
    pub const AUX_ERR: u8 = 1 << 7;
    pub const FIFO_ERR: u8 = 1 << 6;
    pub const INTERNAL_ERR: u8 = 0b0001_1110;
    pub const FATAL_ERR: u8 = 1;
}

pub struct StatusBits;
impl StatusBits {
    pub const DRDY_ACC: u8 = 1 << 7;
    pub const DRDY_GYR: u8 = 1 << 6;
    pub const DRDY_AUX: u8 = 1 << 5;
    pub const CMD_RDY: u8 = 1 << 4;
    pub const AUX_BUSY: u8 = 1 << 2;
}
