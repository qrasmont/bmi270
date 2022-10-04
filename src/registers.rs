pub struct Registers;
impl Registers {
    pub const CHIP_ID: u8 = 0x00;
    pub const ERR_REG: u8 = 0x02;
    pub const STATUS: u8 = 0x03;
    pub const AUX_DATA_0: u8 = 0x04;
    pub const ACC_DATA_0: u8 = 0x0C;
    pub const GYR_DATA_0: u8 = 0x12;
    pub const SENSORTIME_0: u8 = 0x18;
    pub const EVENT: u8 = 0x1B;
    pub const INT_STATUS_0: u8 = 0x1C;
    pub const INT_STATUS_1: u8 = 0x1D;
    pub const SC_OUT_0: u8 = 0x1E;
    pub const WR_GEST_ACT: u8 = 0x20;
    pub const INTERNAL_STATUS: u8 = 0x21;
    pub const TEMPERATURE_0: u8 = 0x22;
    pub const FIFO_LENGTH_0: u8 = 0x24;
    pub const FIFO_DATA: u8 = 0x26;
    pub const ACC_CONF: u8 = 0x40;
    pub const ACC_RANGE: u8 = 0x41;
}
