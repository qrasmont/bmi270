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

pub struct EventBits;
impl EventBits {
    pub const ERR_CODE: u8 = 0b0001_1100;
    pub const POR_DETECTED: u8 = 1;
}

pub struct PersistentErrVal;
impl PersistentErrVal {
    pub const NO_ERR: u8 = 0x00;
    pub const ACC_ERR: u8 = 0x01;
    pub const GYR_ERR: u8 = 0x02;
    pub const ACC_GYR_ERR: u8 = 0x03;
}

pub struct InterruptStatus0Bits;
impl InterruptStatus0Bits {
    pub const SIG_MOTION_OUT: u8 = 1;
    pub const STEP_COUNTER_OUT: u8 = 1 << 1;
    pub const ACTIVITY_OUT: u8 = 1 << 2;
    pub const WRIST_WEAR_WAKEUP_OUT: u8 = 1 << 3;
    pub const WRIST_GESTURE_OUT: u8 = 1 << 4;
    pub const NO_MOTION_OUT: u8 = 1 << 5;
    pub const ANY_MOTION_OUT: u8 = 1 << 6;
}

pub struct InterruptStatus1Bits;
impl InterruptStatus1Bits {
    pub const FFULL_INT: u8 = 1;
    pub const FWM_INT: u8 = 1 << 1;
    pub const ERR_INT: u8 = 1 << 2;
    pub const AUX_DRDY_INT: u8 = 1 << 5;
    pub const GYR_DRDY_INT: u8 = 1 << 6;
    pub const ACC_DRDY_INT: u8 = 1 << 7;
}
