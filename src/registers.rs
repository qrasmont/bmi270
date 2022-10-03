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

pub struct WristGestureOutVal;
impl WristGestureOutVal {
    pub const UNKNOWN: u8 = 0x00;
    pub const PUSH_ARM_DOWN: u8 = 0x01;
    pub const PIVOT_UP: u8 = 0x02;
    pub const SHAKE: u8 = 0x03;
    pub const FLICK_IN: u8 = 0x04;
    pub const FLICK_OUT: u8 = 0x05;
}

pub struct ActivityOutVal;
impl ActivityOutVal {
    pub const STILL: u8 = 0x00;
    pub const WALKING: u8 = 0x01;
    pub const RUNNING: u8 = 0x02;
    pub const UNKNOWN: u8 = 0x03;
}

pub struct WristGestureActivityBits;
impl WristGestureActivityBits {
    pub const WRIST_GESTURE: u8 = 0b0000_0111;
    pub const ACTIVITY: u8 = 0b0001_1000;
}

pub struct MessageVal;
impl MessageVal {
    pub const NOT_INIT: u8 = 0x00;
    pub const INIT_OK: u8 = 0x01;
    pub const INIT_ERR: u8 = 0x02;
    pub const DRV_ERR: u8 = 0x03;
    pub const SNS_ERR: u8 = 0x04;
    pub const NVM_ERR: u8 = 0x05;
    pub const STARTUP_ERR: u8 = 0x06;
    pub const COMPAT_ERR: u8 = 0x07;
}

pub struct InternalStatusBits;
impl InternalStatusBits {
    pub const MESSAGE: u8 = 0b0000_0111;
    pub const AXES_REMAP_ERROR: u8 = 1 << 5;
    pub const ODR_50HZ_ERROR: u8 = 1 << 6;
}

pub const FIFO_LENGTH_1_BITS: u8 = 0b0011_1111;

pub struct AccOrdVal;
impl AccOrdVal {
    pub const ODR_0P78: u8 = 0x01;
    pub const ODR_1P5: u8 = 0x02;
    pub const ODR_3P1: u8 = 0x03;
    pub const ODR_6P25: u8 = 0x04;
    pub const ODR_12P5: u8 = 0x05;
    pub const ODR_25: u8 = 0x06;
    pub const ODR_50: u8 = 0x07;
    pub const ODR_100: u8 = 0x08;
    pub const ODR_200: u8 = 0x09;
    pub const ODR_400: u8 = 0x0A;
    pub const ODR_800: u8 = 0x0B;
    pub const ODR_1K6: u8 = 0x0C;
    pub const ODR_3K2: u8 = 0x0D;
    pub const ODR_6K4: u8 = 0x0E;
    pub const ODR_12K8: u8 = 0x0F;
}

pub struct AccBwpVal;
impl AccBwpVal {
    pub const OSR4_AVG1: u8 = 0x00;
    pub const OSR2_AVG2: u8 = 0x01;
    pub const NORM_AVG4: u8 = 0x02;
    pub const CIC_AVG8: u8 = 0x03;
    pub const RES_AVG16: u8 = 0x04;
    pub const RES_AVG32: u8 = 0x05;
    pub const RES_AVG64: u8 = 0x06;
    pub const RES_AVG128: u8 = 0x07;
}

pub struct AccFilterPerfVal;
impl AccFilterPerfVal {
    pub const POWER: u8 = 0x00;
    pub const PERF: u8 = 0x01;
}

pub struct AccConfBits;
impl AccConfBits {
    pub const ACC_ODR: u8 = 0b0000_1111;
    pub const ACC_BWP: u8 = 0b0111_0000;
    pub const ACC_FILTER_PERF: u8 = 1 << 7;
}
