/// The possible errors that could be encountered.
pub enum Error<CommE, CsE> {
    /// Communication error over I2C or SPI.
    Comm(CommE),
    /// Pin error on the SPI chip select.
    Cs(CsE),
}

pub struct ErrRegMask;
impl ErrRegMask {
    pub const AUX_ERR: u8 = 1 << 7;
    pub const FIFO_ERR: u8 = 1 << 6;
    pub const INTERNAL_ERR: u8 = 0b0001_1110;
    pub const FATAL_ERR: u8 = 1;
}

/// Reports sensor error conditions.
pub struct ErrorReg {
    /// The chip is not in an operational state.
    pub fatal_err: bool,
    /// Interal error (Bosch advises to contact the Sensortec support).
    pub internal_err: u8,
    /// Error when a frame is reaad in streaming mode and the fifo is overfilled.
    pub fifo_err: bool,
    /// Error in I2C-Master detected.
    pub aux_err: bool,
}

impl ErrorReg {
    pub fn from_reg(reg: u8) -> ErrorReg {
        ErrorReg {
            fatal_err: (reg & ErrRegMask::FATAL_ERR) != 0,
            internal_err: reg & ErrRegMask::INTERNAL_ERR,
            fifo_err: (reg & ErrRegMask::FIFO_ERR) != 0,
            aux_err: (reg & ErrRegMask::AUX_ERR) != 0,
        }
    }
}

pub struct StatusBits;
impl StatusBits {
    pub const DRDY_ACC: u8 = 1 << 7;
    pub const DRDY_GYR: u8 = 1 << 6;
    pub const DRDY_AUX: u8 = 1 << 5;
    pub const CMD_RDY: u8 = 1 << 4;
    pub const AUX_BUSY: u8 = 1 << 2;
}

/// Sensor status flags.
pub struct Status {
    /// Data ready for Accelerometer.
    pub acc_data_ready: bool,
    /// Data ready for Gyroscope.
    pub gyr_data_ready: bool,
    /// Data ready for Auxiliary sensor.
    pub aux_data_ready: bool,
    /// Command decoder ready for a new command.
    pub cmd_ready: bool,
    /// Auxiliary sensor operation ongoing.
    pub aux_dev_busy: bool,
}

impl Status {
    pub fn from_reg(reg: u8) -> Status {
        Status {
            acc_data_ready: (reg & StatusBits::DRDY_ACC) != 0,
            gyr_data_ready: (reg & StatusBits::DRDY_GYR) != 0,
            aux_data_ready: (reg & StatusBits::DRDY_AUX) != 0,
            cmd_ready: (reg & StatusBits::CMD_RDY) != 0,
            aux_dev_busy: (reg & StatusBits::AUX_BUSY) != 0,
        }
    }
}

/// Axis data.
pub struct AxisData {
    /// X axis data.
    pub x: i16,
    /// Y axis data.
    pub y: i16,
    /// Z axis data.
    pub z: i16,
}

/// Auxiliary sensor data.
pub struct AuxData {
    /// Axis data.
    pub axis: AxisData,
    // TODO
    /// Last aux registers data.
    pub r: i16,
}

/// Sensor data.
pub struct Data {
    /// Accelerometer data.
    pub acc: AxisData,
    /// Gyroscope data.
    pub gyr: AxisData,
    /// Sensor time.
    pub time: u32,
}

pub struct EventMask;
impl EventMask {
    pub const ERR_CODE: u8 = 0b0001_1100;
    pub const POR_DETECTED: u8 = 1;
}

/// Possible persistent errors.
#[repr(u8)]
pub enum PersistentErrors {
    /// No errors reported.
    NoErr = 0x00,
    /// Error in the accelerometer config register.
    AccErr = 0x01,
    /// Error in the gyroscope config register.
    GyrErr = 0x02,
    /// Error in both the accelerometer and gyroscope config registers.
    AccGyrErr = 0x03,
}

/// Sensor event flags. Will be cleared on read when bit 0 is sent out over the bus.
pub struct Event {
    /// True after device power up or softreset. False after status read.
    pub por_detected: bool,
    /// Persistent errors.
    pub persistent_err: PersistentErrors,
}

impl Event {
    pub fn from_reg(reg: u8) -> Event {
        Event {
            por_detected: (reg & EventMask::POR_DETECTED) != 0,
            persistent_err: match (reg & EventMask::ERR_CODE) >> 2 {
                0x00 => PersistentErrors::NoErr,
                0x01 => PersistentErrors::AccErr,
                0x02 => PersistentErrors::GyrErr,
                0x03 => PersistentErrors::AccGyrErr,
                _ => panic!(), // TODO
            },
        }
    }
}

pub struct InterruptStatus0Mask;
impl InterruptStatus0Mask {
    pub const SIG_MOTION_OUT: u8 = 1;
    pub const STEP_COUNTER_OUT: u8 = 1 << 1;
    pub const ACTIVITY_OUT: u8 = 1 << 2;
    pub const WRIST_WEAR_WAKEUP_OUT: u8 = 1 << 3;
    pub const WRIST_GESTURE_OUT: u8 = 1 << 4;
    pub const NO_MOTION_OUT: u8 = 1 << 5;
    pub const ANY_MOTION_OUT: u8 = 1 << 6;
}

pub struct InterruptStatus1Mask;
impl InterruptStatus1Mask {
    pub const FFULL_INT: u8 = 1;
    pub const FWM_INT: u8 = 1 << 1;
    pub const ERR_INT: u8 = 1 << 2;
    pub const AUX_DRDY_INT: u8 = 1 << 5;
    pub const GYR_DRDY_INT: u8 = 1 << 6;
    pub const ACC_DRDY_INT: u8 = 1 << 7;
}

/// Interrut/Feature Status. Will be cleared on read.
pub struct InterruptStatus {
    /// Sigmotion output.
    pub sig_motion_out: bool,
    /// Step-counter watermark or Step-detector output.
    pub step_counter_out: bool,
    /// Step activity output.
    pub activity_out: bool,
    /// Wrist wear wakeup ouput
    pub wrist_wear_wakeup_out: bool,
    /// Wrist gesture output.
    pub wrist_gesture_out: bool,
    /// No motion detection output.
    pub no_motion_out: bool,
    /// Any motion detecion ouput.
    pub any_motion_out: bool,
    /// FIFO full interrupt.
    pub ffull_int: bool,
    /// FIFO watermark interrupt.
    pub fwm_int: bool,
    /// Error interrupt.
    pub err_int: bool,
    /// Auxiliary data ready interrupt.
    pub aux_drdy_int: bool,
    /// Gyroscope data ready interrupt.
    pub gyr_drdy_int: bool,
    /// Accelerometer data ready interrupt.
    pub acc_drdy_int: bool,
}

impl InterruptStatus {
    pub fn from_regs(int_stat_0: u8, int_stat_1: u8) -> InterruptStatus {
        InterruptStatus {
            sig_motion_out: (int_stat_0 & InterruptStatus0Mask::SIG_MOTION_OUT) != 0,
            step_counter_out: (int_stat_0 & InterruptStatus0Mask::STEP_COUNTER_OUT) != 0,
            activity_out: (int_stat_0 & InterruptStatus0Mask::ACTIVITY_OUT) != 0,
            wrist_wear_wakeup_out: (int_stat_0 & InterruptStatus0Mask::WRIST_WEAR_WAKEUP_OUT) != 0,
            wrist_gesture_out: (int_stat_0 & InterruptStatus0Mask::WRIST_GESTURE_OUT) != 0,
            no_motion_out: (int_stat_0 & InterruptStatus0Mask::NO_MOTION_OUT) != 0,
            any_motion_out: (int_stat_0 & InterruptStatus0Mask::ANY_MOTION_OUT) != 0,
            ffull_int: (int_stat_1 & InterruptStatus1Mask::FFULL_INT) != 0,
            fwm_int: (int_stat_1 & InterruptStatus1Mask::FWM_INT) != 0,
            err_int: (int_stat_1 & InterruptStatus1Mask::ERR_INT) != 0,
            aux_drdy_int: (int_stat_1 & InterruptStatus1Mask::AUX_DRDY_INT) != 0,
            gyr_drdy_int: (int_stat_1 & InterruptStatus1Mask::GYR_DRDY_INT) != 0,
            acc_drdy_int: (int_stat_1 & InterruptStatus1Mask::ACC_DRDY_INT) != 0,
        }
    }
}

pub struct WristGestureActivityMask;
impl WristGestureActivityMask {
    pub const WRIST_GESTURE: u8 = 0b0000_0111;
    pub const ACTIVITY: u8 = 0b0001_1000;
}

/// Wrist gestures.
#[repr(u8)]
pub enum WristGesture {
    /// Unknown.
    Unknown = 0x00,
    /// Push the arm down.
    PushArmDown = 0x01,
    /// Pivot up.
    PivotUp = 0x02,
    /// Wrist shake/jiggle.
    Shake = 0x03,
    /// Arm flick in.
    FlickIn = 0x04,
    /// Arm flick out.
    FlickOut = 0x05,
}

/// Activity detection.
#[repr(u8)]
pub enum Activity {
    /// User stationnary.
    Still = 0x00,
    /// User walking.
    Walking = 0x01,
    /// User running.
    Running = 0x02,
    /// Unknown state.
    Unknown = 0x03,
}

/// Wrist gesture and activity.
pub struct WristGestureActivity {
    /// Wrist gesture.
    pub wrist_gesture: WristGesture,
    /// Activity.
    pub activity: Activity,
}

impl WristGestureActivity {
    pub fn from_reg(reg: u8) -> WristGestureActivity {
        WristGestureActivity {
            wrist_gesture: match reg & WristGestureActivityMask::WRIST_GESTURE {
                0x00 => WristGesture::Unknown,
                0x01 => WristGesture::PushArmDown,
                0x02 => WristGesture::PivotUp,
                0x03 => WristGesture::Shake,
                0x04 => WristGesture::FlickIn,
                0x05 => WristGesture::FlickOut,
                _ => panic!(), // TODO
            },
            activity: match (reg & WristGestureActivityMask::ACTIVITY) >> 3 {
                0x00 => Activity::Still,
                0x01 => Activity::Walking,
                0x02 => Activity::Running,
                0x03 => Activity::Unknown,
                _ => panic!(), // TODO
            },
        }
    }
}

pub struct InternalStatusMask;
impl InternalStatusMask {
    pub const MESSAGE: u8 = 0b0000_0111;
    pub const AXES_REMAP_ERROR: u8 = 1 << 5;
    pub const ODR_50HZ_ERROR: u8 = 1 << 6;
}

/// Internal status message.
#[repr(u8)]
pub enum Message {
    /// ASIC is not initialized.
    NotInit = 0x00,
    /// ASIC initialized.
    InitOk = 0x01,
    /// initialization error.
    InitErr = 0x02,
    /// Ivalid driver.
    DrvErr = 0x03,
    /// Sensor stopped.
    SnsErr = 0x04,
    /// Internal error while accessing NVM.
    NvmErr = 0x05,
    /// Internal error while accessing NVM and initialization error.
    StartUpErr = 0x06,
    /// Compatibility error.
    CompatErr = 0x07,
}

/// Internal status.
pub struct InternalStatus {
    /// Internal status message.
    pub message: Message,
    /// Incorrect axies remapping.
    pub axes_remap_error: bool,
    /// The min bandwidth contions are not respected.
    pub odr_50hz_error: bool,
}

impl InternalStatus {
    pub fn from_reg(reg: u8) -> InternalStatus {
        InternalStatus {
            message: match reg & InternalStatusMask::MESSAGE {
                0x00 => Message::NotInit,
                0x01 => Message::InitOk,
                0x02 => Message::InitErr,
                0x03 => Message::DrvErr,
                0x04 => Message::SnsErr,
                0x05 => Message::NvmErr,
                0x06 => Message::StartUpErr,
                0x07 => Message::CompatErr,
                _ => panic!(), // TODO
            },
            axes_remap_error: (reg & InternalStatusMask::AXES_REMAP_ERROR) != 0,
            odr_50hz_error: (reg & InternalStatusMask::ODR_50HZ_ERROR) != 0,
        }
    }
}

pub const FIFO_LENGTH_1_MASK: u8 = 0b0011_1111;

pub struct AccConfMask;
impl AccConfMask {
    pub const ACC_ODR: u8 = 0b0000_1111;
    pub const ACC_BWP: u8 = 0b0111_0000;
    pub const ACC_FILTER_PERF: u8 = 1 << 7;
}

/// Accelerometer Output Data Rate in Hz.
#[repr(u8)]
pub enum AccOdr {
    /// 25/32 Hz.
    Odr0p78 = 0x01,
    /// 25/16 Hz.
    Odr1p5 = 0x02,
    /// 25/8 Hz.
    Odr3p1 = 0x03,
    /// 25/4 Hz.
    Odr6p25 = 0x04,
    /// 25/2 Hz.
    Odr12p5 = 0x05,
    /// 25 Hz.
    Odr25 = 0x06,
    /// 50 Hz.
    Odr50 = 0x07,
    /// 100 Hz.
    Odr100 = 0x08,
    /// 200 Hz.
    Odr200 = 0x09,
    /// 400 Hz.
    Odr400 = 0x0A,
    /// 800 Hz.
    Odr800 = 0x0B,
    /// 1600 Hz.
    Odr1k6 = 0x0C,
    /// 3200 Hz.
    Odr3k2 = 0x0D,
    /// 6400 Hz.
    Odr6k4 = 0x0E,
    /// 12800 Hz.
    Odr12k8 = 0x0F,
}

/// Accelerometer filter config & averaging.
#[repr(u8)]
pub enum AccBwp {
    /// OSR4 filter, no averaging.
    Osr4Avg1 = 0x00,
    /// OSR2 filter, average 2 samples.
    Osr2Avg2 = 0x01,
    /// Normal filter, average 4 samples.
    NormAvg4 = 0x02,
    /// CIC filter, average 8 samples.
    CicAvg8 = 0x03,
    /// Reserved filter, average 16 samples.
    ResAvg16 = 0x04,
    /// Reserved filter, average 32 samples.
    ResAvg32 = 0x05,
    /// Reserved filter, average 64 samples.
    ResAvg64 = 0x06,
    /// Reserved filter, average 128 samples.
    ResAvg128 = 0x07,
}

/// Accelerometer filter performance mode.
#[repr(u8)]
pub enum AccFilterPerf {
    /// Power optimized.
    Power = 0x00,
    /// Performance optimized.
    Perf = 0x01,
}

/// Accelerometer configuration.
pub struct AccConf {
    /// Accelerometer Output Data Rate in Hz.
    pub odr: AccOdr,
    /// Accelerometer filter config & averaging.
    pub bwp: AccBwp,
    /// Accelerometer filter performance mode.
    pub filter_perf: AccFilterPerf,
}

impl AccConf {
    pub fn from_reg(reg: u8) -> AccConf {
        AccConf {
            odr: match reg & AccConfMask::ACC_ODR {
                0x01 => AccOdr::Odr0p78,
                0x02 => AccOdr::Odr1p5,
                0x03 => AccOdr::Odr3p1,
                0x04 => AccOdr::Odr6p25,
                0x05 => AccOdr::Odr12p5,
                0x06 => AccOdr::Odr25,
                0x07 => AccOdr::Odr50,
                0x08 => AccOdr::Odr100,
                0x09 => AccOdr::Odr200,
                0x0A => AccOdr::Odr400,
                0x0B => AccOdr::Odr800,
                0x0C => AccOdr::Odr1k6,
                0x0D => AccOdr::Odr3k2,
                0x0E => AccOdr::Odr6k4,
                0x0F => AccOdr::Odr12k8,
                _ => panic!(), // TODO
            },
            bwp: match (reg & AccConfMask::ACC_BWP) >> 4 {
                0x00 => AccBwp::Osr4Avg1,
                0x01 => AccBwp::Osr2Avg2,
                0x02 => AccBwp::NormAvg4,
                0x03 => AccBwp::CicAvg8,
                0x04 => AccBwp::ResAvg16,
                0x05 => AccBwp::ResAvg32,
                0x06 => AccBwp::ResAvg64,
                0x07 => AccBwp::ResAvg128,
                _ => panic!(), // TODO
            },
            filter_perf: match (reg & AccConfMask::ACC_FILTER_PERF) >> 7 {
                0x00 => AccFilterPerf::Power,
                0x01 => AccFilterPerf::Perf,
                _ => panic!(), // TODO
            },
        }
    }

    pub fn to_reg(self) -> u8 {
        let odr = self.odr as u8;
        let bwp = self.bwp as u8;
        let filter_perf = self.filter_perf as u8;

        odr | bwp << 4 | filter_perf << 7
    }
}

/// Accelerometer g range.
#[repr(u8)]
pub enum AccRange {
    /// +/- 2g.
    Range2g = 0x00,
    /// +/- 4g.
    Range4g = 0x01,
    /// +/- 8g.
    Range8g = 0x02,
    /// +/- 16g.
    Range16g = 0x03,
}

impl AccRange {
    pub fn from_reg(reg: u8) -> AccRange {
        match reg {
            0x00 => AccRange::Range2g,
            0x01 => AccRange::Range4g,
            0x02 => AccRange::Range8g,
            0x03 => AccRange::Range16g,
            _ => panic!(), // TODO
        }
    }
}
