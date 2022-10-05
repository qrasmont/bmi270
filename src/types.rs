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
pub enum Odr {
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
pub enum PerfMode {
    /// Power optimized.
    Power = 0x00,
    /// Performance optimized.
    Perf = 0x01,
}

/// Accelerometer configuration.
pub struct AccConf {
    /// Accelerometer Output Data Rate in Hz.
    pub odr: Odr,
    /// Accelerometer filter config & averaging.
    pub bwp: AccBwp,
    /// Accelerometer filter performance mode.
    pub filter_perf: PerfMode,
}

impl AccConf {
    pub fn from_reg(reg: u8) -> AccConf {
        AccConf {
            odr: match reg & AccConfMask::ACC_ODR {
                0x01 => Odr::Odr0p78,
                0x02 => Odr::Odr1p5,
                0x03 => Odr::Odr3p1,
                0x04 => Odr::Odr6p25,
                0x05 => Odr::Odr12p5,
                0x06 => Odr::Odr25,
                0x07 => Odr::Odr50,
                0x08 => Odr::Odr100,
                0x09 => Odr::Odr200,
                0x0A => Odr::Odr400,
                0x0B => Odr::Odr800,
                0x0C => Odr::Odr1k6,
                0x0D => Odr::Odr3k2,
                0x0E => Odr::Odr6k4,
                0x0F => Odr::Odr12k8,
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
                0x00 => PerfMode::Power,
                0x01 => PerfMode::Perf,
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

pub struct GyrConfMask;
impl GyrConfMask {
    pub const GYR_ODR: u8 = 0b0000_1111;
    pub const GYR_BWP: u8 = 0b0011_0000;
    pub const GYR_NOISE_PERF: u8 = 1 << 6;
    pub const GYR_FILTER_PERF: u8 = 1 << 7;
}

/// Gyroscope filter config & averaging.
#[repr(u8)]
pub enum GyrBwp {
    /// OSR4 filter, no averaging.
    Osr4 = 0x00,
    /// OSR2 filter, average 2 samples.
    Osr2 = 0x01,
    /// Normal filter, average 4 samples.
    Norm = 0x02,
    /// CIC filter, average 8 samples.
    Res = 0x03,
}

/// Gyroscope configuration.
pub struct GyrConf {
    /// Gyroscope Output Data Rate in Hz.
    pub odr: Odr,
    /// Gyroscope 3dB cutoff frequency for the low pass filter.
    pub bwp: GyrBwp,
    /// Gyroscope noise performance mode.
    pub noise_perf: PerfMode,
    /// Gyroscope filter performance mode.
    pub filter_perf: PerfMode,
}

impl GyrConf {
    pub fn from_reg(reg: u8) -> GyrConf {
        GyrConf {
            odr: match reg & GyrConfMask::GYR_ODR {
                0x01 => Odr::Odr0p78,
                0x02 => Odr::Odr1p5,
                0x03 => Odr::Odr3p1,
                0x04 => Odr::Odr6p25,
                0x05 => Odr::Odr12p5,
                0x06 => Odr::Odr25,
                0x07 => Odr::Odr50,
                0x08 => Odr::Odr100,
                0x09 => Odr::Odr200,
                0x0A => Odr::Odr400,
                0x0B => Odr::Odr800,
                0x0C => Odr::Odr1k6,
                0x0D => Odr::Odr3k2,
                0x0E => Odr::Odr6k4,
                0x0F => Odr::Odr12k8,
                _ => panic!(), // TODO
            },
            bwp: match (reg & GyrConfMask::GYR_BWP) >> 4 {
                0x00 => GyrBwp::Osr4,
                0x01 => GyrBwp::Osr2,
                0x02 => GyrBwp::Norm,
                0x03 => GyrBwp::Res,
                _ => panic!(), // TODO
            },
            noise_perf: match (reg & GyrConfMask::GYR_NOISE_PERF) >> 6 {
                0x00 => PerfMode::Power,
                0x01 => PerfMode::Perf,
                _ => panic!(), // TODO
            },
            filter_perf: match (reg & GyrConfMask::GYR_FILTER_PERF) >> 7 {
                0x00 => PerfMode::Power,
                0x01 => PerfMode::Perf,
                _ => panic!(), // TODO
            },
        }
    }

    pub fn to_reg(self) -> u8 {
        let odr = self.odr as u8;
        let bwp = self.bwp as u8;
        let noise_perf = self.noise_perf as u8;
        let filter_perf = self.filter_perf as u8;

        odr | bwp << 4 | noise_perf << 6 | filter_perf << 7
    }
}

pub struct GyrRangeMask;
impl GyrRangeMask {
    pub const GYR_RANGE: u8 = 0b0000_0011;
    pub const OIS_RANGE: u8 = 1 << 3;
}

#[repr(u8)]
pub enum GyrRangeVal {
    Range2000 = 0x00,
    Range1000 = 0x01,
    Range500 = 0x02,
    Range250 = 0x03,
    Range125 = 0x04,
}

#[repr(u8)]
pub enum OisRange {
    Range250 = 0x00,
    Range2000 = 0x01,
}

pub struct GyrRange {
    pub range: GyrRangeVal,
    pub ois_range: OisRange,
}

impl GyrRange {
    pub fn from_reg(reg: u8) -> GyrRange {
        GyrRange {
            range: match reg & GyrRangeMask::GYR_RANGE {
                0x00 => GyrRangeVal::Range2000,
                0x01 => GyrRangeVal::Range1000,
                0x02 => GyrRangeVal::Range500,
                0x03 => GyrRangeVal::Range250,
                0x04 => GyrRangeVal::Range125,
                _ => panic!(), // TODO
            },
            ois_range: match (reg & GyrRangeMask::OIS_RANGE) >> 3 {
                0x00 => OisRange::Range250,
                0x01 => OisRange::Range2000,
                _ => panic!(), // TODO
            },
        }
    }

    pub fn to_reg(self) -> u8 {
        let range = self.range as u8;
        let ois = self.ois_range as u8;

        range | ois << 3
    }
}

pub struct AuxConfMask;
impl AuxConfMask {
    pub const AUX_ODR: u8 = 0b0000_1111;
    pub const AUX_OFFSET: u8 = 0b1111_0000;
}

/// Auxiliary device configuration.
pub struct AuxConf {
    /// Auxiliary device Output Data Rate in Hz.
    pub odr: Odr,
    /// Trigger-readout offset in units of 2.5ms.
    pub offset: u8,
}

impl AuxConf {
    pub fn from_reg(reg: u8) -> AuxConf {
        AuxConf {
            odr: match reg & AuxConfMask::AUX_ODR {
                0x01 => Odr::Odr0p78,
                0x02 => Odr::Odr1p5,
                0x03 => Odr::Odr3p1,
                0x04 => Odr::Odr6p25,
                0x05 => Odr::Odr12p5,
                0x06 => Odr::Odr25,
                0x07 => Odr::Odr50,
                0x08 => Odr::Odr100,
                0x09 => Odr::Odr200,
                0x0A => Odr::Odr400,
                0x0B => Odr::Odr800,
                0x0C => Odr::Odr1k6,
                0x0D => Odr::Odr3k2,
                0x0E => Odr::Odr6k4,
                0x0F => Odr::Odr12k8,
                _ => panic!(), // TODO
            },
            offset: (reg & AuxConfMask::AUX_OFFSET) >> 4,
        }
    }

    pub fn to_reg(self) -> u8 {
        let odr = self.odr as u8;
        let offset = self.offset;

        odr | offset << 4
    }
}

pub struct FifoDownsMask;
impl FifoDownsMask {
    pub const GYR_FIFO_DOWNS: u8 = 0b0000_0111;
    pub const GYR_FIFO_FILT_DATA: u8 = 1 << 3;
    pub const ACC_FIFO_DOWNS: u8 = 0b0111_0000;
    pub const ACC_FIFO_FILT_DATA: u8 = 1 << 7;
}

/// Select filtered or unfiltered fifo data.
#[repr(u8)]
pub enum FilterData {
    Unfiltered = 0x00,
    Filtered = 0x01,
}

/// Fifo downsampling configuration.
pub struct FifoDowns {
    /// Downsampling for the gyroscope.
    pub gyr_downs: u8,
    /// Select filtered or unfiltered gyroscope fifo data.
    pub gyr_filt_data: FilterData,
    /// Downsampling for the accelerometer.
    pub acc_downs: u8,
    /// Select filtered or unfiltered accelerometer fifo data.
    pub acc_filt_data: FilterData,
}

impl FifoDowns {
    pub fn from_reg(reg: u8) -> FifoDowns {
        FifoDowns {
            gyr_downs: reg & FifoDownsMask::GYR_FIFO_DOWNS,
            gyr_filt_data: match (reg & FifoDownsMask::GYR_FIFO_FILT_DATA) >> 3 {
                0x00 => FilterData::Unfiltered,
                0x01 => FilterData::Filtered,
                _ => panic!(), // TODO
            },
            acc_downs: (reg & FifoDownsMask::ACC_FIFO_DOWNS) >> 4,
            acc_filt_data: match (reg & FifoDownsMask::ACC_FIFO_FILT_DATA) >> 7 {
                0x00 => FilterData::Unfiltered,
                0x01 => FilterData::Filtered,
                _ => panic!(), // TODO
            },
        }
    }

    pub fn to_reg(self) -> u8 {
        let gyr_downs = self.gyr_downs;
        let gyr_filt_data = self.gyr_filt_data as u8;
        let acc_downs = self.acc_downs;
        let acc_filt_data = self.acc_filt_data as u8;

        gyr_downs | gyr_filt_data << 3 | acc_downs << 4 | acc_filt_data << 7
    }
}

pub struct FifoConfig0Mask;
impl FifoConfig0Mask {
    pub const STOP_ON_FULL: u8 = 1;
    pub const TIME_EN: u8 = 1 << 1;
}

pub struct FifoConfig1Mask;
impl FifoConfig1Mask {
    pub const TAG_INT1_EN: u8 = 0b000_0011;
    pub const TAG_INT2_EN: u8 = 0b000_1100;
    pub const HEADER_EN: u8 = 1 << 4;
    pub const AUX_EN: u8 = 1 << 5;
    pub const ACC_EN: u8 = 1 << 6;
    pub const GYR_EN: u8 = 1 << 7;
}

/// Fifo interrupt tag enable.
#[repr(u8)]
pub enum FifoTagIntEnable {
    IntEdge = 0x00,
    IntLevel = 0x01,
    AccSat = 0x02,
    GyrSat = 0x03,
}

/// Fifo configuration.
pub struct FifoConf {
    /// Stop wriing samples into the fifo when it is full.
    pub stop_on_full: bool,
    /// Return the sensor time frame after the last valid data frame.
    pub time_enable: bool,
    /// Interrupt 1 tag enable.
    pub tag_int1_en: FifoTagIntEnable,
    /// Interrupt 2 tag enable.
    pub tag_int2_en: FifoTagIntEnable,
    /// Fifo frame header enable.
    pub header_enable: bool,
    /// Store auxiliary sensor data in the fifo
    pub aux_enable: bool,
    /// Store accelerometer data in the fifo
    pub acc_enable: bool,
    /// Store gyroscope data in the fifo
    pub gyr_enable: bool,
}

impl FifoConf {
    pub fn from_regs(reg_0: u8, reg_1: u8) -> FifoConf {
        FifoConf {
            stop_on_full: (reg_0 & FifoConfig0Mask::STOP_ON_FULL) != 0,
            time_enable: (reg_0 & FifoConfig0Mask::TIME_EN) != 0,
            tag_int1_en: match reg_1 & FifoConfig1Mask::TAG_INT1_EN {
                0x00 => FifoTagIntEnable::IntEdge,
                0x01 => FifoTagIntEnable::IntLevel,
                0x02 => FifoTagIntEnable::AccSat,
                0x03 => FifoTagIntEnable::GyrSat,
                _ => panic!(), // TODO
            },
            tag_int2_en: match (reg_1 & FifoConfig1Mask::TAG_INT2_EN) >> 2 {
                0x00 => FifoTagIntEnable::IntEdge,
                0x01 => FifoTagIntEnable::IntLevel,
                0x02 => FifoTagIntEnable::AccSat,
                0x03 => FifoTagIntEnable::GyrSat,
                _ => panic!(), // TODO
            },
            header_enable: (reg_1 & FifoConfig1Mask::HEADER_EN) >> 4 != 0,
            aux_enable: (reg_1 & FifoConfig1Mask::AUX_EN) >> 5 != 0,
            acc_enable: (reg_1 & FifoConfig1Mask::ACC_EN) >> 6 != 0,
            gyr_enable: (reg_1 & FifoConfig1Mask::GYR_EN) >> 7 != 0,
        }
    }

    pub fn to_regs(self) -> (u8, u8) {
        let stop_on_full: u8 = if self.stop_on_full { 0x01 } else { 0x00 };
        let time_enable: u8 = if self.time_enable { 0x01 } else { 0x00 };
        let tag_int1_en = self.tag_int1_en as u8;
        let tag_int2_en = self.tag_int2_en as u8;
        let header_enable = if self.header_enable { 0x01 } else { 0x00 };
        let aux_enable: u8 = if self.aux_enable { 0x01 } else { 0x00 };
        let acc_enable: u8 = if self.acc_enable { 0x01 } else { 0x00 };
        let gyr_enable: u8 = if self.gyr_enable { 0x01 } else { 0x00 };

        (
            stop_on_full | time_enable << 1,
            tag_int1_en
                | tag_int2_en << 2
                | header_enable << 4
                | aux_enable << 5
                | acc_enable << 6
                | gyr_enable << 7,
        )
    }
}

pub struct SaturationMask;
impl SaturationMask {
    pub const ACC_X: u8 = 1;
    pub const ACC_Y: u8 = 1 << 1;
    pub const ACC_Z: u8 = 1 << 2;
    pub const GYR_X: u8 = 1 << 3;
    pub const GYR_Y: u8 = 1 << 4;
    pub const GYR_Z: u8 = 1 << 5;
}

/// Notifies if the current values have been saturated. Synchronously update with the data
/// registers.
pub struct Saturation {
    /// Accelerometer x axis is saturated.
    pub acc_x: bool,
    /// Accelerometer y axis is saturated.
    pub acc_y: bool,
    /// Accelerometer z axis is saturated.
    pub acc_z: bool,
    /// Gyroscope x axis is saturated.
    pub gyr_x: bool,
    /// Gyroscope y axis is saturated.
    pub gyr_y: bool,
    /// Gyroscope z axis is saturated.
    pub gyr_z: bool,
}

impl Saturation {
    pub fn from_reg(reg: u8) -> Saturation {
        Saturation {
            acc_x: reg & SaturationMask::ACC_X != 0,
            acc_y: (reg & SaturationMask::ACC_Y) >> 1 != 0,
            acc_z: (reg & SaturationMask::ACC_Z) >> 2 != 0,
            gyr_x: (reg & SaturationMask::GYR_X) >> 3 != 0,
            gyr_y: (reg & SaturationMask::GYR_Y) >> 4 != 0,
            gyr_z: (reg & SaturationMask::GYR_Z) >> 5 != 0,
        }
    }
}
