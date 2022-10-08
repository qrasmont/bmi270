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

pub struct AuxIfConfMask;
impl AuxIfConfMask {
    pub const AUX_RD_BURST: u8 = 0b000_0011;
    pub const MAN_RD_BURST: u8 = 0b000_1100;
    pub const AUX_FCU_WRITE_EN: u8 = 1 << 6;
    pub const AUX_MANUAL_EN: u8 = 1 << 7;
}

/// Read burst.
#[repr(u8)]
pub enum ReadBurst {
    /// 1 byte.
    Burst1Byte = 0x00,
    /// 2 bytes.
    Burst2Byte = 0x01,
    /// 4 bytes.
    Burst4Byte = 0x02,
    /// 8 bytes.
    Burst8Byte = 0x03,
}

/// Auxiliary sensor configuration..
pub struct AuxIfConf {
    /// Burst data lenngth.
    pub aux_read_burst: ReadBurst,
    /// Manual burst data lenngth.
    pub man_read_burst: ReadBurst,
    /// Enables FCU write command.
    pub aux_fcu_write_en: bool,
    /// Switches between automatic and manual mode.
    pub aux_manual_en: bool,
}

impl AuxIfConf {
    pub fn from_reg(reg: u8) -> AuxIfConf {
        AuxIfConf {
            aux_read_burst: match reg & AuxIfConfMask::AUX_RD_BURST {
                0x00 => ReadBurst::Burst1Byte,
                0x01 => ReadBurst::Burst2Byte,
                0x02 => ReadBurst::Burst4Byte,
                0x03 => ReadBurst::Burst8Byte,
                _ => panic!(), // TODO
            },
            man_read_burst: match (reg & AuxIfConfMask::MAN_RD_BURST) >> 2 {
                0x00 => ReadBurst::Burst1Byte,
                0x01 => ReadBurst::Burst2Byte,
                0x02 => ReadBurst::Burst4Byte,
                0x03 => ReadBurst::Burst8Byte,
                _ => panic!(), // TODO
            },
            aux_fcu_write_en: (reg & AuxIfConfMask::AUX_FCU_WRITE_EN) >> 6 != 0,
            aux_manual_en: (reg & AuxIfConfMask::AUX_MANUAL_EN) >> 7 != 0,
        }
    }

    pub fn to_reg(self) -> u8 {
        let aux_read_burst = self.aux_read_burst as u8;
        let man_read_burst = self.man_read_burst as u8;
        let aux_fcu_write_en = if self.aux_fcu_write_en { 0x01 } else { 0x00 };
        let aux_manual_en = if self.aux_manual_en { 0x01 } else { 0x00 };

        aux_read_burst | man_read_burst << 2 | aux_fcu_write_en << 6 | aux_manual_en << 7
    }
}

/// Define which error flag will trigger the error interrupt.
pub struct ErrorRegMsk {
    /// Use fatal error.
    pub fatal_err: bool,
    /// Use internal error.
    pub internal_err: bool,
    /// Use fifo error.
    pub fifo_err: bool,
    /// Use auxiliary interface error.
    pub aux_err: bool,
}

impl ErrorRegMsk {
    pub fn from_reg(reg: u8) -> ErrorRegMsk {
        ErrorRegMsk {
            fatal_err: (reg & ErrRegMask::FATAL_ERR) != 0,
            internal_err: (reg & ErrRegMask::INTERNAL_ERR) != 0,
            fifo_err: (reg & ErrRegMask::FIFO_ERR) != 0,
            aux_err: (reg & ErrRegMask::AUX_ERR) != 0,
        }
    }

    pub fn to_reg(self) -> u8 {
        let fatal_err = if self.fatal_err { 0x01 } else { 0x00 };
        let internal_err = if self.internal_err { 0b0001_1110 } else { 0x00 };
        let fifo_err = if self.fifo_err { 0x01 } else { 0x00 };
        let aux_err = if self.aux_err { 0x01 } else { 0x00 };

        fatal_err | internal_err << 1 | fifo_err << 6 | aux_err << 7
    }
}

pub struct IntIoCtrlMask;
impl IntIoCtrlMask {
    pub const LEVEL: u8 = 1 << 1;
    pub const OD: u8 = 1 << 2;
    pub const OUTPUT_EN: u8 = 1 << 3;
    pub const INPUT_EN: u8 = 1 << 4;
}

/// Output level.
#[repr(u8)]
pub enum OutputLevel {
    /// Active low.
    ActiveLow = 0x00,
    /// Active high.
    ActiveHigh = 0x01,
}

/// Output behavior.
#[repr(u8)]
pub enum OutputBehavior {
    /// Active low.
    PushPull = 0x00,
    /// Active high.
    OpenDrain = 0x01,
}

/// Configuration of the electrical behavior of an interrupt.
pub struct IntIoCtrl {
    pub level: OutputLevel,
    pub od: OutputBehavior,
    pub ouput_en: bool,
    pub input_en: bool,
}

impl IntIoCtrl {
    pub fn from_reg(reg: u8) -> IntIoCtrl {
        IntIoCtrl {
            level: match (reg & IntIoCtrlMask::LEVEL) >> 1 {
                0x00 => OutputLevel::ActiveLow,
                0x01 => OutputLevel::ActiveHigh,
                _ => panic!(), // TODO
            },
            od: match (reg & IntIoCtrlMask::OD) >> 2 {
                0x00 => OutputBehavior::PushPull,
                0x01 => OutputBehavior::OpenDrain,
                _ => panic!(), // TODO
            },
            ouput_en: (reg & IntIoCtrlMask::OUTPUT_EN) != 0,
            input_en: (reg & IntIoCtrlMask::INPUT_EN) != 0,
        }
    }

    pub fn to_reg(self) -> u8 {
        let level = self.level as u8;
        let od = self.od as u8;
        let output_en = if self.input_en { 0x01 } else { 0x00 };
        let input_en = if self.ouput_en { 0x01 } else { 0x00 };

        level << 1 | od << 2 | output_en << 3 | input_en << 4
    }
}

pub struct IntLatchMask;
impl IntLatchMask {
    pub const LATCH: u8 = 1;
}

/// Latched mode for the interrupt.
#[repr(u8)]
pub enum IntLatch {
    /// Non latched.
    None = 0x00,
    /// Permanenet latched.
    Permanent = 0x01,
}

impl IntLatch {
    pub fn from_reg(reg: u8) -> IntLatch {
        match reg & IntLatchMask::LATCH {
            0x00 => IntLatch::None,
            0x01 => IntLatch::Permanent,
            _ => panic!(), // TODO
        }
    }
}

pub struct IntMapFeatMask;
impl IntMapFeatMask {
    pub const SIG_MOTION_OUT: u8 = 1;
    pub const STEP_COUNTER_OUT: u8 = 1 << 1;
    pub const ACTIVITY_OUT: u8 = 1 << 2;
    pub const WRIST_WEAR_WAKEUP_OUT: u8 = 1 << 3;
    pub const WRIST_GESTURE_OUT: u8 = 1 << 4;
    pub const NO_MOTION_OUT: u8 = 1 << 5;
    pub const ANY_MOTION_OUT: u8 = 1 << 6;
}

/// Interrupt feature mapping.
pub struct IntMapFeat {
    /// Sigmotion output.
    pub sig_motion_out: bool,
    /// Step-counter watermark or Step-detector output.
    pub step_counter_out: bool,
    /// Step activity output.
    pub activity_out: bool,
    /// Wrist wear wakeup output.
    pub wrist_wear_wakeup_out: bool,
    /// Wrist gesture output.
    pub wrist_gesture_out: bool,
    /// No motion detection output.
    pub no_motion_out: bool,
    /// Any motion detection output.
    pub any_motion_out: bool,
}

impl IntMapFeat {
    pub fn from_reg(reg: u8) -> IntMapFeat {
        IntMapFeat {
            sig_motion_out: (reg & IntMapFeatMask::SIG_MOTION_OUT) != 0,
            step_counter_out: (reg & IntMapFeatMask::STEP_COUNTER_OUT) != 0,
            activity_out: (reg & IntMapFeatMask::ACTIVITY_OUT) != 0,
            wrist_wear_wakeup_out: (reg & IntMapFeatMask::WRIST_WEAR_WAKEUP_OUT) != 0,
            wrist_gesture_out: (reg & IntMapFeatMask::WRIST_GESTURE_OUT) != 0,
            no_motion_out: (reg & IntMapFeatMask::NO_MOTION_OUT) != 0,
            any_motion_out: (reg & IntMapFeatMask::ANY_MOTION_OUT) != 0,
        }
    }

    pub fn to_reg(self) -> u8 {
        let sig_motion_out = if self.sig_motion_out { 0x01 } else { 0x00 };
        let step_counter_out = if self.step_counter_out { 0x01 } else { 0x00 };
        let activity_out = if self.activity_out { 0x01 } else { 0x00 };
        let wrist_wear_wakeup_out = if self.wrist_wear_wakeup_out {
            0x01
        } else {
            0x00
        };
        let wrist_gesture_out = if self.wrist_gesture_out { 0x01 } else { 0x00 };
        let no_motion_out = if self.no_motion_out { 0x01 } else { 0x00 };
        let any_motion_out = if self.any_motion_out { 0x01 } else { 0x00 };

        sig_motion_out
            | step_counter_out << 1
            | activity_out << 2
            | wrist_wear_wakeup_out << 3
            | wrist_gesture_out << 4
            | no_motion_out << 5
            | any_motion_out << 6
    }
}

pub struct IntMapDataMask;
impl IntMapDataMask {
    pub const FFULL_INT1: u8 = 1;
    pub const FWM_INT1: u8 = 1 << 1;
    pub const DRDY_INT1: u8 = 1 << 2;
    pub const ERR_INT1: u8 = 1 << 3;
    pub const FFULL_INT2: u8 = 1 << 4;
    pub const FWM_INT2: u8 = 1 << 5;
    pub const DRDY_INT2: u8 = 1 << 6;
    pub const ERR_INT2: u8 = 1 << 7;
}

/// Data interrupt mapping.
pub struct MapData {
    /// Map fifo full interrupt.
    pub ffull: bool,
    /// Map fifo watermark interrupt.
    pub fwm: bool,
    /// Map data ready interrupt.
    pub drdy: bool,
    /// Map error interrupt.
    pub err: bool,
}

/// Data interrupt for int1 and int2.
pub struct IntMapData {
    /// Data interrupt mapping for int1.
    pub int1: MapData,
    /// Data interrupt mapping for int2.
    pub int2: MapData,
}

impl IntMapData {
    pub fn from_reg(reg: u8) -> IntMapData {
        IntMapData {
            int1: MapData {
                ffull: (reg & IntMapDataMask::FFULL_INT1) != 0,
                fwm: (reg & IntMapDataMask::FWM_INT1) != 0,
                drdy: (reg & IntMapDataMask::DRDY_INT1) != 0,
                err: (reg & IntMapDataMask::ERR_INT1) != 0,
            },
            int2: MapData {
                ffull: (reg & IntMapDataMask::FFULL_INT2) != 0,
                fwm: (reg & IntMapDataMask::FWM_INT2) != 0,
                drdy: (reg & IntMapDataMask::DRDY_INT2) != 0,
                err: (reg & IntMapDataMask::ERR_INT2) != 0,
            },
        }
    }

    pub fn to_reg(self) -> u8 {
        let ffull_int1 = if self.int1.ffull { 0x01 } else { 0x00 };
        let fwm_int1 = if self.int1.fwm { 0x01 } else { 0x00 };
        let drdy_int1 = if self.int1.drdy { 0x01 } else { 0x00 };
        let err_int1 = if self.int1.err { 0x01 } else { 0x00 };

        let ffull_int2 = if self.int2.ffull { 0x01 } else { 0x00 };
        let fwm_int2 = if self.int2.fwm { 0x01 } else { 0x00 };
        let drdy_int2 = if self.int2.drdy { 0x01 } else { 0x00 };
        let err_int2 = if self.int2.err { 0x01 } else { 0x00 };

        ffull_int1
            | fwm_int1 << 1
            | drdy_int1 << 2
            | err_int1 << 3
            | ffull_int2 << 4
            | fwm_int2 << 5
            | drdy_int2 << 6
            | err_int2 << 7
    }
}

pub struct InternalErrorMask;
impl InternalErrorMask {
    pub const INT_ERR_1: u8 = 1;
    pub const INT_ERR_2: u8 = 1 << 2;
    pub const FEAT_ENG_DIS: u8 = 1 << 4;
}

/// Internal error flags.
pub struct InternalError {
    /// Long processing time, processing halted.
    pub int_err_1: bool,
    /// Fatal error, procesing halted.
    pub int_err_2: bool,
    /// Feature engine has been disabled by host during sensor operation.
    pub feat_eng_dis: bool,
}

impl InternalError {
    pub fn from_reg(reg: u8) -> InternalError {
        InternalError {
            int_err_1: (reg & InternalErrorMask::INT_ERR_1) != 0,
            int_err_2: (reg & InternalErrorMask::INT_ERR_2) >> 2 != 0,
            feat_eng_dis: (reg & InternalErrorMask::FEAT_ENG_DIS) >> 4 != 0,
        }
    }
}

pub struct AuxIfTrimMask;
impl AuxIfTrimMask {
    pub const PUPSEL: u8 = 0b0000_0011;
}

/// Pull up configuration.
#[repr(u8)]
pub enum PullUpConf {
    /// Pull up off
    PullUpOff = 0x00,
    /// Pull up 40k.
    PullUp40K = 0x01,
    /// Pull up 10k.
    PullUp10K = 0x02,
    /// Pull up 2k.
    PullUp2K = 0x03,
}

impl PullUpConf {
    pub fn from_reg(reg: u8) -> PullUpConf {
        match reg & AuxIfTrimMask::PUPSEL {
            0x00 => PullUpConf::PullUpOff,
            0x01 => PullUpConf::PullUp40K,
            0x02 => PullUpConf::PullUp10K,
            0x03 => PullUpConf::PullUp2K,
            _ => panic!(), // TODO
        }
    }

    pub fn to_reg(self) -> u8 {
        self as u8
    }
}

pub struct GyrCrtConfMask;
impl GyrCrtConfMask {
    pub const CRT_RUNNING: u8 = 1 << 2;
    pub const RDY_FOR_DL: u8 = 1 << 3;
}

/// Crt data ready for download.
#[repr(u8)]
pub enum ReadyForDl {
    /// Pull up off
    OnGoing = 0x00,
    /// Pull up 40k.
    Complete = 0x01,
}

/// Component retrimming for gyroscope.
pub struct GyrCrtConf {
    /// Indicates that the CRT is currently running.
    pub crt_running: bool,
    /// Pacemaker bit for downloading CRT (ready only).
    pub rdy_for_dl: ReadyForDl,
}

impl GyrCrtConf {
    pub fn from_reg(reg: u8) -> GyrCrtConf {
        GyrCrtConf {
            crt_running: reg & GyrCrtConfMask::CRT_RUNNING >> 2 != 0,
            rdy_for_dl: match (reg & GyrCrtConfMask::RDY_FOR_DL) >> 3 {
                0x00 => ReadyForDl::OnGoing,
                0x01 => ReadyForDl::Complete,
                _ => panic!(), // TODO
            },
        }
    }

    pub fn to_reg(self) -> u8 {
        let crt_running = if self.crt_running { 0x01 } else { 0x00 };

        crt_running << 2
    }
}

pub struct IfConfMask;
impl IfConfMask {
    pub const SPI3: u8 = 1;
    pub const SPI3_OIS: u8 = 1 << 1;
    pub const OIS_EN: u8 = 1 << 4;
    pub const AUX_EN: u8 = 1 << 5;
}

/// SPI interface mode.
#[repr(u8)]
pub enum SpiMode {
    /// SPI 4-wire mode.
    Spi4 = 0x00,
    /// SPI 3-wire mode.
    Spi3 = 0x01,
}

/// Serial interface settings.
pub struct IfConf {
    /// SPI interface mode for primary interface.
    pub spi_mode: SpiMode,
    /// SPI interface mode for OIS interface.
    pub spi_mode_ois: SpiMode,
    /// OIS enable.
    pub ois_en: bool,
    /// AUX enable.
    pub aux_en: bool,
}

impl IfConf {
    pub fn from_reg(reg: u8) -> IfConf {
        IfConf {
            spi_mode: match reg & IfConfMask::SPI3 {
                0x00 => SpiMode::Spi4,
                0x01 => SpiMode::Spi3,
                _ => panic!(), // TODO
            },
            spi_mode_ois: match (reg & IfConfMask::SPI3_OIS) >> 1 {
                0x00 => SpiMode::Spi4,
                0x01 => SpiMode::Spi3,
                _ => panic!(), // TODO
            },
            ois_en: (reg & IfConfMask::OIS_EN) >> 2 != 0,
            aux_en: (reg & IfConfMask::AUX_EN) >> 3 != 0,
        }
    }

    pub fn to_reg(self) -> u8 {
        let spi_mode = self.spi_mode as u8;
        let spi_mode_ois = self.spi_mode_ois as u8;
        let ois_en = if self.ois_en { 0x01 } else { 0x00 };
        let aux_en = if self.aux_en { 0x01 } else { 0x00 };

        spi_mode | spi_mode_ois << 1 | ois_en << 2 | aux_en << 3
    }
}

pub struct DrvMask;
impl DrvMask {
    pub const IO_PAD_DRV1: u8 = 0b0000_0111;
    pub const IO_PAD_I2C_B1: u8 = 1 << 3;
    pub const IO_PAD_DRV2: u8 = 0b0111_0000;
    pub const IO_PAD_I2C_B2: u8 = 1 << 7;
}

/// Drive strength. L7 is 10x stronger than L0.
#[repr(u8)]
pub enum DriveStrength {
    L0 = 0b000,
    L1 = 0b001,
    L2 = 0b010,
    L3 = 0b011,
    L4 = 0b100,
    L5 = 0b101,
    L6 = 0b110,
    L7 = 0b111,
}

/// Drive strength control register.
pub struct Drv {
    /// Output pad drive strength for the SDO and SDx pins
    pub io_pad_drv1: DriveStrength,
    /// Enable additional increase in pull down strength of the SDx pin in I2C mode.
    pub io_pad_i2c_b1: bool,
    /// Output pad drive strength for the OSDO ASCx and ASDx pins
    pub io_pad_drv2: DriveStrength,
    /// Enable additional increase in pull down strength of the ASCx and ASDx pin in I2C mode.
    pub io_pad_i2c_b2: bool,
}

impl Drv {
    pub fn from_reg(reg: u8) -> Drv {
        Drv {
            io_pad_drv1: match reg & DrvMask::IO_PAD_DRV1 {
                0b000 => DriveStrength::L0,
                0b001 => DriveStrength::L1,
                0b010 => DriveStrength::L2,
                0b011 => DriveStrength::L3,
                0b100 => DriveStrength::L4,
                0b101 => DriveStrength::L5,
                0b110 => DriveStrength::L6,
                0b111 => DriveStrength::L7,
                _ => panic!(), // TODO
            },
            io_pad_i2c_b1: (reg & DrvMask::IO_PAD_I2C_B1) >> 3 != 0,
            io_pad_drv2: match (reg & DrvMask::IO_PAD_DRV2) >> 4 {
                0b000 => DriveStrength::L0,
                0b001 => DriveStrength::L1,
                0b010 => DriveStrength::L2,
                0b011 => DriveStrength::L3,
                0b100 => DriveStrength::L4,
                0b101 => DriveStrength::L5,
                0b110 => DriveStrength::L6,
                0b111 => DriveStrength::L7,
                _ => panic!(), // TODO
            },
            io_pad_i2c_b2: (reg & DrvMask::IO_PAD_I2C_B2) >> 7 != 0,
        }
    }

    pub fn to_reg(self) -> u8 {
        let io_pad_drv1 = self.io_pad_drv1 as u8;
        let io_pad_i2c_b1 = if self.io_pad_i2c_b1 { 0x01 } else { 0x00 };
        let io_pad_drv2 = self.io_pad_drv2 as u8;
        let io_pad_i2c_b2 = if self.io_pad_i2c_b2 { 0x01 } else { 0x00 };

        io_pad_drv1 | io_pad_i2c_b1 << 3 | io_pad_drv2 << 4 | io_pad_i2c_b2 << 7
    }
}

pub struct AccSelfTestMask;
impl AccSelfTestMask {
    pub const ACC_SELF_TEST_EN: u8 = 1;
    pub const ACC_SELF_TEST_SIGN: u8 = 1 << 2;
    pub const ACC_SELF_TEST_AMP: u8 = 1 << 3;
}

/// A sign.
#[repr(u8)]
pub enum Sign {
    /// Negative.
    Negative = 0x00,
    /// Positive.
    Positive = 0x01,
}

/// An amplitude.
#[repr(u8)]
pub enum Amplitude {
    /// Low..
    Low = 0x00,
    /// High.
    High = 0x01,
}

/// Accelerometer self test settings.
pub struct AccSelfTest {
    /// Enable accelerometer self test.
    pub enable: bool,
    /// Sign of self test excitation.
    pub sign: Sign,
    /// Amplitude of the self test deflection.
    pub amplitude: Amplitude,
}

impl AccSelfTest {
    pub fn from_reg(reg: u8) -> AccSelfTest {
        AccSelfTest {
            enable: (reg & AccSelfTestMask::ACC_SELF_TEST_EN) != 0,
            sign: match (reg & AccSelfTestMask::ACC_SELF_TEST_SIGN) >> 2 {
                0x00 => Sign::Negative,
                0x01 => Sign::Positive,
                _ => panic!(), // TODO
            },
            amplitude: match (reg & AccSelfTestMask::ACC_SELF_TEST_AMP) >> 3 {
                0x00 => Amplitude::Low,
                0x01 => Amplitude::High,
                _ => panic!(), // TODO
            },
        }
    }

    pub fn to_reg(self) -> u8 {
        let enable = if self.enable { 0x01 } else { 0x00 };
        let sign = self.sign as u8;
        let amplitude = self.amplitude as u8;

        enable | sign << 2 | amplitude << 3
    }
}

pub struct GyrSelfTestMask;
impl GyrSelfTestMask {
    pub const GYR_ST_AXES_DONE: u8 = 1;
    pub const GYR_AXIS_X_OK: u8 = 1 << 1;
    pub const GYR_AXIS_Y_OK: u8 = 1 << 2;
    pub const GYR_AXIS_Z_OK: u8 = 1 << 3;
}

/// Gyroscope self test settings.
pub struct GyrSelfTest {
    /// Functional test of detection channels finished.
    pub done: bool,
    /// Self test status of the x axis.
    pub x_ok: bool,
    /// Self test status of the y axis.
    pub y_ok: bool,
    /// Self test status of the z axis.
    pub z_ok: bool,
}

impl GyrSelfTest {
    pub fn from_reg(reg: u8) -> GyrSelfTest {
        GyrSelfTest {
            done: (reg & GyrSelfTestMask::GYR_ST_AXES_DONE) != 0,
            x_ok: (reg & GyrSelfTestMask::GYR_AXIS_X_OK) != 0,
            y_ok: (reg & GyrSelfTestMask::GYR_AXIS_Y_OK) != 0,
            z_ok: (reg & GyrSelfTestMask::GYR_AXIS_Z_OK) != 0,
        }
    }
}

pub struct NvConfMask;
impl NvConfMask {
    pub const SPI_EN: u8 = 1;
    pub const I2C_WDT_SEL: u8 = 1 << 1;
    pub const I2C_WDT_EN: u8 = 1 << 2;
    pub const ACC_OFF_EN: u8 = 1 << 3;
}

/// A timer period.
#[repr(u8)]
pub enum TimePedriod {
    /// A 1.25ms period.
    Short1_25Ms = 0x00,
    /// A 40ms period.
    Long40Ms = 0x01,
}

/// NVM backed configuration.
pub struct NvConf {
    /// Enable SPI as the primary interface instead of I2C.
    pub spi_en: bool,
    /// Timer period for the I2C watchdog.
    pub i2c_wdt_sel: TimePedriod,
    /// I2C watchdog fo the SDA pin.
    pub i2c_wdt_en: bool,
    /// Add the offset defined in the off_acc_[xyz] registers to filtered and unfiltered
    /// accelerometer data.
    pub acc_off_en: bool,
}

impl NvConf {
    pub fn from_reg(reg: u8) -> NvConf {
        NvConf {
            spi_en: (reg & NvConfMask::SPI_EN) != 0,
            i2c_wdt_sel: match (reg & NvConfMask::I2C_WDT_SEL) >> 1 {
                0x00 => TimePedriod::Short1_25Ms,
                0x01 => TimePedriod::Long40Ms,
                _ => panic!(), // TODO
            },
            i2c_wdt_en: (reg & NvConfMask::I2C_WDT_EN) != 0,
            acc_off_en: (reg & NvConfMask::ACC_OFF_EN) != 0,
        }
    }

    pub fn to_reg(self) -> u8 {
        let spi_en = if self.spi_en { 0x01 } else { 0x00 };
        let i2c_wdt_sel = self.i2c_wdt_sel as u8;
        let i2c_wdt_en = if self.i2c_wdt_en { 0x01 } else { 0x00 };
        let acc_off_en = if self.acc_off_en { 0x01 } else { 0x00 };

        spi_en | i2c_wdt_sel << 1 | i2c_wdt_en << 2 | acc_off_en << 3
    }
}

/// Accelerometer offsets compensation.
pub struct AccOffsets {
    /// Offset compensation for the x axis.
    pub x: u8,
    /// Offset compensation for the y axis.
    pub y: u8,
    /// Offset compensation for the z axis.
    pub z: u8,
}

/// Gyroscope offsets compensation.
pub struct GyrOffsets {
    /// Offset compensation for the x axis.
    pub x: u16,
    /// Offset compensation for the y axis.
    pub y: u16,
    /// Offset compensation for the z axis.
    pub z: u16,
    /// Enable offset for filtered and unfiltered data.
    pub offset_en: bool,
    /// Enable gain compensation.
    pub gain_en: bool,
}
