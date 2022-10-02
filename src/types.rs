/// The possible errors that could be encountered.
pub enum Error<CommE, CsE> {
    /// Communication error over I2C or SPI.
    Comm(CommE),
    /// Pin error on the SPI chip select.
    Cs(CsE),
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

/// Possible persistent errors.
pub enum PersistentErrors {
    /// No errors reported.
    NoErr,
    /// Error in the accelerometer config register.
    AccErr,
    /// Error in the gyroscope config register.
    GyrErr,
    /// Error in both the accelerometer and gyroscope config registers.
    AccGyrErr,
}

/// Sensor event flags. Will be cleared on read when bit 0 is sent out over the bus.
pub struct Event {
    /// True after device power up or softreset. False after status read.
    pub por_detected: bool,
    /// Persistent errors.
    pub persistent_err: PersistentErrors,
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

/// Wrist gestures.
pub enum WristGesture {
    /// Unknown.
    Unknown,
    /// Push the arm down.
    PushArmDown,
    /// Pivot up.
    PivotUp,
    /// Wrist shake/jiggle.
    Shake,
    /// Arm flick in.
    FlickIn,
    /// Arm flick out.
    FlickOut,
}

/// Activity detection.
pub enum Activity {
    /// User stationnary.
    Still,
    /// User walking.
    Walking,
    /// User running.
    Running,
    /// Unknown state.
    Unknown,
}

/// Wrist gesture and activity.
pub struct WristGestureActivity {
    /// Wrist gesture.
    pub wrist_gesture: WristGesture,
    /// Activity.
    pub activity: Activity,
}
