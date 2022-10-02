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
