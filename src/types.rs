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
