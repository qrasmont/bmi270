#![no_std]

use interface::{I2cInterface, ReadData, SpiInterface, WriteData};
use registers::{ErrRegBits, Registers, StatusBits};
use types::{Error, ErrorReg, Status};

pub mod interface;
mod registers;
mod types;

pub struct Bmi270<I> {
    iface: I,
}

impl<I2C> Bmi270<I2cInterface<I2C>> {
    /// Create a new Bmi270 device with I2C communication.
    pub fn new_i2c(i2c: I2C) -> Self {
        Bmi270 {
            iface: I2cInterface { i2c },
        }
    }

    /// Release I2C.
    pub fn release(self) -> I2C {
        self.iface.i2c
    }
}

impl<SPI, CS> Bmi270<SpiInterface<SPI, CS>> {
    /// Create a new Bmi270 device with SPI communication.
    pub fn new_i2c(spi: SPI, cs: CS) -> Self {
        Bmi270 {
            iface: SpiInterface { spi, cs },
        }
    }

    /// Release I2C and CS.
    pub fn release(self) -> (SPI, CS) {
        (self.iface.spi, self.iface.cs)
    }
}

impl<I, CommE, CsE> Bmi270<I>
where
    I: ReadData<Error = Error<CommE, CsE>> + WriteData<Error = Error<CommE, CsE>>,
{
    /// Get the chip id.
    pub fn get_chip_id(&mut self) -> Result<u8, Error<CommE, CsE>> {
        self.iface.read_reg(Registers::CHIP_ID)
    }

    /// Get the errors from the error register.
    pub fn get_errors(&mut self) -> Result<ErrorReg, Error<CommE, CsE>> {
        let errors = self.iface.read_reg(Registers::ERR_REG)?;

        Ok(ErrorReg {
            fatal_err: (errors & ErrRegBits::FATAL_ERR) != 0,
            internal_err: errors & ErrRegBits::INTERNAL_ERR,
            fifo_err: (errors & ErrRegBits::FIFO_ERR) != 0,
            aux_err: (errors & ErrRegBits::AUX_ERR) != 0,
        })
    }

    /// Get the sensor status.
    pub fn get_status(&mut self) -> Result<Status, Error<CommE, CsE>> {
        let status = self.iface.read_reg(Registers::STATUS)?;

        Ok(Status {
            acc_data_ready: (status & StatusBits::DRDY_ACC) != 0,
            gyr_data_ready: (status & StatusBits::DRDY_GYR) != 0,
            aux_data_ready: (status & StatusBits::DRDY_AUX) != 0,
            cmd_ready: (status & StatusBits::CMD_RDY) != 0,
            aux_dev_busy: (status & StatusBits::AUX_BUSY) != 0,
        })
    }
}
