#![no_std]

use interface::{I2cInterface, SpiInterface, WriteData, ReadData};
use registers::Registers;
use types::Error;

pub mod interface;
mod types;
mod registers;

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
    I: ReadData<Error = Error<CommE, CsE>> + WriteData<Error = Error<CommE, CsE>>
{
    /// Get the chip id.
    pub fn get_chip_id(&mut self) -> Result<u8, Error<CommE, CsE>> {
        self.iface.read_reg(Registers::CHIP_ID)
    }
}
