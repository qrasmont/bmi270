#![no_std]

use interface::{I2cInterface, ReadData, SpiInterface, WriteData};
use registers::Registers;
use types::{
    AccConf, AccRange, AuxConf, AuxData, AuxIfConf, AxisData, Data, Error, ErrorReg, Event,
    FifoConf, FifoDowns, GyrConf, GyrRange, InternalStatus, InterruptStatus, Saturation, Status,
    WristGestureActivity, FIFO_LENGTH_1_MASK,
};

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

        Ok(ErrorReg::from_reg(errors))
    }

    /// Get the sensor status.
    pub fn get_status(&mut self) -> Result<Status, Error<CommE, CsE>> {
        let status = self.iface.read_reg(Registers::STATUS)?;

        Ok(Status::from_reg(status))
    }

    /// Get the sensor auxiliary data.
    pub fn get_aux_data(&mut self) -> Result<AuxData, Error<CommE, CsE>> {
        let mut payload = [0_u8; 9];
        payload[0] = Registers::AUX_DATA_0;
        self.iface.read(&mut payload)?;

        Ok(AuxData {
            axis: payload_to_axis(&payload[1..7]),
            r: (i16::from(payload[7]) | (i16::from(payload[8]) << 8)),
        })
    }

    /// Get the sensor accelerometer data.
    pub fn get_acc_data(&mut self) -> Result<AxisData, Error<CommE, CsE>> {
        let mut payload = [0_u8; 7];
        payload[0] = Registers::ACC_DATA_0;
        self.iface.read(&mut payload)?;

        Ok(payload_to_axis(&payload[1..]))
    }

    /// Get the sensor gyroscope data.
    pub fn get_gyr_data(&mut self) -> Result<AxisData, Error<CommE, CsE>> {
        let mut payload = [0_u8; 7];
        payload[0] = Registers::GYR_DATA_0;
        self.iface.read(&mut payload)?;

        Ok(payload_to_axis(&payload[1..]))
    }

    /// Get the sensor time.
    pub fn get_sensor_time(&mut self) -> Result<u32, Error<CommE, CsE>> {
        let mut payload = [Registers::SENSORTIME_0, 0, 0, 0];
        self.iface.read(&mut payload)?;

        Ok(payload_to_sensortime(&payload[1..]))
    }

    /// Get all the sensor data (excluding auxiliary data).
    pub fn get_data(&mut self) -> Result<Data, Error<CommE, CsE>> {
        let mut payload = [0_u8; 16];
        payload[0] = Registers::ACC_DATA_0;
        self.iface.read(&mut payload)?;

        Ok(Data {
            acc: payload_to_axis(&payload[1..7]),
            gyr: payload_to_axis(&payload[7..13]),
            time: payload_to_sensortime(&payload[13..16]),
        })
    }

    /// Get the event register.
    pub fn get_event(&mut self) -> Result<Event, Error<CommE, CsE>> {
        let event = self.iface.read_reg(Registers::EVENT)?;

        Ok(Event::from_reg(event))
    }

    /// Get the interrupt/feature status.
    pub fn get_int_status(&mut self) -> Result<InterruptStatus, Error<CommE, CsE>> {
        let int_stat_0 = self.iface.read_reg(Registers::INT_STATUS_0)?;
        let int_stat_1 = self.iface.read_reg(Registers::INT_STATUS_1)?;

        Ok(InterruptStatus::from_regs(int_stat_0, int_stat_1))
    }

    /// Get the step count.
    pub fn get_step_count(&mut self) -> Result<u16, Error<CommE, CsE>> {
        let mut payload = [Registers::SC_OUT_0, 0, 0];
        self.iface.read(&mut payload)?;

        let steps: u16 = u16::from(payload[1]) | u16::from(payload[2]) << 8;

        Ok(steps)
    }

    /// Get the detected wrist gesture and activity type.
    pub fn get_wrist_gesture_activity(
        &mut self,
    ) -> Result<WristGestureActivity, Error<CommE, CsE>> {
        let wr_gest_acc = self.iface.read_reg(Registers::WR_GEST_ACT)?;

        Ok(WristGestureActivity::from_reg(wr_gest_acc))
    }

    /// Get the sensor internal status.
    pub fn get_internal_status(&mut self) -> Result<InternalStatus, Error<CommE, CsE>> {
        let internal_status = self.iface.read_reg(Registers::INTERNAL_STATUS)?;

        Ok(InternalStatus::from_reg(internal_status))
    }

    /// Get the sensor temperature.
    pub fn get_temperature(&mut self) -> Result<Option<f32>, Error<CommE, CsE>> {
        let mut payload = [Registers::TEMPERATURE_0, 0, 0];
        self.iface.read(&mut payload)?;
        let raw_temp = u16::from(payload[1]) | u16::from(payload[2]) << 8;

        Ok(match raw_temp {
            0x8000 => None,
            _ => Some(f32::from(raw_temp as i16) * (1.0_f32 / 512.0_f32) + 23.0),
        })
    }

    /// Get the current fill level of the FIFO buffer.
    pub fn get_fifo_len(&mut self) -> Result<i16, Error<CommE, CsE>> {
        let mut payload = [Registers::FIFO_LENGTH_0, 0, 0];
        self.iface.read(&mut payload)?;
        let len = i16::from(payload[1]) | i16::from(payload[2] & FIFO_LENGTH_1_MASK) << 8;

        Ok(len)
    }

    pub fn get_fifo_data(&mut self) -> Result<(), Error<CommE, CsE>> {
        // TODO Fifo is 6KB, will need the max read info from user + fifo config
        Ok(())
    }

    /// Get the accelerometer configuration.
    pub fn get_acc_conf(&mut self) -> Result<AccConf, Error<CommE, CsE>> {
        let acc_conf = self.iface.read_reg(Registers::ACC_CONF)?;
        Ok(AccConf::from_reg(acc_conf))
    }

    /// Set the accelerometer configuration.
    pub fn set_acc_conf(&mut self, acc_conf: AccConf) -> Result<(), Error<CommE, CsE>> {
        let reg = acc_conf.to_reg();
        self.iface.write_reg(Registers::ACC_CONF, reg)?;
        Ok(())
    }

    /// Get the accelerometer range.
    pub fn get_acc_range(&mut self) -> Result<AccRange, Error<CommE, CsE>> {
        let acc_range = self.iface.read_reg(Registers::ACC_RANGE)?;
        Ok(AccRange::from_reg(acc_range))
    }

    /// Set the accelerometer range.
    pub fn set_acc_range(&mut self, acc_range: AccRange) -> Result<(), Error<CommE, CsE>> {
        self.iface
            .write_reg(Registers::ACC_RANGE, acc_range as u8)?;
        Ok(())
    }

    /// Get the gyroscope configuration.
    pub fn get_gyr_conf(&mut self) -> Result<GyrConf, Error<CommE, CsE>> {
        let gyr_conf = self.iface.read_reg(Registers::GYR_CONF)?;
        Ok(GyrConf::from_reg(gyr_conf))
    }

    /// Set the gyroscope configuration.
    pub fn set_gyr_conf(&mut self, gyr_conf: GyrConf) -> Result<(), Error<CommE, CsE>> {
        let reg = gyr_conf.to_reg();
        self.iface.write_reg(Registers::GYR_CONF, reg)?;
        Ok(())
    }

    /// Get the gyroscope range.
    pub fn get_gyr_range(&mut self) -> Result<GyrRange, Error<CommE, CsE>> {
        let gyr_range = self.iface.read_reg(Registers::GYR_RANGE)?;
        Ok(GyrRange::from_reg(gyr_range))
    }

    /// Set the gyroscope configuration.
    pub fn set_gyr_range(&mut self, gyr_range: GyrRange) -> Result<(), Error<CommE, CsE>> {
        let reg = gyr_range.to_reg();
        self.iface.write_reg(Registers::GYR_RANGE, reg)?;
        Ok(())
    }

    /// Get the Auxiliary device configuration.
    pub fn get_aux_conf(&mut self) -> Result<AuxConf, Error<CommE, CsE>> {
        let aux_conf = self.iface.read_reg(Registers::AUX_CONF)?;
        Ok(AuxConf::from_reg(aux_conf))
    }

    /// Set the Auxiliary device configuration.
    pub fn set_aux_conf(&mut self, aux_conf: AuxConf) -> Result<(), Error<CommE, CsE>> {
        let reg = aux_conf.to_reg();
        self.iface.write_reg(Registers::AUX_CONF, reg)?;
        Ok(())
    }

    /// Get the fifo downsampling configuration.
    pub fn get_fifo_downs(&mut self) -> Result<FifoDowns, Error<CommE, CsE>> {
        let fifo_downs = self.iface.read_reg(Registers::FIFO_DOWNS)?;
        Ok(FifoDowns::from_reg(fifo_downs))
    }

    /// Set the fifo downsampling configuration.
    pub fn set_fifo_downs(&mut self, fifo_downs: FifoDowns) -> Result<(), Error<CommE, CsE>> {
        let reg = fifo_downs.to_reg();
        self.iface.write_reg(Registers::FIFO_DOWNS, reg)?;
        Ok(())
    }

    /// Get the fifo watermark level.
    pub fn get_fifo_wtm(&mut self) -> Result<u16, Error<CommE, CsE>> {
        let mut payload = [Registers::FIFO_WTM_0, 0, 0];
        self.iface.read(&mut payload)?;
        Ok(u16::from(payload[1]) | u16::from(payload[2]) << 8)
    }

    /// Set the fifo watermark level. Interrupt will trigger when the fifo reaches wtm * 256 bytes.
    pub fn set_fifo_wtm(&mut self, wtm: u16) -> Result<(), Error<CommE, CsE>> {
        let reg_0 = wtm as u8;
        let reg_1 = (wtm >> 8) as u8;
        let mut payload = [Registers::FIFO_WTM_0, reg_0, reg_1];
        self.iface.write(&mut payload)?;
        Ok(())
    }

    /// Get the fifo configuration.
    pub fn get_fifo_conf(&mut self) -> Result<FifoConf, Error<CommE, CsE>> {
        let mut payload = [Registers::FIFO_CONFIG_0, 0, 0];
        self.iface.read(&mut payload)?;
        Ok(FifoConf::from_regs(payload[1], payload[2]))
    }

    /// Set the fifo configuration.
    pub fn set_fifo_conf(&mut self, fifo_conf: FifoConf) -> Result<(), Error<CommE, CsE>> {
        let (reg_0, reg_1) = fifo_conf.to_regs();
        let mut payload = [Registers::FIFO_CONFIG_0, reg_0, reg_1];
        self.iface.write(&mut payload)?;
        Ok(())
    }

    /// Get the current saturation.
    pub fn get_saturation(&mut self) -> Result<Saturation, Error<CommE, CsE>> {
        let saturation = self.iface.read_reg(Registers::SATURATION)?;
        Ok(Saturation::from_reg(saturation))
    }

    /// Get the auxiliary device id.
    pub fn get_aux_dev_id(&mut self) -> Result<u8, Error<CommE, CsE>> {
        let dev_id = self.iface.read_reg(Registers::AUX_DEV_ID)?;
        Ok(dev_id >> 1)
    }

    /// Set the auxiliary device id.
    pub fn set_aux_dev_id(&mut self, dev_id: u8) -> Result<(), Error<CommE, CsE>> {
        let reg = dev_id << 1;
        self.iface.write_reg(Registers::AUX_DEV_ID, reg)?;
        Ok(())
    }

    /// Get auxiliary device interface configuration.
    pub fn get_aux_if_conf(&mut self) -> Result<AuxIfConf, Error<CommE, CsE>> {
        let aux_if_conf = self.iface.read_reg(Registers::AUX_IF_CONF)?;
        Ok(AuxIfConf::from_reg(aux_if_conf))
    }

    /// Set auxiliary device interface configuration.
    pub fn set_aux_if_conf(&mut self, aux_if_conf: AuxIfConf) -> Result<(), Error<CommE, CsE>> {
        let reg = aux_if_conf.to_reg();
        self.iface.write_reg(Registers::AUX_IF_CONF, reg)?;
        Ok(())
    }

    /// Get auxiliary device read address.
    pub fn get_aux_rd_addr(&mut self) -> Result<u8, Error<CommE, CsE>> {
        let aux_rd_addr = self.iface.read_reg(Registers::AUX_RD_ADDR)?;
        Ok(aux_rd_addr)
    }

    /// Set auxiliary device read address.
    pub fn set_aux_rd_addr(&mut self, aux_rd_addr: u8) -> Result<(), Error<CommE, CsE>> {
        self.iface.write_reg(Registers::AUX_RD_ADDR, aux_rd_addr)?;
        Ok(())
    }

    /// Get auxiliary device write address.
    pub fn get_aux_wr_addr(&mut self) -> Result<u8, Error<CommE, CsE>> {
        let aux_wr_addr = self.iface.read_reg(Registers::AUX_WR_ADDR)?;
        Ok(aux_wr_addr)
    }

    /// Set auxiliary device write address.
    pub fn set_aux_wr_addr(&mut self, aux_wr_addr: u8) -> Result<(), Error<CommE, CsE>> {
        self.iface.write_reg(Registers::AUX_WR_ADDR, aux_wr_addr)?;
        Ok(())
    }

    /// Get auxiliary device data to write.
    pub fn get_aux_wr_data(&mut self) -> Result<u8, Error<CommE, CsE>> {
        let aux_wr_data = self.iface.read_reg(Registers::AUX_WR_DATA)?;
        Ok(aux_wr_data)
    }

    /// Set auxiliary device data to write.
    pub fn set_aux_wr_data(&mut self, aux_wr_data: u8) -> Result<(), Error<CommE, CsE>> {
        self.iface.write_reg(Registers::AUX_WR_DATA, aux_wr_data)?;
        Ok(())
    }
}

fn payload_to_axis(payload: &[u8]) -> AxisData {
    AxisData {
        x: (i16::from(payload[0]) | (i16::from(payload[1]) << 8)),
        y: (i16::from(payload[2]) | (i16::from(payload[3]) << 8)),
        z: (i16::from(payload[4]) | (i16::from(payload[5]) << 8)),
    }
}

fn payload_to_sensortime(payload: &[u8]) -> u32 {
    u32::from(payload[0]) | (u32::from(payload[1]) << 8) | (u32::from(payload[2]) << 16)
}
