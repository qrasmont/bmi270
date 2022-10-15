use fixedvec::FixedVec;

use crate::interface::{I2cAddr, I2cInterface, ReadData, SpiInterface, WriteData};
use crate::registers::Registers;
use crate::config::BMI270_CONFIG_FILE;

use crate::types::{
    AccConf, AccOffsets, AccRange, AccSelfTest, AuxConf, AuxData, AuxIfConf, AxisData, Burst, Cmd,
    Data, Drv, Error, ErrorReg, ErrorRegMsk, Event, FifoConf, FifoDowns, GyrConf, GyrCrtConf,
    GyrOffsets, GyrRange, GyrSelfTest, IfConf, IntIoCtrl, IntLatch, IntMapData, IntMapFeat,
    InternalError, InternalStatus, InterruptStatus, NvConf, PullUpConf, PwrConf, PwrCtrl,
    Saturation, Status, WristGestureActivity, FIFO_LENGTH_1_MASK,
};

pub struct Bmi270<I> {
    iface: I,
    max_burst: u16,
}

impl<I2C> Bmi270<I2cInterface<I2C>> {
    /// Create a new Bmi270 device with I2C communication.
    pub fn new_i2c(i2c: I2C, address: I2cAddr, burst: Burst) -> Self {
        Bmi270 {
            iface: I2cInterface {
                i2c,
                address: address.addr(),
            },
            max_burst: burst.val(),
        }
    }

    /// Release I2C.
    pub fn release(self) -> I2C {
        self.iface.i2c
    }
}

impl<SPI, CS> Bmi270<SpiInterface<SPI, CS>> {
    /// Create a new Bmi270 device with SPI communication.
    pub fn new_spi(spi: SPI, cs: CS, burst: Burst) -> Self {
        Bmi270 {
            iface: SpiInterface { spi, cs },
            max_burst: burst.val(),
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

    /// Get error register mask.
    pub fn get_err_reg_msk(&mut self) -> Result<ErrorRegMsk, Error<CommE, CsE>> {
        let err_reg_msk = self.iface.read_reg(Registers::ERR_REG_MSK)?;
        Ok(ErrorRegMsk::from_reg(err_reg_msk))
    }

    /// Get error register mask.
    pub fn set_err_reg_msk(&mut self, err_reg_msk: ErrorRegMsk) -> Result<(), Error<CommE, CsE>> {
        let reg = err_reg_msk.to_reg();
        self.iface.write_reg(Registers::ERR_REG_MSK, reg)?;
        Ok(())
    }

    /// Get interrupt 1 io control.
    pub fn get_int1_io_ctrl(&mut self) -> Result<IntIoCtrl, Error<CommE, CsE>> {
        let int1_io_ctrl = self.iface.read_reg(Registers::INT1_IO_CTRL)?;
        Ok(IntIoCtrl::from_reg(int1_io_ctrl))
    }

    /// Set interrupt 1 io control.
    pub fn set_int1_io_ctrl(&mut self, int1_io_ctrl: IntIoCtrl) -> Result<(), Error<CommE, CsE>> {
        let reg = int1_io_ctrl.to_reg();
        self.iface.write_reg(Registers::INT1_IO_CTRL, reg)?;
        Ok(())
    }

    /// Get interrupt 2 io control.
    pub fn get_int2_io_ctrl(&mut self) -> Result<IntIoCtrl, Error<CommE, CsE>> {
        let int2_io_ctrl = self.iface.read_reg(Registers::INT2_IO_CTRL)?;
        Ok(IntIoCtrl::from_reg(int2_io_ctrl))
    }

    /// Set interrupt 2 io control.
    pub fn set_int2_io_ctrl(&mut self, int2_io_ctrl: IntIoCtrl) -> Result<(), Error<CommE, CsE>> {
        let reg = int2_io_ctrl.to_reg();
        self.iface.write_reg(Registers::INT2_IO_CTRL, reg)?;
        Ok(())
    }

    /// Get interrupt latched mode.
    pub fn get_int_latch(&mut self) -> Result<IntLatch, Error<CommE, CsE>> {
        let int_latch = self.iface.read_reg(Registers::INT_LATCH)?;
        Ok(IntLatch::from_reg(int_latch))
    }

    /// Set interrupt latched mode.
    pub fn set_int_latch(&mut self, int_latch: IntLatch) -> Result<(), Error<CommE, CsE>> {
        self.iface
            .write_reg(Registers::INT_LATCH, int_latch as u8)?;
        Ok(())
    }

    /// Get interrupt 1 feature mapping.
    pub fn get_int1_map_feat(&mut self) -> Result<IntMapFeat, Error<CommE, CsE>> {
        let int1_map_feat = self.iface.read_reg(Registers::INT1_MAP_FEAT)?;
        Ok(IntMapFeat::from_reg(int1_map_feat))
    }

    /// Set interrupt 1 feature mapping.
    pub fn set_int1_map_feat(
        &mut self,
        int1_map_feat: IntMapFeat,
    ) -> Result<(), Error<CommE, CsE>> {
        let reg = int1_map_feat.to_reg();
        self.iface.write_reg(Registers::INT1_MAP_FEAT, reg)?;
        Ok(())
    }

    /// Get interrupt 2 feature mapping.
    pub fn get_int2_map_feat(&mut self) -> Result<IntMapFeat, Error<CommE, CsE>> {
        let int2_map_feat = self.iface.read_reg(Registers::INT2_MAP_FEAT)?;
        Ok(IntMapFeat::from_reg(int2_map_feat))
    }

    /// Set interrupt 2 feature mapping.
    pub fn set_int2_map_feat(
        &mut self,
        int2_map_feat: IntMapFeat,
    ) -> Result<(), Error<CommE, CsE>> {
        let reg = int2_map_feat.to_reg();
        self.iface.write_reg(Registers::INT2_MAP_FEAT, reg)?;
        Ok(())
    }

    /// Get interrupt data map.
    pub fn get_int_map_data(&mut self) -> Result<IntMapData, Error<CommE, CsE>> {
        let int_map_data = self.iface.read_reg(Registers::INT_MAP_DATA)?;
        Ok(IntMapData::from_reg(int_map_data))
    }

    /// Set interrupt data map.
    pub fn set_int_map_data(&mut self, int_map_data: IntMapData) -> Result<(), Error<CommE, CsE>> {
        let reg = int_map_data.to_reg();
        self.iface.write_reg(Registers::INT_LATCH, reg)?;
        Ok(())
    }

    /// Get initialization control register.
    pub fn get_init_ctrl(&mut self) -> Result<u8, Error<CommE, CsE>> {
        let init_ctrl = self.iface.read_reg(Registers::INIT_CTRL)?;
        Ok(init_ctrl)
    }

    /// Set initialization control register (start initialization).
    pub fn set_init_ctrl(&mut self, init_ctrl: u8) -> Result<(), Error<CommE, CsE>> {
        self.iface.write_reg(Registers::INIT_CTRL, init_ctrl)?;
        Ok(())
    }

    /// Get init address.
    pub fn get_init_addr(&mut self) -> Result<u16, Error<CommE, CsE>> {
        let mut payload = [Registers::INIT_ADDR_0, 0, 0];
        self.iface.read(&mut payload)?;
        Ok(u16::from(payload[1] & 0b0000_1111) | u16::from(payload[2]) << 4)
    }

    /// Set init address.
    pub fn set_init_addr(&mut self, init_addr: u16) -> Result<(), Error<CommE, CsE>> {
        let reg_0 = init_addr as u8 & 0b0000_1111;
        let reg_1 = (init_addr >> 4) as u8;
        let mut payload = [Registers::INIT_ADDR_0, reg_0, reg_1];
        self.iface.write(&mut payload)?;
        Ok(())
    }

    /// Get the initialization register.
    pub fn get_init_data(&mut self) -> Result<u8, Error<CommE, CsE>> {
        let init_data = self.iface.read_reg(Registers::INIT_DATA)?;
        Ok(init_data)
    }

    /// Set the initialization register.
    pub fn set_init_data(&mut self, init_data: u8) -> Result<(), Error<CommE, CsE>> {
        self.iface.write_reg(Registers::INIT_DATA, init_data)?;
        Ok(())
    }

    /// Get the internal errors.
    pub fn get_internal_error(&mut self) -> Result<InternalError, Error<CommE, CsE>> {
        let internal_error = self.iface.read_reg(Registers::INTERNAL_ERROR)?;
        Ok(InternalError::from_reg(internal_error))
    }

    /// Get ASDA pull up.
    pub fn get_asda_pullup(&mut self) -> Result<PullUpConf, Error<CommE, CsE>> {
        let aux_if_trim = self.iface.read_reg(Registers::AUX_IF_TRIM)?;
        Ok(PullUpConf::from_reg(aux_if_trim))
    }

    /// Set ASDA pull up.
    pub fn set_asda_pullup(&mut self, pull_up_conf: PullUpConf) -> Result<(), Error<CommE, CsE>> {
        self.iface
            .write_reg(Registers::AUX_IF_TRIM, pull_up_conf.to_reg())?;
        Ok(())
    }

    /// Get gyroscope component retrimming register.
    pub fn get_gyr_crt_conf(&mut self) -> Result<GyrCrtConf, Error<CommE, CsE>> {
        let gyr_crt_conf = self.iface.read_reg(Registers::GYR_CRT_CONF)?;
        Ok(GyrCrtConf::from_reg(gyr_crt_conf))
    }

    /// Set gyroscope component retrimming register.
    /// GyrCrtConf.rdy_for_dl is read-only, only crt_running will be set.
    pub fn set_gyr_crt_conf(&mut self, gyr_crt_conf: GyrCrtConf) -> Result<(), Error<CommE, CsE>> {
        self.iface
            .write_reg(Registers::GYR_CRT_CONF, gyr_crt_conf.to_reg())?;
        Ok(())
    }

    /// Get NVM configuration.
    pub fn get_nvm_conf(&mut self) -> Result<bool, Error<CommE, CsE>> {
        let nvm_conf = self.iface.read_reg(Registers::NVM_CONF)?;
        Ok((nvm_conf & 1 << 1) != 0)
    }

    /// Set NVM configuration.
    pub fn set_nvm_conf(&mut self, gyr_crt_conf: bool) -> Result<(), Error<CommE, CsE>> {
        let value: u8 = if gyr_crt_conf { 0x01 } else { 0x00 };
        self.iface.write_reg(Registers::NVM_CONF, value << 1)?;
        Ok(())
    }

    /// Get the interface configuration.
    pub fn get_if_conf(&mut self) -> Result<IfConf, Error<CommE, CsE>> {
        let if_conf = self.iface.read_reg(Registers::IF_CONF)?;
        Ok(IfConf::from_reg(if_conf))
    }

    /// Set the interface configuration.
    pub fn set_if_conf(&mut self, if_conf: IfConf) -> Result<(), Error<CommE, CsE>> {
        self.iface.write_reg(Registers::IF_CONF, if_conf.to_reg())?;
        Ok(())
    }

    /// Get the drive strength configuration.
    pub fn get_drv(&mut self) -> Result<Drv, Error<CommE, CsE>> {
        let drv = self.iface.read_reg(Registers::DRV)?;
        Ok(Drv::from_reg(drv))
    }

    /// Set the drive strength configuration.
    pub fn set_drv(&mut self, drv: Drv) -> Result<(), Error<CommE, CsE>> {
        self.iface.write_reg(Registers::DRV, drv.to_reg())?;
        Ok(())
    }

    /// Get the accelerometer self test configuration.
    pub fn get_acc_self_test(&mut self) -> Result<AccSelfTest, Error<CommE, CsE>> {
        let acc_self_test = self.iface.read_reg(Registers::ACC_SELF_TEST)?;
        Ok(AccSelfTest::from_reg(acc_self_test))
    }

    /// Set the accelerometer self test configuration.
    pub fn set_acc_self_test(
        &mut self,
        acc_self_test: AccSelfTest,
    ) -> Result<(), Error<CommE, CsE>> {
        self.iface
            .write_reg(Registers::ACC_SELF_TEST, acc_self_test.to_reg())?;
        Ok(())
    }

    /// Get the gyroscope self test configuration.
    pub fn get_gyr_self_test(&mut self) -> Result<GyrSelfTest, Error<CommE, CsE>> {
        let gyr_self_test = self.iface.read_reg(Registers::GYR_SELF_TEST)?;
        Ok(GyrSelfTest::from_reg(gyr_self_test))
    }

    /// Get NV configuration.
    pub fn get_nv_conf(&mut self) -> Result<NvConf, Error<CommE, CsE>> {
        let nv_conf = self.iface.read_reg(Registers::NV_CONF)?;
        Ok(NvConf::from_reg(nv_conf))
    }

    /// Set NV configuration.
    pub fn set_nv_conf(&mut self, nv_conf: NvConf) -> Result<(), Error<CommE, CsE>> {
        self.iface.write_reg(Registers::NV_CONF, nv_conf.to_reg())?;
        Ok(())
    }

    /// Get accelerometer offsets.
    pub fn get_acc_offsets(&mut self) -> Result<AccOffsets, Error<CommE, CsE>> {
        let mut payload = [Registers::OFFSET_0, 0, 0, 0];
        self.iface.read(&mut payload)?;
        Ok(AccOffsets {
            x: payload[1],
            y: payload[2],
            z: payload[3],
        })
    }

    /// Set accelerometer offsets.
    pub fn set_acc_offsets(&mut self, acc_offsets: AccOffsets) -> Result<(), Error<CommE, CsE>> {
        let mut payload = [
            Registers::OFFSET_0,
            acc_offsets.x,
            acc_offsets.y,
            acc_offsets.z,
        ];
        self.iface.write(&mut payload)?;
        Ok(())
    }

    /// Get gyroscope offsets.
    pub fn get_gyr_offsets(&mut self) -> Result<GyrOffsets, Error<CommE, CsE>> {
        let mut payload = [0_u8; 5];
        payload[0] = Registers::OFFSET_3;
        self.iface.read(&mut payload)?;

        let x = u16::from(payload[1]) | u16::from(payload[4] & 0b0000_0011) << 8;
        let y = u16::from(payload[2]) | u16::from(payload[4] & 0b0000_1100) << 6;
        let z = u16::from(payload[3]) | u16::from(payload[4] & 0b0011_0000) << 4;
        let offset_en = (payload[4] & 1 << 6) != 0;
        let gain_en = (payload[4] & 1 << 7) != 0;

        Ok(GyrOffsets {
            x,
            y,
            z,
            offset_en,
            gain_en,
        })
    }

    /// Set gyroscope offsets.
    pub fn set_gyr_offsets(&mut self, gyr_offsets: GyrOffsets) -> Result<(), Error<CommE, CsE>> {
        let mut payload = [0_u8; 5];
        payload[0] = Registers::OFFSET_3;

        payload[1] = gyr_offsets.x as u8;
        payload[2] = gyr_offsets.y as u8;
        payload[3] = gyr_offsets.z as u8;

        payload[4] |= (gyr_offsets.x >> 8 & 0b0000_0011) as u8;
        payload[4] |= (gyr_offsets.y >> 6 & 0b0000_1100) as u8;
        payload[4] |= (gyr_offsets.z >> 4 & 0b0011_0000) as u8;
        payload[4] |= if gyr_offsets.offset_en { 0x01 } else { 0x00 } << 6;
        payload[4] |= if gyr_offsets.gain_en { 0x01 } else { 0x00 } << 7;
        self.iface.write(&mut payload)?;

        Ok(())
    }

    /// Get power configuration.
    pub fn get_pwr_conf(&mut self) -> Result<PwrConf, Error<CommE, CsE>> {
        let pwr_conf = self.iface.read_reg(Registers::PWR_CONF)?;
        Ok(PwrConf::from_reg(pwr_conf))
    }

    /// Set power configuration.
    pub fn set_pwr_conf(&mut self, pwr_conf: PwrConf) -> Result<(), Error<CommE, CsE>> {
        self.iface
            .write_reg(Registers::PWR_CONF, pwr_conf.to_reg())?;
        Ok(())
    }

    /// Get power control.
    pub fn get_pwr_ctrl(&mut self) -> Result<PwrCtrl, Error<CommE, CsE>> {
        let pwr_ctrl = self.iface.read_reg(Registers::PWR_CTRL)?;
        Ok(PwrCtrl::from_reg(pwr_ctrl))
    }

    /// Set power control.
    pub fn set_pwr_ctrl(&mut self, pwr_ctrl: PwrCtrl) -> Result<(), Error<CommE, CsE>> {
        self.iface
            .write_reg(Registers::PWR_CTRL, pwr_ctrl.to_reg())?;
        Ok(())
    }

    /// Send a command.
    pub fn send_cmd(&mut self, cmd: Cmd) -> Result<(), Error<CommE, CsE>> {
        self.iface.write_reg(Registers::CMD, cmd as u8)?;
        Ok(())
    }

    /// Initialize sensor.
    pub fn init(&mut self) -> Result<(), Error<CommE, CsE>> {
        // TODO allw config of pre alloc
        let mut preallocated_space = alloc_stack!([u8; 512]);
        let mut vec = FixedVec::new(&mut preallocated_space);

        let mut offset = 0u16;
        let max_len = BMI270_CONFIG_FILE.len() as u16;
        let burst = self.max_burst - 1; // Remove 1 for address byte

        self.set_init_ctrl(0)?;

        while offset < max_len {
            self.set_init_addr(offset / 2)?;

            let end = offset + burst % max_len;

            vec.push(Registers::INIT_DATA)
                .map_err(|_| Error::<CommE, CsE>::Alloc)?;

            vec.push_all(&BMI270_CONFIG_FILE[offset as usize..end as usize])
                .map_err(|_| Error::<CommE, CsE>::Alloc)?;

            self.iface.write(&mut vec.as_mut_slice())?;

            offset += burst;
            vec.clear();
        }

        self.set_init_ctrl(1)?;

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
