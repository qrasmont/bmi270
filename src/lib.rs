#![no_std]

use interface::{I2cInterface, ReadData, SpiInterface, WriteData};
use registers::{
    ActivityOutVal, ErrRegBits, EventBits, InternalStatusBits, InterruptStatus0Bits,
    InterruptStatus1Bits, MessageVal, PersistentErrVal, Registers, StatusBits,
    WristGestureActivityBits, WristGestureOutVal, FIFO_LENGTH_1_BITS,
};
use types::{
    Activity, AuxData, AxisData, Data, Error, ErrorReg, Event, InternalStatus, InterruptStatus,
    Message, PersistentErrors, Status, WristGesture, WristGestureActivity,
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

        Ok(Event {
            por_detected: (event & EventBits::POR_DETECTED) != 0,
            persistent_err: match (event & EventBits::ERR_CODE) >> 2 {
                PersistentErrVal::NO_ERR => PersistentErrors::NoErr,
                PersistentErrVal::ACC_ERR => PersistentErrors::AccErr,
                PersistentErrVal::GYR_ERR => PersistentErrors::GyrErr,
                PersistentErrVal::ACC_GYR_ERR => PersistentErrors::AccGyrErr,
                _ => panic!(), // TODO
            },
        })
    }

    /// Get the interrupt/feature status.
    pub fn get_int_status(&mut self) -> Result<InterruptStatus, Error<CommE, CsE>> {
        let int_stat_0 = self.iface.read_reg(Registers::INT_STATUS_0)?;
        let int_stat_1 = self.iface.read_reg(Registers::INT_STATUS_1)?;

        Ok(InterruptStatus {
            sig_motion_out: (int_stat_0 & InterruptStatus0Bits::SIG_MOTION_OUT) != 0,
            step_counter_out: (int_stat_0 & InterruptStatus0Bits::STEP_COUNTER_OUT) != 0,
            activity_out: (int_stat_0 & InterruptStatus0Bits::ACTIVITY_OUT) != 0,
            wrist_wear_wakeup_out: (int_stat_0 & InterruptStatus0Bits::WRIST_WEAR_WAKEUP_OUT) != 0,
            wrist_gesture_out: (int_stat_0 & InterruptStatus0Bits::WRIST_GESTURE_OUT) != 0,
            no_motion_out: (int_stat_0 & InterruptStatus0Bits::NO_MOTION_OUT) != 0,
            any_motion_out: (int_stat_0 & InterruptStatus0Bits::ANY_MOTION_OUT) != 0,
            ffull_int: (int_stat_1 & InterruptStatus1Bits::FFULL_INT) != 0,
            fwm_int: (int_stat_1 & InterruptStatus1Bits::FWM_INT) != 0,
            err_int: (int_stat_1 & InterruptStatus1Bits::ERR_INT) != 0,
            aux_drdy_int: (int_stat_1 & InterruptStatus1Bits::AUX_DRDY_INT) != 0,
            gyr_drdy_int: (int_stat_1 & InterruptStatus1Bits::GYR_DRDY_INT) != 0,
            acc_drdy_int: (int_stat_1 & InterruptStatus1Bits::ACC_DRDY_INT) != 0,
        })
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

        Ok(WristGestureActivity {
            wrist_gesture: match wr_gest_acc & WristGestureActivityBits::WRIST_GESTURE {
                WristGestureOutVal::UNKNOWN => WristGesture::Unknown,
                WristGestureOutVal::PUSH_ARM_DOWN => WristGesture::PushArmDown,
                WristGestureOutVal::PIVOT_UP => WristGesture::PivotUp,
                WristGestureOutVal::SHAKE => WristGesture::Shake,
                WristGestureOutVal::FLICK_IN => WristGesture::FlickIn,
                WristGestureOutVal::FLICK_OUT => WristGesture::FlickOut,
                _ => panic!(), // TODO
            },
            activity: match (wr_gest_acc & WristGestureActivityBits::ACTIVITY) >> 3 {
                ActivityOutVal::STILL => Activity::Still,
                ActivityOutVal::WALKING => Activity::Walking,
                ActivityOutVal::RUNNING => Activity::Running,
                ActivityOutVal::UNKNOWN => Activity::Unknown,
                _ => panic!(), // TODO
            },
        })
    }

    /// Get the sensor internal status.
    pub fn get_internal_status(&mut self) -> Result<InternalStatus, Error<CommE, CsE>> {
        let internal_status = self.iface.read_reg(Registers::INTERNAL_STATUS)?;

        Ok(InternalStatus {
            message: match internal_status & InternalStatusBits::MESSAGE {
                MessageVal::NOT_INIT => Message::NotInit,
                MessageVal::INIT_OK => Message::InitOk,
                MessageVal::INIT_ERR => Message::InitErr,
                MessageVal::DRV_ERR => Message::DrvErr,
                MessageVal::SNS_ERR => Message::SnsErr,
                MessageVal::NVM_ERR => Message::NvmErr,
                MessageVal::STARTUP_ERR => Message::StartUpErr,
                MessageVal::COMPAT_ERR => Message::CompatErr,
                _ => panic!(), // TODO
            },
            axes_remap_error: (internal_status & InternalStatusBits::AXES_REMAP_ERROR) != 0,
            odr_50hz_error: (internal_status & InternalStatusBits::ODR_50HZ_ERROR) != 0,
        })
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
        let len = i16::from(payload[1]) | i16::from(payload[2] & FIFO_LENGTH_1_BITS) << 8;

        Ok(len)
    }

    pub fn get_fifo_data(&mut self) -> Result<(), Error<CommE, CsE>> {
        // TODO Fifo is 6KB, will need the max read info from user + fifo config
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
