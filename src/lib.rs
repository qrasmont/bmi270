#![no_std]

#[macro_use]
extern crate fixedvec;

mod registers;
mod types;
pub mod config;

pub mod interface;
pub use interface::I2cAddr;
pub use types::{
    AccConf, AccOffsets, AccRange, AccSelfTest, AuxConf, AuxData, AuxIfConf, AxisData, Burst, Cmd,
    Data, Drv, Error, ErrorReg, ErrorRegMsk, Event, FifoConf, FifoDowns, GyrConf, GyrCrtConf,
    GyrOffsets, GyrRange, GyrSelfTest, IfConf, IntIoCtrl, IntLatch, IntMapData, IntMapFeat,
    InternalError, InternalStatus, InterruptStatus, MapData, NvConf, OutputBehavior, OutputLevel, 
    PullUpConf, PwrConf, PwrCtrl, Saturation, Status, WristGestureActivity, FIFO_LENGTH_1_MASK,
};

pub mod bmi270;
pub use bmi270::Bmi270;
