#![no_std]

#[macro_use]
extern crate fixedvec;

mod registers;
pub mod config;

pub mod interface;
pub use interface::I2cAddr;
pub mod types;

pub mod bmi270;
pub use bmi270::Bmi270;
