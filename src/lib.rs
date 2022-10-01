#![no_std]

pub mod interface;
mod types;

pub struct Bmi270<I> {
    iface: I,
}
