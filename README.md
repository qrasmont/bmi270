# Rust BMI260/270 driver

[![stability-wip](https://img.shields.io/badge/stability-wip-lightgrey.svg)](https://github.com/mkenney/software-guides/blob/master/STABILITY-BADGES.md#work-in-progress)

This is an [embedded-hal](https://github.com/rust-embedded/embedded-hal) driver for the Bosch BMI260/270 inertial measurement unit.

## Quick start

```rust
// ...

/// Create a new Bmi270 device using I2C with its alternative address (0x69).
/// Configure the max data burst to 255 bytes:
/// - used for the upload of the configuration during initialization.
/// - This is a limitation from your device or its HAL. 
let mut bmi = Bmi270::new_i2c(i2c, I2cAddr::Alternative, Burst::Other(255));

/// Get the chip id. Should be 0x24 or 36 in decimal
let chip_id = bmi.get_chip_id().unwrap();

/// Initialize the senor.
/// During this process a configuration of > 8kB is uploaded to the sensor.
bmi.init(&config::BMI270_CONFIG_FILE).unwrap();

/// Enable power for the accelerometer and the gyroscope.
let pwr_ctrl = PwrCtrl { aux_en: false, gyr_en: true, acc_en: true, temp_en: false };
bmi.set_pwr_ctrl(pwr_ctrl).unwrap();

/// Read the raw data
let data = bmi.get_data().unwrap();

// ...
```
