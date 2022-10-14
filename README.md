# Rust BMI270 driver

[![stability-wip](https://img.shields.io/badge/stability-wip-lightgrey.svg)](https://github.com/mkenney/software-guides/blob/master/STABILITY-BADGES.md#work-in-progress)

This is an [embedded-hal](https://github.com/rust-embedded/embedded-hal) driver for the Bosch BMI270 inertial measurement unit.

## Init

I sill need to figure out / learn how to get a platform agnostic way to use a delay.

In the mean time the init sequence has to be perform in 2 steps:
1. Disable advance power save and wait 450 us
2. Call init()

```rust
// ...

// Disable advanced power save
let mut pwr_conf = bmi.get_pwr_conf().unwrap();
pwr_conf.power_save = false;
bmi.set_pwr_conf(pwr_conf).unwrap();

// Wait for 450 us
timer.delay_us(450_u32);

// Init -> upload the 8kB config file.
bmi.init().unwrap();

// ...
```

With an agnostic delay, the goal is to just require `bmi.init().unwrap();`
