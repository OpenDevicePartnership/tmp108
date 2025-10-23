# TMP108

[![no-std](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/nostd.yml/badge.svg)](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/nostd.yml)
[![check](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/check.yml/badge.svg)](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/check.yml)
[![rolling](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/rolling.yml/badge.svg)](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/rolling.yml)
[![crates.io](https://img.shields.io/crates/v/tmp108.svg)](https://crates.io/crates/tmp108)
[![Documentation](https://docs.rs/tmp108/badge.svg)](https://docs.rs/tmp108)
[![LICENSE](https://img.shields.io/badge/License-MIT-blue)](./LICENSE)

A `#[no_std]` platform-agnostic driver for the
[TMP108](https://www.ti.com/lit/gpn/tmp108) temperature sensor using
the [embedded-hal](https://docs.rs/embedded-hal) traits.

The TMP108 can take one of 4 I2C addresses depending on the state of
the A0 pin, as described in the table below:

| A0  | Addr |
|-----|------|
| GND | 0x48 |
| V+  | 0x49 |
| SDA | 0x4a |
| SCL | 0x4b |

The driver has specific constructors for each of these states, to
ensure that an invalid address is not attempted.

## Usage

```rust,ignore
let delay = DelayNs;
let mut tmp = Tmp108::new_with_a0_gnd(i2c, delay);
// let mut tmp = Tmp108::new_with_a0_vplus(i2c, delay);
// let mut tmp = Tmp108::new_with_a0_sda(i2c, delay);
// let mut tmp = Tmp108::new_with_a0_scl(i2c, delay);
// let mut tmp = Tmp108::new(i2c, delay, A0::Gnd);

let cfg = Default::default()
    .with_cm(ConversionMode::OneShot)
    .with_tm(ThermostatMode::Comparator)
    .with_cr(ConversionRate::Hertz16)
    .with_hysteresis(Hysteresis::FourCelsius)
    .with_polarity(Polarity::ActiveLow);

tmp.set_configuration(cfg)?;

let temp = tmp.temperature()?;

let cfg = cfg
    .with_cm(ConversionMode::OneShot)
    .with_tm(ThermostatMode::Interrupt)
    .with_cr(ConversionRate::Hertz1)
    .with_fl(true)
    .with_fh(true);

tmp.set_configuration(cfg)?;

let high_limit = 48.0;
let low_limit = 26.5;

tmp.set_low_limit(low_limit)?;
tmp.set_high_limit(high_limit)?;

tmp.continuous(Default::default(), |t| {
	for _ in 0..10 {
		let temp = tmp.wait_for_temperature()?;
		info!("Temperature {}", temp);
	}

	Ok(())
})?;

```

## MSRV

Currently, rust `1.85` and up is supported, but some previous versions
may work.

## License

Licensed under the terms of the [MIT license](http://opensource.org/licenses/MIT).

## Contribution

Unless you explicitly state otherwise, any contribution submitted for
inclusion in the work by you shall be licensed under the terms of the
MIT license.

License: MIT

