/// Root block of the Inner driver

#[derive(Debug)]
pub struct Inner<I> {
    pub(crate) interface: I,

    #[doc(hidden)]
    base_address: u8,
}

impl<I> Inner<I> {
    /// Create a new instance of the block based on device interface
    pub const fn new(interface: I) -> Self {
        Self {
            interface,
            base_address: 0,
        }
    }

    /// A reference to the interface used to communicate with the device
    pub(crate) fn interface(&mut self) -> &mut I {
        &mut self.interface
    }

    /// Read all readable register values in this block from the device.
    /// The callback is called for each of them.
    /// Any registers in child blocks are not included.
    ///
    /// The callback has three arguments:
    ///
    /// - The address of the register
    /// - The name of the register (with index for repeated registers)
    /// - The read value from the register
    ///
    /// This is useful for e.g. debug printing all values.
    /// The given [field_sets::FieldSetValue] has a Debug and Format implementation that forwards to the concrete type
    /// the lies within so it can be printed without matching on it.
    #[allow(unused_mut)]
    #[allow(unused_variables)]
    pub fn read_all_registers(
        &mut self,
        mut callback: impl FnMut(u8, &'static str, field_sets::FieldSetValue),
    ) -> Result<(), I::Error>
    where
        I: ::device_driver::RegisterInterface<AddressType = u8>,
    {
        let reg = self.temperature().read()?;

        callback(0 + 0 * 0, "temperature", reg.into());

        let reg = self.configuration().read()?;

        callback(1 + 0 * 0, "configuration", reg.into());

        let reg = self.t_low().read()?;

        callback(2 + 0 * 0, "t_low", reg.into());

        let reg = self.t_high().read()?;

        callback(3 + 0 * 0, "t_high", reg.into());

        Ok(())
    }

    /// Read all readable register values in this block from the device.
    /// The callback is called for each of them.
    /// Any registers in child blocks are not included.
    ///
    /// The callback has three arguments:
    ///
    /// - The address of the register
    /// - The name of the register (with index for repeated registers)
    /// - The read value from the register
    ///
    /// This is useful for e.g. debug printing all values.
    /// The given [field_sets::FieldSetValue] has a Debug and Format implementation that forwards to the concrete type
    /// the lies within so it can be printed without matching on it.
    #[allow(unused_mut)]
    #[allow(unused_variables)]
    pub async fn read_all_registers_async(
        &mut self,
        mut callback: impl FnMut(u8, &'static str, field_sets::FieldSetValue),
    ) -> Result<(), I::Error>
    where
        I: ::device_driver::AsyncRegisterInterface<AddressType = u8>,
    {
        let reg = self.temperature().read_async().await?;

        callback(0 + 0 * 0, "temperature", reg.into());

        let reg = self.configuration().read_async().await?;

        callback(1 + 0 * 0, "configuration", reg.into());

        let reg = self.t_low().read_async().await?;

        callback(2 + 0 * 0, "t_low", reg.into());

        let reg = self.t_high().read_async().await?;

        callback(3 + 0 * 0, "t_high", reg.into());

        Ok(())
    }

    /// Temperature register

    pub fn temperature(
        &mut self,
    ) -> ::device_driver::RegisterOperation<'_, I, u8, field_sets::Temperature, ::device_driver::RO> {
        let address = self.base_address + 0;

        ::device_driver::RegisterOperation::<'_, I, u8, field_sets::Temperature, ::device_driver::RO>::new(
            self.interface(),
            address as u8,
            field_sets::Temperature::new,
        )
    }

    /// Configuration register

    pub fn configuration(
        &mut self,
    ) -> ::device_driver::RegisterOperation<'_, I, u8, field_sets::Configuration, ::device_driver::RW> {
        let address = self.base_address + 1;

        ::device_driver::RegisterOperation::<'_, I, u8, field_sets::Configuration, ::device_driver::RW>::new(
            self.interface(),
            address as u8,
            field_sets::Configuration::new,
        )
    }

    /// Temperature low register

    pub fn t_low(&mut self) -> ::device_driver::RegisterOperation<'_, I, u8, field_sets::TLow, ::device_driver::RW> {
        let address = self.base_address + 2;

        ::device_driver::RegisterOperation::<'_, I, u8, field_sets::TLow, ::device_driver::RW>::new(
            self.interface(),
            address as u8,
            field_sets::TLow::new,
        )
    }

    /// Temperature high register

    pub fn t_high(&mut self) -> ::device_driver::RegisterOperation<'_, I, u8, field_sets::THigh, ::device_driver::RW> {
        let address = self.base_address + 3;

        ::device_driver::RegisterOperation::<'_, I, u8, field_sets::THigh, ::device_driver::RW>::new(
            self.interface(),
            address as u8,
            field_sets::THigh::new,
        )
    }
}

/// Module containing the generated fieldsets of the registers and commands
pub mod field_sets {
    #[allow(unused_imports)]
    use super::*;

    /// Temperature register

    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct Temperature {
        /// The internal bits
        bits: [u8; 2],
    }

    impl ::device_driver::FieldSet for Temperature {
        const SIZE_BITS: u32 = 16;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }

    impl Temperature {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0, 0] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 2] }
        }
    }

    impl From<[u8; 2]> for Temperature {
        fn from(bits: [u8; 2]) -> Self {
            Self { bits }
        }
    }

    impl From<Temperature> for [u8; 2] {
        fn from(val: Temperature) -> Self {
            val.bits
        }
    }

    impl core::fmt::Debug for Temperature {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("Temperature");

            d.finish()
        }
    }

    impl core::ops::BitAnd for Temperature {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }

    impl core::ops::BitAndAssign for Temperature {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }

    impl core::ops::BitOr for Temperature {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }

    impl core::ops::BitOrAssign for Temperature {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }

    impl core::ops::BitXor for Temperature {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }

    impl core::ops::BitXorAssign for Temperature {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }

    impl core::ops::Not for Temperature {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }

    /// Configuration register

    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct Configuration {
        /// The internal bits
        bits: [u8; 2],
    }

    impl ::device_driver::FieldSet for Configuration {
        const SIZE_BITS: u32 = 16;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }

    impl Configuration {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [34, 16] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 2] }
        }

        ///Read the `m` field of the register.
        ///
        /// Device functional mode

        pub fn m(&self) -> Result<super::Mode, <super::Mode as TryFrom<u8>>::Error> {
            let raw = unsafe { ::device_driver::ops::load_lsb0::<u8, ::device_driver::ops::LE>(&self.bits, 0, 2) };

            raw.try_into()
        }

        ///Read the `tm` field of the register.
        ///
        /// Thermostat mode

        pub fn tm(&self) -> super::Thermostat {
            let raw = unsafe { ::device_driver::ops::load_lsb0::<u8, ::device_driver::ops::LE>(&self.bits, 2, 3) };

            unsafe { raw.try_into().unwrap_unchecked() }
        }

        ///Read the `fl` field of the register.
        ///
        /// Temperature watchdog low flag

        pub fn fl(&self) -> bool {
            let raw = unsafe { ::device_driver::ops::load_lsb0::<u8, ::device_driver::ops::LE>(&self.bits, 3, 4) };

            raw > 0
        }

        ///Read the `fh` field of the register.
        ///
        /// Temperature watchdog high flag

        pub fn fh(&self) -> bool {
            let raw = unsafe { ::device_driver::ops::load_lsb0::<u8, ::device_driver::ops::LE>(&self.bits, 4, 5) };

            raw > 0
        }

        ///Read the `cr` field of the register.
        ///
        /// Conversion rate

        pub fn cr(&self) -> super::ConversionRate {
            let raw = unsafe { ::device_driver::ops::load_lsb0::<u8, ::device_driver::ops::LE>(&self.bits, 5, 7) };

            unsafe { raw.try_into().unwrap_unchecked() }
        }

        ///Read the `id` field of the register.
        ///
        /// ID

        pub fn id(&self) -> bool {
            let raw = unsafe { ::device_driver::ops::load_lsb0::<u8, ::device_driver::ops::LE>(&self.bits, 7, 8) };

            raw > 0
        }

        ///Read the `hys` field of the register.
        ///
        /// Hysteresis control

        pub fn hys(&self) -> super::Hysteresis {
            let raw = unsafe { ::device_driver::ops::load_lsb0::<u8, ::device_driver::ops::LE>(&self.bits, 12, 14) };

            unsafe { raw.try_into().unwrap_unchecked() }
        }

        ///Read the `pol` field of the register.
        ///
        /// ALERT pin polarity

        pub fn pol(&self) -> super::Polarity {
            let raw = unsafe { ::device_driver::ops::load_lsb0::<u8, ::device_driver::ops::LE>(&self.bits, 15, 16) };

            unsafe { raw.try_into().unwrap_unchecked() }
        }

        ///Write the `m` field of the register.
        ///
        /// Device functional mode

        pub fn set_m(&mut self, value: super::Mode) {
            let raw = value.into();

            unsafe { ::device_driver::ops::store_lsb0::<u8, ::device_driver::ops::LE>(raw, 0, 2, &mut self.bits) };
        }

        ///Write the `tm` field of the register.
        ///
        /// Thermostat mode

        pub fn set_tm(&mut self, value: super::Thermostat) {
            let raw = value.into();

            unsafe { ::device_driver::ops::store_lsb0::<u8, ::device_driver::ops::LE>(raw, 2, 3, &mut self.bits) };
        }

        ///Write the `fl` field of the register.
        ///
        /// Temperature watchdog low flag

        pub fn set_fl(&mut self, value: bool) {
            let raw = value as _;

            unsafe { ::device_driver::ops::store_lsb0::<u8, ::device_driver::ops::LE>(raw, 3, 4, &mut self.bits) };
        }

        ///Write the `fh` field of the register.
        ///
        /// Temperature watchdog high flag

        pub fn set_fh(&mut self, value: bool) {
            let raw = value as _;

            unsafe { ::device_driver::ops::store_lsb0::<u8, ::device_driver::ops::LE>(raw, 4, 5, &mut self.bits) };
        }

        ///Write the `cr` field of the register.
        ///
        /// Conversion rate

        pub fn set_cr(&mut self, value: super::ConversionRate) {
            let raw = value.into();

            unsafe { ::device_driver::ops::store_lsb0::<u8, ::device_driver::ops::LE>(raw, 5, 7, &mut self.bits) };
        }

        ///Write the `id` field of the register.
        ///
        /// ID

        pub fn set_id(&mut self, value: bool) {
            let raw = value as _;

            unsafe { ::device_driver::ops::store_lsb0::<u8, ::device_driver::ops::LE>(raw, 7, 8, &mut self.bits) };
        }

        ///Write the `hys` field of the register.
        ///
        /// Hysteresis control

        pub fn set_hys(&mut self, value: super::Hysteresis) {
            let raw = value.into();

            unsafe { ::device_driver::ops::store_lsb0::<u8, ::device_driver::ops::LE>(raw, 12, 14, &mut self.bits) };
        }

        ///Write the `pol` field of the register.
        ///
        /// ALERT pin polarity

        pub fn set_pol(&mut self, value: super::Polarity) {
            let raw = value.into();

            unsafe { ::device_driver::ops::store_lsb0::<u8, ::device_driver::ops::LE>(raw, 15, 16, &mut self.bits) };
        }
    }

    impl From<[u8; 2]> for Configuration {
        fn from(bits: [u8; 2]) -> Self {
            Self { bits }
        }
    }

    impl From<Configuration> for [u8; 2] {
        fn from(val: Configuration) -> Self {
            val.bits
        }
    }

    impl core::fmt::Debug for Configuration {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("Configuration");

            d.field("m", &self.m());

            d.field("tm", &self.tm());

            d.field("fl", &self.fl());

            d.field("fh", &self.fh());

            d.field("cr", &self.cr());

            d.field("id", &self.id());

            d.field("hys", &self.hys());

            d.field("pol", &self.pol());

            d.finish()
        }
    }

    impl core::ops::BitAnd for Configuration {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }

    impl core::ops::BitAndAssign for Configuration {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }

    impl core::ops::BitOr for Configuration {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }

    impl core::ops::BitOrAssign for Configuration {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }

    impl core::ops::BitXor for Configuration {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }

    impl core::ops::BitXorAssign for Configuration {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }

    impl core::ops::Not for Configuration {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }

    /// Temperature low register

    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct TLow {
        /// The internal bits
        bits: [u8; 2],
    }

    impl ::device_driver::FieldSet for TLow {
        const SIZE_BITS: u32 = 16;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }

    impl TLow {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0, 0] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 2] }
        }
    }

    impl From<[u8; 2]> for TLow {
        fn from(bits: [u8; 2]) -> Self {
            Self { bits }
        }
    }

    impl From<TLow> for [u8; 2] {
        fn from(val: TLow) -> Self {
            val.bits
        }
    }

    impl core::fmt::Debug for TLow {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("TLow");

            d.finish()
        }
    }

    impl core::ops::BitAnd for TLow {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }

    impl core::ops::BitAndAssign for TLow {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }

    impl core::ops::BitOr for TLow {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }

    impl core::ops::BitOrAssign for TLow {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }

    impl core::ops::BitXor for TLow {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }

    impl core::ops::BitXorAssign for TLow {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }

    impl core::ops::Not for TLow {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }

    /// Temperature high register

    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct THigh {
        /// The internal bits
        bits: [u8; 2],
    }

    impl ::device_driver::FieldSet for THigh {
        const SIZE_BITS: u32 = 16;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }

    impl THigh {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0, 0] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 2] }
        }
    }

    impl From<[u8; 2]> for THigh {
        fn from(bits: [u8; 2]) -> Self {
            Self { bits }
        }
    }

    impl From<THigh> for [u8; 2] {
        fn from(val: THigh) -> Self {
            val.bits
        }
    }

    impl core::fmt::Debug for THigh {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("THigh");

            d.finish()
        }
    }

    impl core::ops::BitAnd for THigh {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }

    impl core::ops::BitAndAssign for THigh {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }

    impl core::ops::BitOr for THigh {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }

    impl core::ops::BitOrAssign for THigh {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }

    impl core::ops::BitXor for THigh {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }

    impl core::ops::BitXorAssign for THigh {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }

    impl core::ops::Not for THigh {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }

    /// Enum containing all possible field set types
    pub enum FieldSetValue {
        /// Temperature register
        Temperature(Temperature),

        /// Configuration register
        Configuration(Configuration),

        /// Temperature low register
        TLow(TLow),

        /// Temperature high register
        THigh(THigh),
    }
    impl core::fmt::Debug for FieldSetValue {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            match self {
                Self::Temperature(val) => core::fmt::Debug::fmt(val, f),

                Self::Configuration(val) => core::fmt::Debug::fmt(val, f),

                Self::TLow(val) => core::fmt::Debug::fmt(val, f),

                Self::THigh(val) => core::fmt::Debug::fmt(val, f),

                #[allow(unreachable_patterns)]
                _ => unreachable!(),
            }
        }
    }

    impl From<Temperature> for FieldSetValue {
        fn from(val: Temperature) -> Self {
            Self::Temperature(val)
        }
    }

    impl From<Configuration> for FieldSetValue {
        fn from(val: Configuration) -> Self {
            Self::Configuration(val)
        }
    }

    impl From<TLow> for FieldSetValue {
        fn from(val: TLow) -> Self {
            Self::TLow(val)
        }
    }

    impl From<THigh> for FieldSetValue {
        fn from(val: THigh) -> Self {
            Self::THigh(val)
        }
    }
}

/// Device functional mode

#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]

pub enum Mode {
    Shutdown = 0,

    OneShot = 1,

    Continuous = 2,
}

impl core::convert::TryFrom<u8> for Mode {
    type Error = ::device_driver::ConversionError<u8>;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::Shutdown),

            1 => Ok(Self::OneShot),

            2 => Ok(Self::Continuous),

            val => Err(::device_driver::ConversionError {
                source: val,
                target: "Mode",
            }),
        }
    }
}

impl From<Mode> for u8 {
    fn from(val: Mode) -> Self {
        match val {
            Mode::Shutdown => 0,

            Mode::OneShot => 1,

            Mode::Continuous => 2,
        }
    }
}

/// Thermostat mode

#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]

pub enum Thermostat {
    Comparator = 0,

    Interrupt = 1,
}

impl core::convert::TryFrom<u8> for Thermostat {
    type Error = ::device_driver::ConversionError<u8>;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::Comparator),

            1 => Ok(Self::Interrupt),

            val => Err(::device_driver::ConversionError {
                source: val,
                target: "Thermostat",
            }),
        }
    }
}

impl From<Thermostat> for u8 {
    fn from(val: Thermostat) -> Self {
        match val {
            Thermostat::Comparator => 0,

            Thermostat::Interrupt => 1,
        }
    }
}

/// Conversion rate

#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]

pub enum ConversionRate {
    _0_25Hz = 0,

    _1Hz = 1,

    _4Hz = 2,

    _16Hz = 3,
}

impl core::convert::TryFrom<u8> for ConversionRate {
    type Error = ::device_driver::ConversionError<u8>;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::_0_25Hz),

            1 => Ok(Self::_1Hz),

            2 => Ok(Self::_4Hz),

            3 => Ok(Self::_16Hz),

            val => Err(::device_driver::ConversionError {
                source: val,
                target: "ConversionRate",
            }),
        }
    }
}

impl From<ConversionRate> for u8 {
    fn from(val: ConversionRate) -> Self {
        match val {
            ConversionRate::_0_25Hz => 0,

            ConversionRate::_1Hz => 1,

            ConversionRate::_4Hz => 2,

            ConversionRate::_16Hz => 3,
        }
    }
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]

pub enum Hysteresis {
    _0C = 0,

    _1C = 1,

    _2C = 2,

    _4C = 3,
}

impl core::convert::TryFrom<u8> for Hysteresis {
    type Error = ::device_driver::ConversionError<u8>;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::_0C),

            1 => Ok(Self::_1C),

            2 => Ok(Self::_2C),

            3 => Ok(Self::_4C),

            val => Err(::device_driver::ConversionError {
                source: val,
                target: "Hysteresis",
            }),
        }
    }
}

impl From<Hysteresis> for u8 {
    fn from(val: Hysteresis) -> Self {
        match val {
            Hysteresis::_0C => 0,

            Hysteresis::_1C => 1,

            Hysteresis::_2C => 2,

            Hysteresis::_4C => 3,
        }
    }
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]

pub enum Polarity {
    ActiveLow = 0,

    ActiveHigh = 1,
}

impl core::convert::TryFrom<u8> for Polarity {
    type Error = ::device_driver::ConversionError<u8>;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::ActiveLow),

            1 => Ok(Self::ActiveHigh),

            val => Err(::device_driver::ConversionError {
                source: val,
                target: "Polarity",
            }),
        }
    }
}

impl From<Polarity> for u8 {
    fn from(val: Polarity) -> Self {
        match val {
            Polarity::ActiveLow => 0,

            Polarity::ActiveHigh => 1,
        }
    }
}
