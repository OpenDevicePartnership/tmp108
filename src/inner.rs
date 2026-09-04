// This code was generated using device-driver `2.1.0` (),
// a tool distributed under MIT OR Apache-2.0 by Dion Dokter <dev@diondokter.nl>
//
// For more information about device-driver, visit the website: https://device-driver.com

/// Root block of the Inner driver
#[derive(Debug)]
pub struct Inner<I> {
    interface: I,
    #[doc(hidden)]
    #[allow(unused)]
    base_address: u8,
}
impl<I> Inner<I> {
    /// Create a new instance of the device
    pub const fn new(interface: I) -> Self {
        Self {
            interface,
            base_address: 0,
        }
    }
    /// Drop the driver instance and reclaim the interface
    pub fn free(self) -> I {
        self.interface
    }
    /// Temperature register
    ///
    /// Register operation:
    /// - Address: `0`
    /// - Reset value: `0`
    pub fn temperature(
        &mut self,
    ) -> ::device_driver::RegisterOperation<'_, Self, Temperature, u8, ::device_driver::RO, ()>
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 0;
        ::device_driver::RegisterOperation::new(self, address as u8, Temperature::default)
    }
    /// Configuration register
    ///
    /// Register operation:
    /// - Address: `1`
    /// - Reset value: `0x1022`
    pub fn configuration(
        &mut self,
    ) -> ::device_driver::RegisterOperation<'_, Self, Configuration, u8, ::device_driver::RW, ()>
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 1;
        ::device_driver::RegisterOperation::new(self, address as u8, || Configuration::from([34, 16]))
    }
    /// Temperature low register
    ///
    /// Register operation:
    /// - Address: `2`
    /// - Reset value: `0`
    #[doc(alias = "t-low")]
    pub fn t_low(&mut self) -> ::device_driver::RegisterOperation<'_, Self, TLow, u8, ::device_driver::RW, ()>
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 2;
        ::device_driver::RegisterOperation::new(self, address as u8, TLow::default)
    }
    /// Temperature high register
    ///
    /// Register operation:
    /// - Address: `3`
    /// - Reset value: `0`
    #[doc(alias = "t-high")]
    pub fn t_high(&mut self) -> ::device_driver::RegisterOperation<'_, Self, THigh, u8, ::device_driver::RW, ()>
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 3;
        ::device_driver::RegisterOperation::new(self, address as u8, THigh::default)
    }
}
impl<I> ::device_driver::Block for Inner<I> {
    type Interface = I;
    type RegisterAddressType = u8;
    type CommandAddressType = u8;
    type BufferAddressType = u8;
    type RegisterAddressMode = ();
    fn interface(&mut self) -> &mut Self::Interface {
        &mut self.interface
    }
}
#[doc(alias = "t-high")]
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct THigh {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 2],
}
unsafe impl ::device_driver::Fieldset for THigh {
    const METADATA: ::device_driver::FieldsetMetadata =
        ::device_driver::FieldsetMetadata::new().with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 2] };
}
impl THigh {}
impl Default for THigh {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[doc(alias = "t-low")]
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct TLow {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 2],
}
unsafe impl ::device_driver::Fieldset for TLow {
    const METADATA: ::device_driver::FieldsetMetadata =
        ::device_driver::FieldsetMetadata::new().with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 2] };
}
impl TLow {}
impl Default for TLow {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[doc(alias = "configuration")]
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct Configuration {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 2],
}
unsafe impl ::device_driver::Fieldset for Configuration {
    const METADATA: ::device_driver::FieldsetMetadata =
        ::device_driver::FieldsetMetadata::new().with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 2] };
}
impl Configuration {
    /// `1:0` - Read the `m` field.
    ///
    /// Device functional mode
    #[must_use]
    pub fn m(&self) -> Result<Mode, <Mode as TryFrom<u8>>::Error> {
        let start = 0;
        let end = 1;
        let raw = unsafe { ::device_driver::ops::load::<u8, ::device_driver::ops::LE>(&self.bits, start, end) };
        raw.try_into()
    }
    /// `bit 2` - Read the `tm` field.
    ///
    /// Thermostat mode
    #[must_use]
    pub fn tm(&self) -> Thermostat {
        let start = 2;
        let end = 2;
        let raw = unsafe { ::device_driver::ops::load::<u8, ::device_driver::ops::LE>(&self.bits, start, end) };
        unsafe { raw.try_into().unwrap_unchecked() }
    }
    /// `bit 3` - Read the `fl` field.
    ///
    /// Temperature watchdog low flag
    #[must_use]
    pub fn fl(&self) -> bool {
        let start = 3;
        let end = 3;
        let raw = unsafe { ::device_driver::ops::load::<u8, ::device_driver::ops::LE>(&self.bits, start, end) };
        raw > 0
    }
    /// `bit 4` - Read the `fh` field.
    ///
    /// Temperature watchdog high flag
    #[must_use]
    pub fn fh(&self) -> bool {
        let start = 4;
        let end = 4;
        let raw = unsafe { ::device_driver::ops::load::<u8, ::device_driver::ops::LE>(&self.bits, start, end) };
        raw > 0
    }
    /// `6:5` - Read the `cr` field.
    ///
    /// Conversion rate
    #[must_use]
    pub fn cr(&self) -> ConversionRate {
        let start = 5;
        let end = 6;
        let raw = unsafe { ::device_driver::ops::load::<u8, ::device_driver::ops::LE>(&self.bits, start, end) };
        unsafe { raw.try_into().unwrap_unchecked() }
    }
    /// `bit 7` - Read the `id` field.
    ///
    /// ID
    #[must_use]
    pub fn id(&self) -> bool {
        let start = 7;
        let end = 7;
        let raw = unsafe { ::device_driver::ops::load::<u8, ::device_driver::ops::LE>(&self.bits, start, end) };
        raw > 0
    }
    /// `13:12` - Read the `hys` field.
    ///
    /// Hysteresis control
    #[must_use]
    pub fn hys(&self) -> Hysteresis {
        let start = 12;
        let end = 13;
        let raw = unsafe { ::device_driver::ops::load::<u8, ::device_driver::ops::LE>(&self.bits, start, end) };
        unsafe { raw.try_into().unwrap_unchecked() }
    }
    /// `bit 15` - Read the `pol` field.
    ///
    /// ALERT pin polarity
    #[must_use]
    pub fn pol(&self) -> Polarity {
        let start = 15;
        let end = 15;
        let raw = unsafe { ::device_driver::ops::load::<u8, ::device_driver::ops::LE>(&self.bits, start, end) };
        unsafe { raw.try_into().unwrap_unchecked() }
    }
    /// `1:0` - Set the `m` field.
    ///
    /// Device functional mode
    pub fn set_m(&mut self, value: Mode) {
        let start = 0;
        let end = 1;
        let raw = value.into();
        unsafe { ::device_driver::ops::store::<u8, ::device_driver::ops::LE>(raw, start, end, &mut self.bits) };
    }
    /// `bit 2` - Set the `tm` field.
    ///
    /// Thermostat mode
    pub fn set_tm(&mut self, value: Thermostat) {
        let start = 2;
        let end = 2;
        let raw = value.into();
        unsafe { ::device_driver::ops::store::<u8, ::device_driver::ops::LE>(raw, start, end, &mut self.bits) };
    }
    /// `bit 3` - Set the `fl` field.
    ///
    /// Temperature watchdog low flag
    pub fn set_fl(&mut self, value: bool) {
        let start = 3;
        let end = 3;
        let raw = value as _;
        unsafe { ::device_driver::ops::store::<u8, ::device_driver::ops::LE>(raw, start, end, &mut self.bits) };
    }
    /// `bit 4` - Set the `fh` field.
    ///
    /// Temperature watchdog high flag
    pub fn set_fh(&mut self, value: bool) {
        let start = 4;
        let end = 4;
        let raw = value as _;
        unsafe { ::device_driver::ops::store::<u8, ::device_driver::ops::LE>(raw, start, end, &mut self.bits) };
    }
    /// `6:5` - Set the `cr` field.
    ///
    /// Conversion rate
    pub fn set_cr(&mut self, value: ConversionRate) {
        let start = 5;
        let end = 6;
        let raw = value.into();
        unsafe { ::device_driver::ops::store::<u8, ::device_driver::ops::LE>(raw, start, end, &mut self.bits) };
    }
    /// `bit 7` - Set the `id` field.
    ///
    /// ID
    pub fn set_id(&mut self, value: bool) {
        let start = 7;
        let end = 7;
        let raw = value as _;
        unsafe { ::device_driver::ops::store::<u8, ::device_driver::ops::LE>(raw, start, end, &mut self.bits) };
    }
    /// `13:12` - Set the `hys` field.
    ///
    /// Hysteresis control
    pub fn set_hys(&mut self, value: Hysteresis) {
        let start = 12;
        let end = 13;
        let raw = value.into();
        unsafe { ::device_driver::ops::store::<u8, ::device_driver::ops::LE>(raw, start, end, &mut self.bits) };
    }
    /// `bit 15` - Set the `pol` field.
    ///
    /// ALERT pin polarity
    pub fn set_pol(&mut self, value: Polarity) {
        let start = 15;
        let end = 15;
        let raw = value.into();
        unsafe { ::device_driver::ops::store::<u8, ::device_driver::ops::LE>(raw, start, end, &mut self.bits) };
    }
}
impl Default for Configuration {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[doc(alias = "temperature")]
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct Temperature {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 2],
}
unsafe impl ::device_driver::Fieldset for Temperature {
    const METADATA: ::device_driver::FieldsetMetadata =
        ::device_driver::FieldsetMetadata::new().with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 2] };
}
impl Temperature {}
impl Default for Temperature {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
/// ALERT pin polarity
#[doc(alias = "polarity")]
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum Polarity {
    #[doc(alias = "active-low")]
    ActiveLow = 0,
    #[doc(alias = "active-high")]
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
#[doc(hidden)]
impl ::device_driver::EnumIndex for Polarity {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
    }
}
/// Temperature hysteresis
#[doc(alias = "hysteresis")]
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum Hysteresis {
    #[doc(alias = "zero-c")]
    ZeroC = 0,
    #[doc(alias = "one-c")]
    OneC = 1,
    #[doc(alias = "two-c")]
    TwoC = 2,
    #[doc(alias = "four-c")]
    FourC = 3,
}
impl core::convert::TryFrom<u8> for Hysteresis {
    type Error = ::device_driver::ConversionError<u8>;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::ZeroC),
            1 => Ok(Self::OneC),
            2 => Ok(Self::TwoC),
            3 => Ok(Self::FourC),
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
            Hysteresis::ZeroC => 0,
            Hysteresis::OneC => 1,
            Hysteresis::TwoC => 2,
            Hysteresis::FourC => 3,
        }
    }
}
#[doc(hidden)]
impl ::device_driver::EnumIndex for Hysteresis {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
    }
}
/// Conversion rate
#[doc(alias = "conversion-rate")]
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum ConversionRate {
    #[doc(alias = "quarter-hz")]
    QuarterHz = 0,
    #[doc(alias = "one-hz")]
    OneHz = 1,
    #[doc(alias = "four-hz")]
    FourHz = 2,
    #[doc(alias = "sixteen-hz")]
    SixteenHz = 3,
}
impl core::convert::TryFrom<u8> for ConversionRate {
    type Error = ::device_driver::ConversionError<u8>;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::QuarterHz),
            1 => Ok(Self::OneHz),
            2 => Ok(Self::FourHz),
            3 => Ok(Self::SixteenHz),
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
            ConversionRate::QuarterHz => 0,
            ConversionRate::OneHz => 1,
            ConversionRate::FourHz => 2,
            ConversionRate::SixteenHz => 3,
        }
    }
}
#[doc(hidden)]
impl ::device_driver::EnumIndex for ConversionRate {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
    }
}
/// Thermostat mode
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum Thermostat {
    #[doc(alias = "comparator")]
    Comparator = 0,
    #[doc(alias = "interrupt")]
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
#[doc(hidden)]
impl ::device_driver::EnumIndex for Thermostat {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
    }
}
/// Device functional mode
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum Mode {
    #[doc(alias = "shutdown")]
    Shutdown = 0,
    #[doc(alias = "one-shot")]
    OneShot = 1,
    #[doc(alias = "continuous")]
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
#[doc(hidden)]
impl ::device_driver::EnumIndex for Mode {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
    }
}
