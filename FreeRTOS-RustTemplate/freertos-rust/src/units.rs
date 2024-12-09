use core::marker::PhantomData;
use crate::base::FreeRtosTickType;
use crate::prelude::v1::*;
use crate::shim::*;

// FreeRTOS Time Units interface
pub trait FreeRtosTimeUnits {
    fn get_tick_period_ms() -> u32;
    fn get_max_wait() -> u32;
}

// Shim za FreeRTOS vreme
#[derive(Copy, Clone, Default)]
pub struct FreeRtosTimeUnitsShimmed;
impl FreeRtosTimeUnits for FreeRtosTimeUnitsShimmed {
    #[inline]
    fn get_tick_period_ms() -> u32 {
        unsafe { freertos_rs_get_portTICK_PERIOD_MS() }
    }
    #[inline]
    fn get_max_wait() -> u32 {
        unsafe { freertos_rs_max_wait() }
    }
}

// Trait koji omogućava konverziju u ticks
pub trait DurationTicks: Copy + Clone {
    fn to_ticks(&self) -> FreeRtosTickType;
}

// Naš tip za Duration
pub type Duration = DurationImpl<FreeRtosTimeUnitsShimmed>;

// FreeRTOS vremenska jedinica, koristi se u scheduleru
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct DurationImpl<T> {
    ticks: u32,
    _time_units: PhantomData<T>,
}

// Implementacija za DurationImpl sa različitim metodama
impl<T> DurationImpl<T>
where
    T: FreeRtosTimeUnits + Copy,
{
    // Konstruktor koji prima milisekunde i računa ticks na osnovu toga
    pub fn ms(milliseconds: u32) -> Self {
        Self::ticks(milliseconds / T::get_tick_period_ms())
    }

    // Konstruktor koji direktno koristi ticks
    pub fn ticks(ticks: u32) -> Self {
        DurationImpl {
            ticks,
            _time_units: PhantomData,
        }
    }

    // Metod koji vraća trajanje kao `DurationImpl<T>`
    pub fn new(ticks: u32) -> Self {
        DurationImpl {
            ticks,
            _time_units: PhantomData,
        }
    }

    // Vraća beskonačno trajanje
    pub fn infinite() -> Self {
        Self::ticks(T::get_max_wait())
    }

    // Vraća trajanje nula (za neblokirajuće pozive)
    pub fn zero() -> Self {
        Self::ticks(0)
    }

    // Vraća najmanju moguću jedinicu, jedan tick
    pub fn eps() -> Self {
        Self::ticks(1)
    }

    // Vraća trajanje u milisekundama
    pub fn to_ms(&self) -> u32 {
        self.ticks * T::get_tick_period_ms()
    }
}

// Implementacija za trait `DurationTicks`
impl<T> DurationTicks for DurationImpl<T>
where
    T: FreeRtosTimeUnits + Copy,
{
    fn to_ticks(&self) -> FreeRtosTickType {
        self.ticks
    }
}
