#![no_std]
#![no_main]

#![feature(alloc_error_handler)]

extern crate alloc;
// Imports
use core::cell::{Cell, RefCell};
use cortex_m::interrupt::Mutex;
use cortex_m::Peripherals;
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{
    gpio::{self, Edge, Floating, Input, PullUp},
    prelude::*,
};
use freertos_rust::*;
use cortex_m::asm;
use cortex_m_rt::exception;
use cortex_m_rt::ExceptionFrame;
use core::alloc::Layout;
use stm32f4xx_hal::gpio::gpioa::PA5;
use stm32f4xx_hal::gpio::gpioc::PC13;
use stm32f4xx_hal::stm32;
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use stm32f4xx_hal::gpio::{GpioExt, Output, PushPull};
use stm32f4xx_hal::gpio::ExtiPin;
use crate::stm32::interrupt;
use stm32f4xx_hal::delay::Delay;
use stm32f4::stm32f401::NVIC;
use stm32f4xx_hal::rcc::Clocks;
use stm32f4xx_hal::time;


// Create an alias for pin PC13
type ButtonPin = PC13<Input<PullUp>>;

#[global_allocator]
static GLOBAL: freertos_rust::FreeRtosAllocator = freertos_rust::FreeRtosAllocator;

// Global Variable Definitions
static G_BUTTON: Mutex<RefCell<Option<ButtonPin>>> = Mutex::new(RefCell::new(None));
static G_DELAYMS: Mutex<Cell<u32>> = Mutex::new(Cell::new(2000_u32));


#[entry]
fn main() -> ! {
    // Setup handler for device peripherals
    let dp = stm32::Peripherals::take().unwrap(); // Uzimaš vlasništvo nad svim periferijama mikrokontrolera

    // Direktan pristup dp.RCC (u ovom slučaju bez korišćenja constrain)
    dp.RCC.cr.modify(|_, w| w.hseon().set_bit()); // Uključujemo HSE oscilator
    //while dp.RCC.cr.read().hserdy().bit_is_clear() {} // Čekamo da HSE bude spreman

    // Postavljamo HSE kao sistemski sat
    dp.RCC.cfgr.modify(|_, w| w.sw().hse());

    // Omogućavamo takte za GPIO portove A i C
    dp.RCC.ahb1enr.modify(|_, w| w.gpioaen().enabled().gpiocen().enabled());

    // Omogućavanje takta za SYSCFG (sistemsku konfiguraciju)
    dp.RCC.apb2enr.modify(|_, w| w.syscfgen().enabled());

    // Postavljamo GPIO portove
    let gpioa = dp.GPIOA.split(); // Delimo GPIO port A
    let mut led = gpioa.pa5.into_push_pull_output(); // Postavljamo pin PA5 kao izlaz za LED

    let gpioc = dp.GPIOC.split(); // Delimo GPIO port C
    let button: PC13<Input<PullUp>> = gpioc.pc13.into_pull_up_input(); // Postavljamo pin PC13 kao ulaz sa pull-up otpornikom

    // Podešavanje prekida za dugme na PC13
    dp.SYSCFG.exticr4.modify(|_, w| unsafe { w.exti13().bits(0x2) }); // Postavljamo EXTI13 na port C
    dp.EXTI.ftsr.modify(|_, w| w.tr13().set_bit()); // Konfigurišemo Falling edge trigger za EXTI13
    dp.EXTI.rtsr.modify(|_, w| w.tr13().clear_bit()); // Brišemo Rising edge trigger za EXTI13
    dp.EXTI.imr.modify(|_, w| w.mr13().set_bit()); // Omogućavamo EXTI13 interrupt

    // Unmask EXTI13 interrupt u NVIC
    unsafe {
        NVIC::unmask(interrupt::EXTI15_10);
    }

    // Inicijalizacija delay objekta koristeći cortex_m::Peripherals za pristup SYST
    let cp = cortex_m::Peripherals::take().unwrap(); // Preuzimanje perifernog pristupa iz Cortex-M
    let clocks = dp.RCC.constrain().cfgr.freeze(); // Dobijamo sve satove
    let mut delay = Delay::new(cp.SYST, clocks); // Ispravka: koristi se cp.SYST za SysTick

    // Čuvamo referencu na dugme u globalnoj promenljivoj
    cortex_m::interrupt::free(|cs| {
        G_BUTTON.borrow(cs).replace(Some(button));
    });

    loop {
        led.set_high(); // Turn on LED
        delay.delay_ms(cortex_m::interrupt::free(|cs| G_DELAYMS.borrow(cs).get())); // Delay for delay_ms
        led.set_low(); // Turn off LED
        delay.delay_ms(cortex_m::interrupt::free(|cs| G_DELAYMS.borrow(cs).get())); // Delay for delay_ms
    }
}


// Prekid handler za EXTI15_10 (prekid sa dugmadi)
#[interrupt]
fn EXTI15_10() {
    cortex_m::interrupt::free(|cs| {
        let current_delay = G_DELAYMS.borrow(cs).get(); // Uzimamo trenutni delay

        // Ako je delay veći od 500 ms, smanjujemo ga za 500 ms
        if current_delay > 500 {
            G_DELAYMS.borrow(cs).set(current_delay - 500_u32);
        } else {
            // Ako je delay manji od 500 ms, vraćamo ga na početnu vrednost (2000 ms)
            G_DELAYMS.borrow(cs).set(2000_u32);
        }

        // Čistimo pending interrupt bit za EXTI13
        let mut button = G_BUTTON.borrow(cs).borrow_mut();
        button.as_mut().unwrap().clear_interrupt_pending_bit();
    });
}

// Default handler za nepoznate ili neimplementirane prekide
#[exception]
fn DefaultHandler(_irqn: i16) {
    // Custom default handler
}

// Handler za HardFault, bez kraja funkcije (beskonačna petlja)
#[exception]
fn HardFault(_ef: &ExceptionFrame) -> ! {
    loop {}
}

// Handler za greške u alokaciji memorije (ako nije moguće alocirati memoriju)
#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    asm::bkpt(); // Pauzira program
    loop {}
}

// Hook funkcija koja se poziva kada se dogodi stack overflow u FreeRTOS-u
#[no_mangle]
fn vApplicationStackOverflowHook(_pxTask: FreeRtosTaskHandle, _pcTaskName: FreeRtosCharPtr) {
    asm::bkpt(); // Pauzira program
}
