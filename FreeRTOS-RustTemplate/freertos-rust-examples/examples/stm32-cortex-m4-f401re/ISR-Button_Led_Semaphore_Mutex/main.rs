#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

use alloc::sync::Arc;
use cortex_m::asm;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use freertos_rust::*;
use cortex_m::interrupt::{Mutex, free};
use core::cell::RefCell;
use embedded_hal::digital::v2::{InputPin, ToggleableOutputPin};
use stm32f4xx_hal::gpio::{GpioExt, Output, Input, PullUp, PushPull};
use stm32f4xx_hal::stm32;
use stm32f4::stm32f401::NVIC;
use stm32f4xx_hal::interrupt;
use core::borrow::BorrowMut;
use core::borrow::Borrow;
use alloc::alloc::Layout;
use stm32f4xx_hal::gpio::gpioa::PA5;
use stm32f4xx_hal::gpio::gpioc::PC13;
use freertos_rust::FreeRtosSemaphoreHandle;
use freertos_rust::freertos_rs_give_semaphore_isr;
use core::ffi::c_void;
use freertos_rust::DurationImpl;
use freertos_rust::FreeRtosTimeUnitsShimmed;

const PORT_MAX_DELAY: u32 = u32::MAX; // Maksimalno čekanje

extern crate panic_halt;

#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;

// Definicija stanja dugmeta
#[derive(PartialEq, Clone)]
enum button_t {
    BUTTON_PRESSED = 0,
    BUTTON_UNDEF,
}


const QUEUE_LENGHT :i8 = 10;

// Tip dugmeta
type ButtonPin = PC13<Input<PullUp>>;

// Staticki semafor
static mut BUTTON_SEMAPHORE: Option<FreeRtosSemaphoreHandle> = None;

fn setup_hardware() -> Result<(), &'static str> {
    let dp = stm32::Peripherals::take().unwrap();

    // Inicijalizacija HSE oscilatora i GPIO portova
    dp.RCC.cr.modify(|_, w| w.hseon().set_bit());
    dp.RCC.cfgr.modify(|_, w| w.sw().hse());

    dp.RCC.ahb1enr.modify(|_, w| w.gpioaen().enabled().gpiocen().enabled());
    dp.RCC.apb2enr.modify(|_, w| w.syscfgen().enabled());

    // Inicijalizacija GPIO pinova
    let gpioa = dp.GPIOA.split();
    let led = gpioa.pa5.into_push_pull_output();

    let gpioc = dp.GPIOC.split();
    let button: PC13<Input<PullUp>> = gpioc.pc13.into_pull_up_input();

    // Konfigurisanje eksternih prekida za dugme
    dp.SYSCFG.exticr4.modify(|_, w| unsafe { w.exti13().bits(0x2) });
    dp.EXTI.ftsr.modify(|_, w| w.tr13().set_bit());
    dp.EXTI.rtsr.modify(|_, w| w.tr13().clear_bit());
    dp.EXTI.imr.modify(|_, w| w.mr13().set_bit());

    // Maskiranje prekida
    unsafe {
        NVIC::unmask(interrupt::EXTI15_10);
    }

    /* 
    // Postavljanje globalnih pinova
    free(|cs| {
        G_BUTTON.borrow(cs).replace(Some(button));
        G_LED.borrow(cs).replace(Some(led));
    });
    */
    Ok(())
}

fn button_task() {
    loop {
        // Uzmi semafor
        unsafe {
            let max_wait: DurationImpl<FreeRtosTimeUnitsShimmed> = DurationImpl::ticks(freertos_rs_max_wait());
            freertos_rs_give_semaphore(BUTTON_SEMAPHORE.expect("REASON"));
                
                // Dugme je pritisnuto - dodajte ovde željenu logiku
            let a = 5; // Ovo je placeholder za vašu funkciju ili obradu
            }
        }
}



#[entry]
fn main() -> ! {
    setup_hardware().unwrap();

    // Kreiraj binarni semafor
    unsafe {
        BUTTON_SEMAPHORE = Some(freertos_rs_create_binary_semaphore());
        
        
        let b = 1;
        }
    
    // Kreiraj task
    match Task::new()
        .name("Button Task")
        .stack_size(128)
        .priority(TaskPriority(1))
        .start(|_| button_task())
    {
        Ok(_)  => {}
        Err(_) => {
            loop {}
        }
    }

    // Startuj scheduler
    FreeRtosUtils::start_scheduler();

    loop {}
}


#[interrupt]
fn EXTI15_10() {
    // Očistite zastavicu prekida
    let exti = unsafe { &*stm32::EXTI::ptr() };
    exti.pr.write(|w| w.pr13().set_bit());

    // Signalizacija semaforu iz ISR-a
    unsafe {
        
            let mut context = InterruptContext::new();
    // Kastovanje u FreeRtosBaseTypeMutPtr (ako je validno)
            let  context_ptr: FreeRtosBaseTypeMutPtr = &mut context as *mut _ as FreeRtosBaseTypeMutPtr;
           // semaphore.give_from_isr(&mut context);
            freertos_rs_give_semaphore_isr(BUTTON_SEMAPHORE.expect("REASON"), context_ptr);
            // Ako je potrebno prebacivanje konteksta
            freertos_rs_isr_yield(*context.get_task_field_mut());
        }
    
}

/*fn task_button_check() {
    loop {
        
    }
}
*/

// Default exception handler
#[exception]
fn DefaultHandler(_irqn: i16) {
    loop {}
}

// Hard fault exception handler
#[exception]
fn HardFault(_ef: &ExceptionFrame) -> ! {
    loop {}
}

// Alociranje greške
#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    asm::bkpt();
    loop {}
}

// Stack overflow hook
#[no_mangle]
fn vApplicationStackOverflowHook(_pxTask: FreeRtosTaskHandle, _pcTaskName: FreeRtosCharPtr) {
    asm::bkpt();
}