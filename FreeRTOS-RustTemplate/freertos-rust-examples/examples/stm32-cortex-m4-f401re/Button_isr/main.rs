#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

use alloc::sync::Arc;
use cortex_m::asm;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use freertos_rust::*;
use core::convert::Infallible;
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use stm32f4xx_hal::gpio::{GpioExt, Output, Input, PullUp, PushPull};
use stm32f4xx_hal::stm32;
use alloc::alloc::Layout;
use stm32f4xx_hal::gpio::gpioa::PA5;
use stm32f4xx_hal::gpio::gpioc::PC13;
use stm32f4xx_hal::prelude::_stm32f4xx_hal_rcc_RccExt;
use stm32f4xx_hal::interrupt;
use cortex_m::peripheral::NVIC;
use stm32f4::stm32f401::Peripherals;


use core::sync::atomic::{compiler_fence, Ordering};


extern crate panic_halt;

#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;

const CONFIG_MAX_SYSCALL_INTERRUPT_PRIORITY: u8 = 5;

// Staticki semafor
static mut BUTTON_SEMAPHORE_ISR: Option<FreeRtosSemaphoreHandle> = None;
static mut BUTTON_SEMAPHORE: Option<FreeRtosSemaphoreHandle> = None;

pub struct MyDevice<Button: InputPin, LED: OutputPin + ToggleableOutputPin> {
    button: Button,
    led: LED,
}

impl<Button, LED, E> MyDevice<Button, LED>
where
    Button: InputPin<Error = E>,
    LED: OutputPin<Error = E> + ToggleableOutputPin<Error = E>,
    E: core::fmt::Debug,
{
    pub fn new(button: Button, led: LED) -> MyDevice<Button, LED> {
        MyDevice { button, led }
    }

    pub fn is_button_pressed(&self) -> bool {
        self.button.is_low().unwrap()
    }

    pub fn toggle_led(&mut self) {
        self.led.toggle().unwrap(); // Directly call `toggle()` on LED
    }
}

fn setup_hardware() -> Result<MyDevice<PC13<Input<PullUp>>, PA5<Output<PushPull>>>, &'static str> {
    let dp = stm32::Peripherals::take().unwrap();
    //let core_peripherals = cortex_m::Peripherals::take().unwrap();
    //let mut nvic = core_peripherals.NVIC;


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
    let nvic = unsafe { &(*cortex_m::peripheral::NVIC::ptr()) };

    unsafe {
        let priority_shifted = CONFIG_MAX_SYSCALL_INTERRUPT_PRIORITY << 4;
        let interrupt_number = interrupt::EXTI15_10 as usize;

        // Pristup odgovarajućem `ipr` registru
        let ipr_index = interrupt_number;
        nvic.ipr[ipr_index].write(priority_shifted as u8); // Upis prioriteta u NVIC_IPRn

        let iser_index = interrupt_number / 32;
        let setena_bit = 1 << (interrupt_number % 32);
        nvic.iser[iser_index].write(setena_bit);
    }
    /* 
    // Postavljanje globalnih pinova
    free(|cs| {
        G_BUTTON.borrow(cs).replace(Some(button));
        G_LED.borrow(cs).replace(Some(led));
    });
    */

    Ok(MyDevice::new(button, led))
}

fn task_button_check<Button: InputPin<Error = E>, LED: OutputPin<Error = E> + ToggleableOutputPin, E: core::fmt::Debug>(
    button: Button,
) {
    let mut current_button_state = false;

    loop {
        
        unsafe {
            freertos_rs_take_semaphore(BUTTON_SEMAPHORE_ISR.expect("REASON"), 0xFFFFFFFF);
        }

        for _ in 0..1000 {
            // Simulate debouncing
        }

        current_button_state = button.is_low().unwrap();
        

        if current_button_state == true {
            unsafe{
                freertos_rs_give_semaphore(BUTTON_SEMAPHORE.expect("REASON"));
            }
            }
        }

        //CurrentTask::delay(Duration::ms(200));    
}



fn task_led_blink<LED: OutputPin<Error = E> + ToggleableOutputPin, E: core::fmt::Debug>(
    mut led: LED,
) {
    loop {
        unsafe{
            freertos_rs_take_semaphore(BUTTON_SEMAPHORE.expect("REASON"), 0xFFFFFFFF);
        }
        led.toggle();  // Directly use `toggle()` method on LED pin
    }
}

#[entry]
fn main() -> ! {
    compiler_fence(Ordering::Release);
    let device = setup_hardware().unwrap();

    // Kreiraj binarni semafor
    unsafe {
        BUTTON_SEMAPHORE_ISR = Some(freertos_rs_create_binary_semaphore());
        BUTTON_SEMAPHORE = Some(freertos_rs_create_binary_semaphore());
    }
    let button = device.button;
    let led = device.led;
    

    // Task for button press detection
    match Task::new()
        .name("Button Task")
        .stack_size(128)
        .priority(TaskPriority(1))
        .start(move |_| {
            task_button_check::<PC13<Input<PullUp>>, PA5<Output<PushPull>>, Infallible>(button);
        })
    {
        Ok(_) => {}
        Err(_) => loop {} 
    }

    // Task for LED blinking
    match Task::new()
        .name("LED Blink Task")
        .stack_size(128)
        .priority(TaskPriority(2))
        .start(move |_| {
            task_led_blink::<PA5<Output<PushPull>>, Infallible>(led);
        })
    {
        Ok(_) => {}
        Err(_) => loop {} 
    }

    FreeRtosUtils::start_scheduler();
}

#[interrupt]
fn EXTI15_10() {
    // Očistite zastavicu prekida
    
    let mut context = InterruptContext::new();
    // Kastovanje u FreeRtosBaseTypeMutPtr (ako je validno)
    let x_higher_priority_task_woken: FreeRtosBaseTypeMutPtr = &mut context as *mut _ as FreeRtosBaseTypeMutPtr;
    // Signalizacija semaforu iz ISR-a
    unsafe {
           // semaphore.give_from_isr(&mut context);
            freertos_rs_give_semaphore_isr(BUTTON_SEMAPHORE_ISR.expect("REASON"), x_higher_priority_task_woken);
            // Ako je potrebno prebacivanje konteksta
            freertos_rs_isr_yield(*context.get_task_field_mut());
        }
    let exti = unsafe { &*stm32::EXTI::ptr() };
    exti.pr.write(|w| w.pr13().set_bit());
}

#[exception]
fn DefaultHandler(_irqn: i16) {
    // Custom default handler
}

#[exception]
fn HardFault(_ef: &ExceptionFrame) -> ! {
    loop {}
}

#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    asm::bkpt();
    loop {}
}

#[no_mangle]
fn vApplicationStackOverflowHook(_pxTask: FreeRtosTaskHandle, _pcTaskName: FreeRtosCharPtr) {
    asm::bkpt();
}
