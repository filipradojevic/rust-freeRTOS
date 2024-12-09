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

extern crate panic_halt;

#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;


// Staticki semafor
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
    let dp = stm32::Peripherals::take().ok_or("Failed to take peripherals")?;
    let gpioc = dp.GPIOC.split();
    let gpioa = dp.GPIOA.split();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let button = gpioc.pc13.into_pull_up_input(); // PC13 as button with pull-up
    let led = gpioa.pa5.into_push_pull_output(); // PB0 as LED in push-pull mode

    Ok(MyDevice::new(button, led))
}

fn task_button_check<Button: InputPin<Error = E>, LED: OutputPin<Error = E> + ToggleableOutputPin, E: core::fmt::Debug>(
    button: Button,
) {
    let mut last_button_state = false;
    let mut current_button_state = false;

    loop {
        current_button_state = button.is_low().unwrap();

        if current_button_state != last_button_state {
            for _ in 0..1000 {
                // Simulate debouncing
            }

            current_button_state = button.is_low().unwrap();
            last_button_state = current_button_state;

            if current_button_state == false {
                unsafe{
                    freertos_rs_give_semaphore(BUTTON_SEMAPHORE.expect("REASON"));
                }
            }
        }

        CurrentTask::delay(Duration::ms(200));
    }
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
    let device = setup_hardware().unwrap();

    // Kreiraj binarni semafor
    unsafe {
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
