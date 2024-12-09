#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

use cortex_m::asm;
use cortex_m_rt::{entry, ExceptionFrame, exception};
use embedded_hal::digital::v2::OutputPin;
use freertos_rust::*;
use core::alloc::Layout;
use stm32f4xx_hal::gpio::*;
use stm32f4xx_hal as hal;
use crate::hal::stm32::Peripherals;

extern crate panic_halt; // panic handler

#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;

pub struct MyDevice<D1: OutputPin> {
    d1: D1,
}

impl<D1> MyDevice<D1>
where
    D1: OutputPin,
{
    pub fn from_pins(d1: D1) -> MyDevice<D1> {
        MyDevice { d1 }
    }

    pub fn set_led(&mut self, on: bool) {
        if on {
           let _ = self.d1.set_high();
        } else {
           let _ = self.d1.set_low();
        }
    }
}

// Function to handle LED blinking
fn led_blinker<D1>(device: &mut MyDevice<D1>)
where
    D1: OutputPin,
{
    loop {
        freertos_rust::CurrentTask::delay(Duration::ms(1000));
        device.set_led(true);  // Turn LED on
        freertos_rust::CurrentTask::delay(Duration::ms(1000));
        device.set_led(false); // Turn LED off
    }
}

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let gpioa = dp.GPIOA.split(); // Initialize GPIOA instead of GPIOC
    let mut device = MyDevice::from_pins(gpioa.pa5.into_push_pull_output()); // Use PA5 instead of PC13
    device.set_led(false);
    
    // Start the blink task
    Task::new()
        .name("Led blinky")
        .stack_size(128)
        .priority(TaskPriority(1))
        .start(move |_| {
            led_blinker(&mut device); // Call the function that handles LED blinking
        })
        .unwrap();
        
    FreeRtosUtils::start_scheduler();

    loop{};
}



#[exception]
fn DefaultHandler(_irqn: i16) {
    // Custom default handler
    asm::bkpt();
}

#[exception]
fn HardFault(_ef: &ExceptionFrame) -> ! {
    delay_n(10);
    for _ in 0..3 {
        // Blink three times in case of a hard fault
    }
    loop {}
}

// Handle out of memory condition
#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    asm::bkpt();
    loop {}
}

#[no_mangle]
fn vApplicationStackOverflowHook(pxTask: FreeRtosTaskHandle, pcTaskName: FreeRtosCharPtr) {
    asm::bkpt();
}

// Utility function for delays (e.g., for hardfault)
fn delay_n(n: i32) {
    for _ in 0..n {
        delay();
    }
}

fn delay() {
    let mut _i = 0;
    for _ in 0..2_00 {
        _i += 1;
    }
}
