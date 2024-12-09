#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

use cortex_m::asm;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use freertos_rust::*;
use stm32f4xx_hal::gpio::{Output, PushPull, Input, PullUp, GpioExt};
use stm32f4xx_hal::stm32;
use stm32f4xx_hal::gpio::gpioa::PA5;
use stm32f4xx_hal::gpio::gpioc::PC13;
use core::alloc::Layout;

extern crate panic_halt;

// Globalni allocator za FreeRTOS
#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;

// Struktura za uređaj
pub struct MyDevice<Led: OutputPin, Button: InputPin> {
    led: Led,
    button: Button,
}

impl<Led, Button, E> MyDevice<Led, Button>
where
    Led: OutputPin<Error = E> + ToggleableOutputPin,
    Button: InputPin<Error = E>,
    E: core::fmt::Debug,
{
    pub fn new(led: Led, button: Button) -> MyDevice<Led, Button> {
        MyDevice { led, button }
    }

    pub fn toggle_led(&mut self) {
        self.led.toggle().unwrap_or_else(|_| ()); 
    }

    pub fn is_button_pressed(&self) -> bool {
        let pressed = self.button.is_low().unwrap();
        pressed  // Ovo će vratiti vrednost pressed kao rezultat funkcije
    }
}

fn setup_hardware() -> MyDevice<PA5<Output<PushPull>>, PC13<PullUp>> {
    let dp = stm32::Peripherals::take().unwrap();
    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    // Inicijalizuj uređaj
    MyDevice::new(
        gpioa.pa5.into_push_pull_output(),
        gpioc.pc13.into_pull_up_input(),  // PC13 kao dugme
    )
}

fn task_button_check(device: &mut MyDevice<PA5<Output<PushPull>>, PC13<PullUp>>) {
    let mut last_button_state = false;  // Početno stanje tastera je 1 zbog pull-up konfiguracije
    let mut current_button_state = false;

    loop {
        // Pročitaj stanje tastera
        current_button_state = device.is_button_pressed();
        
        // Detektuj promenu stanja tastera
        if current_button_state != last_button_state {
            // Pauza za debouncing (1000 ciklusa, kao u C kodu)
            for _ in 0..1000 {
                // Empty loop, ali se mogu koristiti neki drugi delay pristupi
            }
            
            // Ponovno proveri stanje tastera kako bi se uverio da je prelaz na pravom mestu
            current_button_state = device.is_button_pressed();
            last_button_state = current_button_state;
            
            if current_button_state == false {
                // Ako je taster pritisnut, togluj LED
                device.toggle_led();
            }
        }
        
        // Sinhronizacija sa delay-om (200ms kao u C kodu)
        CurrentTask::delay(Duration::ms(200));
    }
}

#[entry]
fn main() -> ! {
    // Inicijalizuj uređaj
    let mut device = setup_hardware();

    // Kreiraj zadatak za proveru dugmeta i upravljanje LED
    match Task::new()
        .name("Button Task")
        .stack_size(128)
        .priority(TaskPriority(1))
        .start(move |_| {
            task_button_check(&mut device);
        }) 
    {
        Ok(_) => {},  // Zadatak je uspešno kreiran
        Err(_) => loop { /* Greška pri kreiranju, ulazimo u beskonačnu petlju */ },
    }

    // Pokreni FreeRTOS scheduler
    FreeRtosUtils::start_scheduler();

    loop { /* Glavna petlja ostaje prazna jer je scheduler sada aktivan */ }
}

#[exception]
fn DefaultHandler(_irqn: i16) {
    // Custom default handler
    // irqn is negative for Cortex-M exceptions
    // irqn is positive for device specific (line IRQ)
    // set_led(true);
    // panic!("Exception: {}", irqn);
}

#[exception]
fn HardFault(_ef: &ExceptionFrame) -> ! {
    // Blink 3 times long when exception occurs
    for _ in 0..3 {
        // set_led(true);
        // delay_n(1000);
        // set_led(false);
        // delay_n(555);
    }
    loop {}
}

// Define what happens in an Out Of Memory (OOM) condition
#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    // set_led(true);
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
