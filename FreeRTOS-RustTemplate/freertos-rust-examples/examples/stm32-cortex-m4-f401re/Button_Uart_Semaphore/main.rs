#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

use alloc::sync::Arc;
use alloc::fmt::Debug;
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::asm;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use freertos_rust::*;
use embedded_hal::digital::v2::InputPin;
use stm32f4xx_hal::gpio::{Input, PullUp, GpioExt, Alternate, PushPull};
use stm32f4xx_hal::stm32;
use stm32f4xx_hal::gpio::gpioc::PC13;
use stm32f4xx_hal::gpio::gpioa::{PA2, PA3};
use alloc::alloc::Layout;
use stm32f4xx_hal::prelude::_stm32f4xx_hal_rcc_RccExt;
use nb::block;
use stm32f4xx_hal::gpio::AF7;
use embedded_hal::prelude::_embedded_hal_serial_Write;
use stm32f4xx_hal::serial::{Serial, config::{Config, StopBits, Parity, WordLength}};



extern crate panic_halt;

#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;

pub struct MyDevice<Button: InputPin> {
    button: Button,
}

impl<Button, E> MyDevice<Button>
where
    Button: InputPin<Error = E>,
    E: core::fmt::Debug,
{
    pub fn new(button: Button) -> MyDevice<Button> {
        MyDevice { button }
    }

    pub fn is_button_pressed(&self) -> bool {
        self.button.is_low().unwrap()
    }
}

fn setup_hardware() -> Result<(
    MyDevice<PC13<Input<PullUp>>>, 
    Serial<stm32::USART2, (PA2<Alternate<AF7>>, PA3<Alternate<AF7>>)>) , 
    &'static str> 
{
    let dp = stm32::Peripherals::take().ok_or("Failed to take peripherals")?;
    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    // Konfiguracija USART2 sa parametrom za 9600 baud rate, 8-bitnu dužinu reči,
    // bez pariteta, i sa 1 stop bitom.
    let serial = Serial::usart2(
        dp.USART2,
        (
            gpioa.pa2.into_alternate_af7(), // PA2 kao USART2 TX
            gpioa.pa3.into_alternate_af7(), // PA3 kao USART2 RX
        ),
        Config {
            baudrate: stm32f4xx_hal::time::Bps(9600), // Baud rate 9600        
            stopbits: StopBits::STOP1,     // 1 stop bit
            ..Default::default()    
        },
        clocks,
    ).map_err(|_| "Failed to initialize USART2")?;

    Ok((
        MyDevice::new(gpioc.pc13.into_pull_up_input()),  // Inicijalizacija uređaja sa pull-up za PC13
        serial,
    ))
}


fn task_button_check<Button: InputPin<Error = E>, E: Debug>(
    button: Button,
    semaphore: Arc<Semaphore>,
) {
    let mut last_button_state = false;
    let mut current_button_state = false;

    loop {
        current_button_state = button.is_low().unwrap();

        if current_button_state != last_button_state {
            for _ in 0..1000 {
                // Simuliraj debouncing
            }

            current_button_state = button.is_low().unwrap();
            last_button_state = current_button_state;

            if current_button_state == false {
                semaphore.give();
            }
        }

        CurrentTask::delay(Duration::ms(200));
    }
}

fn task_uart_send(
    mut serial: Serial<stm32::USART2, (PA2<Alternate<AF7>>, PA3<Alternate<AF7>>)>, 
    semaphore: Arc<Semaphore>
) {
    loop {
        semaphore.take(Duration::infinite()).unwrap();
        let message = "Button pressed!\r\n";
        for byte in message.bytes() {
            nb::block!(serial.write(byte)).ok();
        }
    }
}

#[entry]
fn main() -> ! {
    let (device, serial) = setup_hardware().unwrap();

    let semaphore = Arc::new(Semaphore::new_binary().expect("Failed to create binary semaphore"));
    
    let semaphore_give = Arc::clone(&semaphore);
    let semaphore_take = Arc::clone(&semaphore);
    
    let button = device.button;

    match Task::new()
        .name("Button Task")
        .stack_size(128)
        .priority(TaskPriority(1))
        .start(move |_| {
            task_button_check(button, semaphore_give);
        })
    {
        Ok(_) => {}
        Err(_) => loop {},
    }
    
    match Task::new()
        .name("UART Task")
        .stack_size(128)
        .priority(TaskPriority(2))
        .start(move |_| {
            task_uart_send(serial, semaphore_take);
        })
    {
        Ok(_) => {}
        Err(_) => loop {},
    }

    FreeRtosUtils::start_scheduler();

    loop {}
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
