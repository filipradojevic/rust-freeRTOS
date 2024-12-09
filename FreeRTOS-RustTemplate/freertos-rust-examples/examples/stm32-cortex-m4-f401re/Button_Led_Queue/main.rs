#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

use alloc::sync::Arc;
use cortex_m::asm;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use freertos_rust::*;
use core::convert::{Infallible, TryInto};
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use stm32f4xx_hal::gpio::{GpioExt, Output, Input, PullUp, PushPull};
use stm32f4xx_hal::stm32;
use alloc::alloc::Layout;
use stm32f4xx_hal::gpio::gpioa::PA5;
use stm32f4xx_hal::gpio::gpioc::PC13;
use stm32f4xx_hal::prelude::_stm32f4xx_hal_rcc_RccExt;



extern crate panic_halt;

const QUEUE_LENGHT :i8 = 10;
const portMAX_DELAY: u32 = u32::MAX;  // Koristi maksimalnu vrednost za u32 kao portMAX_DELAY

#[derive(PartialEq)]
enum ButtonState {
    Pressed = 0,
    Undefined,
}

#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;

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

fn task_button_check<Button: InputPin<Error = E>, LED: OutputPin<Error = E> + ToggleableOutputPin, E: core::fmt::Debug,  FreeRtosTickType>(
    button: Button,
    queue: Arc<Queue<ButtonState>>,
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
                // Send ButtonState::Pressed when button is pressed
                // Sada možemo koristiti DurationImpl sa funkcijom send
                let max_wait = DurationImpl::<FreeRtosTimeUnitsShimmed>::new(portMAX_DELAY);
                queue.send(ButtonState::Pressed, max_wait);
            
        }

        CurrentTask::delay(Duration::ms(200)); // debounce delay
    }
}
}


fn task_led_blink<LED: OutputPin<Error = E> + ToggleableOutputPin, E: core::fmt::Debug>(
    mut led: LED,
    queue: Arc<Queue<ButtonState>>,
) {
    loop {
        let max_wait = DurationImpl::<FreeRtosTimeUnitsShimmed>::new(portMAX_DELAY);
        let state = queue.receive(max_wait).unwrap(); // Receive button state
            
        if state == ButtonState::Pressed {
            // Turn on the LED when button is pressed
            led.toggle();
        }
        else{
            //NOP
        }
    }
}


#[entry]
fn main() -> ! {
    let device = setup_hardware().unwrap();

    let queue = Arc::new(Queue::new(QUEUE_LENGHT.try_into().unwrap()).expect("Failed to create binary semaphore"));

    let queue_send = Arc::clone(&queue);
    let queue_receive = Arc::clone(&queue);

    let button = device.button;
    let led = device.led;

    // Task for button press detection
    match Task::new()
        .name("Button Task")
        .stack_size(128)
        .priority(TaskPriority(1))
        .start(move |_| {
            task_button_check::<PC13<Input<PullUp>>, PA5<Output<PushPull>>, Infallible,  FreeRtosTickType>(button,queue_send);
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
            task_led_blink::<PA5<Output<PushPull>>, Infallible>(led, queue_receive);
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
