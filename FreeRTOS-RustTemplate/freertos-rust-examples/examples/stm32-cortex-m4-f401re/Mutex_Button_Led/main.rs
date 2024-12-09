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

const PORT_MAX_DELAY: u32 = u32::MAX; // Maksimalno čekanje

#[derive(PartialEq, Debug, Clone, Copy)]
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
        self.led.toggle().unwrap(); // Direktno pozivamo `toggle()` na LED-u
    }
}

fn setup_hardware() -> Result<MyDevice<PC13<Input<PullUp>>, PA5<Output<PushPull>>>, &'static str> {
    let dp = stm32::Peripherals::take().ok_or("Failed to take peripherals")?; 
    let gpioc = dp.GPIOC.split();
    let gpioa = dp.GPIOA.split();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let button = gpioc.pc13.into_pull_up_input(); // PC13 as button with pull-up
    let led = gpioa.pa5.into_push_pull_output(); // PA5 as LED in push-pull mode

    Ok(MyDevice::new(button, led))
}

fn task_button_check<Button, LED, E, FreeRtosTickType>(
    button: Button,
    mutex: Arc<Mutex<ButtonState>>, // Arc<Mutex<ButtonState>> for shared state
    semaphore: Arc<Semaphore>,      // Binary semaphore to signal LED task
) 
where
    Button: InputPin<Error = E>,
    LED: OutputPin<Error = E> + ToggleableOutputPin,
    E: core::fmt::Debug,

{

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
                // Lock the mutex to update the shared state (ButtonState)
                let max_wait = DurationImpl::<FreeRtosTimeUnitsShimmed>::new(PORT_MAX_DELAY);
                
                // `lock` će automatski uzeti mutex i vratiti `MutexGuard`
                if let Ok(mut guard) = mutex.lock(max_wait) {
                    // Update shared state via `MutexGuard`
                    *guard = ButtonState::Pressed;
                }

                // Signal the semaphore (button pressed)
                semaphore.give();

                // Delay for debouncing
                CurrentTask::delay(Duration::ms(200));
            }
        }
    }
}

fn task_led_blink<LED, E>(
    mut led: LED,
    mutex: Arc<Mutex<ButtonState>>, // Arc<Mutex<ButtonState>> for shared state
    semaphore: Arc<Semaphore>,
) 
where
    LED: OutputPin<Error = E> + ToggleableOutputPin,
    E: core::fmt::Debug,
{
    
    let mut LOCAL_BUTTON_STATE: ButtonState = ButtonState::Undefined;
    loop {
        
        // Wait for semaphore (button press detected)
        semaphore.take(Duration::infinite()).unwrap();

        // Lock the mutex to check the shared button state
        let max_wait = DurationImpl::<FreeRtosTimeUnitsShimmed>::new(PORT_MAX_DELAY);

        // `lock` automatski uzima mutex i vraća `MutexGuard`
        if let Ok(guard) = mutex.lock(max_wait) {
            LOCAL_BUTTON_STATE = *guard;
        }
        if LOCAL_BUTTON_STATE == ButtonState::Pressed{
            led.toggle();
        }
    }
}


#[entry]
fn main() -> ! {
    let device = setup_hardware().unwrap();

    // Kreiraš mutex sa početnom vrednošću
    let mutex_send = Mutex::new(ButtonState::Undefined).unwrap(); // Create Mutex
    let mutex_receive = Arc::new(mutex_send);  // Wrap the Mutex in an Arc so it can be cloned

    // Kloniranje mutex-a kada se koristi u različitim zadacima
    let mutex_give = Arc::clone(&mutex_receive);
    let mutex_take = Arc::clone(&mutex_receive);

    let semaphore = Arc::new(Semaphore::new_binary().expect("Failed to create binary semaphore"));

    let semaphore_give = Arc::clone(&semaphore);
    let semaphore_take = Arc::clone(&semaphore);

    let button = device.button;
    let led = device.led;

    // Task for button press detection
    // Task for button press detection
    match Task::new()
    .name("Button Task")
    .stack_size(128)
    .priority(TaskPriority(2))
    .start(move |_| {
        task_button_check::<PC13<Input<PullUp>>, PA5<Output<PushPull>>, Infallible, FreeRtosTickType>(
            button, mutex_give, semaphore_give);
    })
    {
    Ok(_) => {}
    Err(_) => loop {} 
    }


    // Task for LED blinking
    match Task::new()
        .name("LED Blink Task")
        .stack_size(128)
        .priority(TaskPriority(1))
        .start(move |_| {
            task_led_blink(led, mutex_take, semaphore_take); // Corrected
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
