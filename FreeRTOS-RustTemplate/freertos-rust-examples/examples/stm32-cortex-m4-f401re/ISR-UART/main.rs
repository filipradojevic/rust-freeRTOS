#![no_std]
#![no_main]

#![feature(alloc_error_handler)]

extern crate alloc;

use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{gpio::{self, Input, PullUp, Output, PushPull}, prelude::*};
use freertos_rust::*;
use stm32f4xx_hal::serial::{Serial, config::{Config, Parity, StopBits, WordLength}};
use cortex_m_rt::exception;
use stm32f4xx_hal::stm32;
use stm32f4xx_hal::rcc::Clocks;
use stm32f4xx_hal::delay::Delay;
use stm32f4xx_hal::gpio::gpioc::PC13;
use stm32f4xx_hal::gpio::gpioa::PA5;
use stm32f4xx_hal::serial::Event;
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use stm32f4xx_hal::interrupt;
use core::cell::{RefCell, Cell};
use cortex_m_rt::ExceptionFrame;
use alloc::alloc::Layout;
use stm32f4xx_hal::gpio::ExtiPin;
use stm32f4xx_hal::gpio::AF7;
use stm32f4xx_hal::gpio::Alternate;
use stm32f4xx_hal::gpio::gpioa::PA3;
use stm32f4xx_hal::gpio::gpioa::PA2;

extern crate panic_halt;


type ButtonPin = PC13<Input<PullUp>>;
type SerialPins = (PA2<Alternate<AF7>>, PA3<Alternate<AF7>>);


#[global_allocator]
static GLOBAL: freertos_rust::FreeRtosAllocator = freertos_rust::FreeRtosAllocator;

// Global Variable Definitions
static G_BUTTON: Mutex<RefCell<Option<ButtonPin>>> = Mutex::new(RefCell::new(None));
static G_USART: Mutex<RefCell<Option<Serial<stm32f4xx_hal::stm32::USART2, SerialPins>>>> = Mutex::new(RefCell::new(None));
static G_DELAYMS: Mutex<Cell<u32>> = Mutex::new(Cell::new(2000_u32));
static G_RECEIVED_DATA: Mutex<Cell<Option<u8>>> = Mutex::new(Cell::new(None));
static G_LED: Mutex<RefCell<Option<PA5<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));


#[entry]
fn main() -> ! {
    // Setup handler for device peripherals
    let dp = stm32::Peripherals::take().unwrap(); // Uzimaš vlasništvo nad svim periferijama mikrokontrolera

    // Direktan pristup dp.RCC (u ovom slučaju bez korišćenja constrain)
    dp.RCC.cr.modify(|_, w| w.hseon().set_bit()); // Uključujemo HSE oscilator

    // Postavljamo HSE kao sistemski sat
    dp.RCC.cfgr.modify(|_, w| w.sw().hse());

    // Omogućavamo takte za GPIO portove A i C
    dp.RCC.ahb1enr.modify(|_, w| w.gpioaen().enabled().gpiocen().enabled());

    // Omogućavanje takta za USART2
    dp.RCC.apb1enr.modify(|_, w| w.usart2en().enabled());

    // Omogućavanje takta za SYSCFG
    dp.RCC.apb2enr.modify(|_, w| w.syscfgen().enabled());

    // Postavljamo GPIO portove
    let gpioa = dp.GPIOA.split(); // Delimo GPIO port A
    let led = gpioa.pa5.into_push_pull_output(); // Postavljamo pin PA5 kao izlaz za LED
    let tx = gpioa.pa2.into_alternate_af7(); // TX pin kao alternativni funkcionalni pin AF7 za USART2
    let rx = gpioa.pa3.into_alternate_af7(); // RX pin kao alternativni funkcionalni pin AF7 za USART2

    // Konfiguracija USART-a
    let config = Config {
        baudrate: 115200.bps(),
        wordlength: WordLength::DataBits8,
        stopbits: StopBits::STOP1,
        parity: Parity::ParityNone,
        ..Default::default()
    };

    let mut serial = Serial::usart2(
        dp.USART2, 
        (tx, rx), 
        config, 
        dp.RCC.constrain().cfgr.freeze()
    ).unwrap();

    
    // Omogućavanje RX interrupt-a
    serial.listen(Event::Rxne); // RXNE (Receive Data Register Not Empty)

    cortex_m::interrupt::free(|cs| {
        G_USART.borrow(cs).replace(Some(serial));
        G_LED.borrow(cs).replace(Some(led));
    });
    // Omogućavanje USART2 interrupt-a u NVIC
    unsafe {
        cortex_m::peripheral::NVIC::unmask(stm32f4xx_hal::stm32::Interrupt::USART2);
    }

    // Postavljanje dugmeta na pin PC13 sa pull-up otpornikom
    let gpioc = dp.GPIOC.split(); // Delimo GPIO port C
    let button: ButtonPin = gpioc.pc13.into_pull_up_input(); // Postavljamo pin PC13 kao ulaz sa pull-up otpornikom

    // Podešavanje prekida za dugme na PC13
    dp.SYSCFG.exticr4.modify(|_, w| unsafe { w.exti13().bits(0x2) }); // Postavljamo EXTI13 na port C
    dp.EXTI.ftsr.modify(|_, w| w.tr13().set_bit()); // Konfigurišemo Falling edge trigger za EXTI13
    dp.EXTI.rtsr.modify(|_, w| w.tr13().clear_bit()); // Brišemo Rising edge trigger za EXTI13
    dp.EXTI.imr.modify(|_, w| w.mr13().set_bit()); // Omogućavamo EXTI13 interrupt

    // Omogućavamo EXTI13 interrupt u NVIC
    unsafe {
        cortex_m::peripheral::NVIC::unmask(stm32f4xx_hal::stm32::Interrupt::EXTI15_10);
    }

    // Čuvamo referencu na dugme u globalnoj promenljivoj
    cortex_m::interrupt::free(|cs| {
        G_BUTTON.borrow(cs).replace(Some(button));
    });

    loop {
       // Čekanje na prekid
        asm::wfi();  // Poziva wfi instrukciju i stavlja procesor u stanje mirovanja dok ne stigne prekid
        // Provera da li je postavljena vrednost u `G_RECEIVED_DATA`
        cortex_m::interrupt::free(|cs| {
            if let Some(data) = G_RECEIVED_DATA.borrow(cs).get() {
                if let Some(led) = G_LED.borrow(cs).borrow_mut().as_mut(){
                match data {
                    b'e' => {
                        led.set_high().unwrap(); // Uključuje LED
                    }
                    b'd' => {
                        led.set_low().unwrap(); // Isključuje LED
                    }
                    _ => {
                        //Nista
                    }
                }
            }
                // Resetujemo primljenu vrednost
                G_RECEIVED_DATA.borrow(cs).set(None);
            }
        });
    }
}

// Prekid handler za EXTI15_10 (prekid sa dugmadi)
#[interrupt]
fn EXTI15_10() {
    cortex_m::interrupt::free(|cs| {
        
        // Čistimo pending interrupt bit za EXTI13
        let mut button = G_BUTTON.borrow(cs).borrow_mut();
        button.as_mut().unwrap().clear_interrupt_pending_bit();
    });
}

#[interrupt]
fn USART2() {
    cortex_m::interrupt::free(|cs| {
        // Uzimanje globalnog serijskog porta
        let serial_opt = G_USART.borrow(cs).borrow_mut().take();
        if let Some(mut serial) = serial_opt {
            if serial.is_rxne() {
                // Čitanje podataka iz USART-a
                if let Ok(received_data) = serial.read() {
                    // Postavljanje vrednosti u globalnu promenljivu
                    G_RECEIVED_DATA.borrow(cs).set(Some(received_data));
                }
            }

            // Vraćanje serial objekta nazad u globalnu promenljivu
            G_USART.borrow(cs).replace(Some(serial));
        }
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
