#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_semihosting as _;
// use panic_rtt_target as _;

use nb::block;

use cortex_m::asm;
use cortex_m_rt::entry;
use stm32f1xx_hal as hal;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    time::ms,
    timer::{Channel, Tim2NoRemap, Timer},
};

use core::iter::Iterator;

// This needed some experimentation, only works for our wooden arm thing
const MIN_DUTY_DIVIDER: u16 = 50;
const MAX_DUTY_DIVIDER: u16 = 4;

fn get_duty_from_angle(
    angle: u16,
    min_duty: u16,
    max_duty: u16,
    min_angle: u16,
    max_angle: u16,
) -> u16 {
    min_duty + angle * ((max_duty - min_duty) / (max_angle - min_angle))
}

#[entry]
fn main() -> ! {
    let core_peripherals = cortex_m::Peripherals::take().unwrap();
    let device_peripherals = hal::stm32::Peripherals::take().unwrap();

    let mut flash = device_peripherals.FLASH.constrain();
    let mut rcc = device_peripherals.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut afio = device_peripherals.AFIO.constrain();

    // Acquire the GPIOC peripheral
    let mut gpioc = device_peripherals.GPIOC.split();

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // We're using Timer 2 which uses the GPIOA pins
    // PA0, PA1, PA2, and PA3 (labeled A0, A1 etc. on the blue-pill)
    let mut gpioa = device_peripherals.GPIOA.split();
    let (c1, c2, c3, c4) = (
        gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl),
    );

    let mut servos = device_peripherals.TIM2.pwm_hz::<Tim2NoRemap, _, _>(
        (c1, c2, c3, c4),
        &mut afio.mapr,
        100.Hz(),
        &clocks,
    );

    let (mut servo0, mut servo1, _, _) = servos.split();

    // You can connect up to four servos on the same timer
    // It's 0 and 1 here since the servos are at A0 and A1
    servo0.enable();
    servo1.enable();

    let duty = servo0.get_max_duty();
    let (min_duty, max_duty) = (duty / MIN_DUTY_DIVIDER, duty / MAX_DUTY_DIVIDER);
    // let min_duty = 10;
    // let max_duty = 20;
    // This is to give it some time to move
    let mut delay = core_peripherals.SYST.delay(&clocks);

    // Spins the two servos in opposite directions
    // 10 degrees at a time, 5 times
    for _ in 0..5 {
        for angle in (0..181).step_by(10) {
            delay.delay_ms(100u16);
            let duty0 = get_duty_from_angle(angle, min_duty,max_duty, 0, 180);
            servo0.set_duty(duty0);

            let duty1 = get_duty_from_angle(180 - angle, min_duty, max_duty, 0, 180);
            servo1.set_duty(duty1);
            delay.delay_ms(100u16);
        }
    }
    servo0.set_duty(0);
    servo1.set_duty(0);

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        // // block!(timer.wait()).unwrap();
        // pwm.set_duty(Channel::C3, 0);
        led.set_high();
        // // servo.set_high();
        delay.delay_ms(500u16);

        // // // block!(timer.wait()).unwrap();
        led.set_low();
        // // // servo.set_low();
        delay.delay_ms(500u16);

        // // Adjust period to 0.5 seconds
        // pwm.set_period(ms(500).into_rate());

        // block!(timer.wait()).unwrap();

        // // Return to the original frequency

        // block!(timer.wait()).unwrap();
    }
}
