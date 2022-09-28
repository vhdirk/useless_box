#![deny(unsafe_code)]
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rtic::app;
// use rtt_target::{rprintln, rtt_init_print};
use stm32f1::stm32f103::TIM3;
use stm32f1xx_hal::gpio::{
    gpioc::PC13, Edge, ExtiPin, Input, Output, PinState, PullDown, PushPull, PA3, PA4,
};
use stm32f1xx_hal::pac::TIM1;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::time::MonoTimer;
use stm32f1xx_hal::timer::{Delay, Tim1NoRemap, Tim2NoRemap, Timer};
use systick_monotonic::{fugit::Duration, Systick};

mod hc_sr04;

// This needed some experimentation, only works for our wooden arm thing
const MIN_DUTY_DIVIDER: u16 = 50;
const MAX_DUTY_DIVIDER: u16 = 4;

/// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

fn get_duty_from_angle(
    angle: u16,
    min_duty: u16,
    max_duty: u16,
    min_angle: u16,
    max_angle: u16,
) -> u16 {
    min_duty + angle * ((max_duty - min_duty) / (max_angle - min_angle))
}

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {

    use super::*;
    use stm32f1::stm32f103::TIM2;
    use stm32f1xx_hal::rcc::Clocks;
    use stm32f1xx_hal::timer::PwmChannel;
    use stm32f1xx_hal::timer::{C1, C2};

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<33_000>;

    #[shared]
    struct Shared {
        distance: u32,
        clocks: Clocks,
    }

    #[local]
    struct Local {
        servo_top: PwmChannel<TIM2, C1>,
        servo_bottom: PwmChannel<TIM2, C2>,
        trigger: PA4<Output<PushPull>>,
        trigger_delay: Delay<TIM1, 1_000_000>,
        echo: PA3<Input<PullDown>>,
        instant: <MonoTimer as rtic::Monotonic>::Instant,
    }

    fn update_bottom_arm(distance: u32) {

        // float motorPosition = map(handDistance, 0, distanceMax, botAngleMax, botAngleMin); // convert the sensor distance range to the output angle range, but inverted so that the angle goes up as the distance goes down
        //     Serial.print(distance);
        //     Serial.print(", ");
        //     Serial.println(motorPosition);
        // {
        //     if (distance < 7)
        //     {
        //     commit();
        //     }
        //     else
        //     {
        //     botServo.write(motorPosition);
        //     }
        // }
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::println!("init");

        // Setup clocks
        let mut flash = ctx.device.FLASH.constrain();
        let rcc = ctx.device.RCC.constrain();

        let mono = Systick::new(ctx.core.SYST, 36_000_000);

        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(36.MHz())
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);

        // Setup LED
        let mut gpioc = ctx.device.GPIOC.split();
        let led = gpioc
            .pc13
            .into_push_pull_output_with_state(&mut gpioc.crh, PinState::Low);

        let mut gpioa = ctx.device.GPIOA.split();

        // let delay = Delay::new(p.core.SYST, clocks);
        let trigger = gpioa
            .pa4
            .into_push_pull_output_with_state(&mut gpioa.crl, PinState::Low);

        let mut afio = ctx.device.AFIO.constrain();


        let mut echo = gpioa.pa3.into_pull_down_input(&mut gpioa.crl);

        // Setup interrupt on Pin PA15
        echo.make_interrupt_source(&mut afio);
        echo.trigger_on_edge(&ctx.device.EXTI, Edge::RisingFalling);
        echo.enable_interrupt(&ctx.device.EXTI);

        // We're using Timer 2 which uses the GPIOA pins
        // PA0, PA1, PA2, and PA3 (labeled A0, A1 etc. on the blue-pill)
        let (c1, c2) = (
            gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl),
        );

        let mut servos = ctx.device.TIM2.pwm_hz::<Tim2NoRemap, _, _>(
            (c1, c2),
            &mut afio.mapr,
            100.Hz(),
            &clocks,
        );

        let (mut servo_top, mut servo_bottom) = servos.split();

        // You can connect up to four servos on the same timer
        // It's 0 and 1 here since the servos are at A0 and A1
        servo_top.enable();
        servo_bottom.enable();

        let duty = servo_top.get_max_duty();
        let (min_duty, max_duty) = (duty / MIN_DUTY_DIVIDER, duty / MAX_DUTY_DIVIDER);
        // let min_duty = 10;
        // let max_duty = 20;
        // This is to give it some time to move

        let mut trigger_delay = ctx.device.TIM1.delay(&clocks);

        // let timer = Timer::new(ctx.device.TIM3, &clocks);

        // Spins the two servos in opposite directions
        // 10 degrees at a time, 5 times
        // for _ in 0..5 {
        //     for angle in (0..181).step_by(10) {
        //         delay.delay_ms(100u16);
        //         let duty0 = get_duty_from_angle(angle, min_duty, max_duty, 0, 180);
        //         servo0.set_duty(duty0);

        //         let duty1 = get_duty_from_angle(180 - angle, min_duty, max_duty, 0, 180);
        //         servo1.set_duty(duty1);
        //         delay.delay_ms(100u16);
        //     }
        // }
        servo_top.set_duty(0);
        servo_bottom.set_duty(0);

        trigger_sensor::spawn().unwrap();

        (
            Shared {
                clocks,
                distance: 0,
            },
            Local {
                servo_top,
                servo_bottom,
                trigger,
                trigger_delay,
                echo,
                instant: monotonics::MonoTimer::now(),
            },
            init::Monotonics(mono),
        )
    }

    #[idle(local=[servo_top, servo_bottom], shared=[distance])]
    fn idle(mut ctx: idle::Context) -> ! {
        // let mut i = 0;
        loop {
            cortex_m::asm::nop();

            (ctx.shared.distance).lock(|distance| {
                defmt::println!("Current distance: {}", distance);
            });

            // defmt::println!("main_loop");

            // i += 1;
        }
    }

    #[task(priority = 5, local = [trigger, trigger_delay])]
    fn trigger_sensor(mut ctx: trigger_sensor::Context) {
        ctx.local.trigger.set_high();

        // TODO: is this correct according to the prescaler?
        ctx.local.trigger_delay.delay(10.micros());

        ctx.local.trigger.set_low();
    }

    #[task(priority = 1, binds = EXTI3, local = [echo, instant], shared = [clocks, distance])]
    fn measure_distance(mut ctx: measure_distance::Context) {
        ctx.local.echo.clear_interrupt_pending_bit();

        let now = monotonics::MonoTimer::now();
        if ctx.local.echo.is_high() {
            *ctx.local.instant = now;
        } else {
            let elapsed = now - *ctx.local.instant;

            ctx.shared.distance.lock(|distance| {
                *distance = elapsed.to_micros() as u32;
            });

            trigger_sensor::spawn().unwrap();
        }
    }
}
