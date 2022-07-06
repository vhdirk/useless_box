//! A platform agnostic driver to interface the [`HC-SR04`][2] (ultrasonic distance sensor).
//!
//! This driver is built using [`embedded-hal`][1] traits.
//!
//! # Usage
//! Currently the sensor is implemented with interrupts in mind. This allows
//! us to accurately measure the pulse width, but at the cost of needing
//! external support for calling `HcSr04::update`.
//!
//! See the `examples` folder for further information.
//!
//! [1]: https://crates.io/crates/embedded-hal
//! [2]: http://www.micropik.com/PDF/HCSR04.pdf

// #![deny(missing_docs)]
// #![deny(warnings)]

use stm32f1xx_hal::timer::DelayUs;
use embedded_hal::digital::v2::OutputPin;
/// Publicly re-export `nb::Error` for easier usage down-stream
pub use nb::Error;
use stm32f1xx_hal::rcc::Clocks;
use stm32f1xx_hal::time::Instant;
use stm32f1xx_hal::time::MonoTimer;
use stm32f1xx_hal::timer::Delay;
use stm32f1xx_hal::timer::Instance;
use stm32f1xx_hal::timer::Timer;
use stm32f1xx_hal::timer::TimerExt;
use systick_monotonic::fugit::{Duration, ExtU32};

/// Wrapper for return value of sensor
#[derive(Debug, Copy, Clone)]
pub struct Distance(u32);

impl Distance {
    /// Get distance as centimeters.
    pub fn cm(&self) -> u32 {
        self.0 / 10
    }

    /// Get distance as millimeters.
    pub fn mm(&self) -> u32 {
        self.0
    }
}

/// Possible error returned by sensor.
#[derive(Debug, Copy, Clone)]
pub enum SensorError {
    /// Sensor is in wrong mode for update to take place.
    WrongMode,
    /// Error setting the pin
    PinError,
}

/// Sensor Mode
enum Mode {
    /// Ready to start new measurement
    Idle,
    /// Sensor has been triggered, waiting for return
    Triggered,
    /// Input pin pulled high
    MeasurePulse(Instant),
    /// Measurement is ready
    Measurement(Distance),
}

/// HC-SR04 device
pub struct HcSr04<Pin, TIM, const FREQ: u32> {
    /// Output pin to trigger sensor
    pin: Pin,
    // /// Delay to wait on for sensor trigger
    delay: Delay<TIM, FREQ>,
    /// Timer to estimate returning pulse width
    // timer: TIM,
    /// Internal mode of sensor
    mode: Mode,
}

impl<Pin, TIM, const FREQ: u32> HcSr04<Pin, TIM, FREQ>
where
    Pin: OutputPin,
    TIM: Instance,
{
    /// Create a new driver.
    ///
    /// # Arguments
    /// - `trigger` is the `OutputPin` connected to the sensor used to trigger
    /// the sensor into taking a measurement.
    /// - `delay` is a timer used to wait for the sensor to trigger.
    /// - `timer` is a timer used to estimate the pulse width of the sensor
    /// return.
    pub fn new(trigger: Pin, delay: Delay<TIM, FREQ>) -> Result<Self, Pin::Error> {
        // Ensure that our starting state is valid, if the pin was already
        // high then all internal methods would have to account for that
        // possibility, by defensively setting it low all internal states
        // can assume it is low.
        let mut trigger = trigger;

        trigger.set_low()?;
        Ok(HcSr04 {
            pin: trigger,
            delay,
            // timer: timer,
            mode: Mode::Idle,
        })
    }

    /// Trigger sensor reading and return the resulting `Distance`.
    ///
    /// This function uses [`nb::Error::WouldBlock`][1] to signal that a
    /// measurement is taking place. Once the measurement is taken, currently
    /// user responsibility of calling `update` on interrupt, the function
    /// will return the distance.
    ///
    /// [1]: https://docs.rs/nb/0.1.1/nb/enum.Error.html
    // pub fn distance(&mut self) -> nb::Result<Distance, Error<SensorError>> {
    //     match self.mode {
    //         // Start a new sensor measurement
    //         Mode::Idle => match self.trigger() {
    //             Err(_err) => Err(Error::Other(nb::Error::Other(SensorError::PinError))),
    //             Ok(()) => Err(Error::WouldBlock),
    //         },
    //         // We have triggered the sensor and are awaiting start of
    //         // return pulse
    //         Mode::Triggered => Err(Error::WouldBlock),
    //         // We have detected start of return pulse, wait for end of pulse
    //         Mode::MeasurePulse(_) => Err(Error::WouldBlock),
    //         // End of pulse detected and distance is ready
    //         Mode::Measurement(dist) => {
    //             self.mode = Mode::Idle;
    //             Ok(dist)
    //         }
    //     }
    // }

    /// Update the internal state noting that an external interrupt has
    /// occurred.
    ///
    /// This function updates the internal state in response to an external
    /// interrupt caused by the sensor. This interface will be removed once
    /// abstract interrupt handling is supported.
    ///
    /// # Return
    /// This function will return `Result::Ok` if called in the correct
    /// state. Otherwise it will return `Result::Err`.
    pub fn update(&mut self) -> Result<(), SensorError> {
        self.mode = match self.mode {
            // Mode::Triggered => Mode::MeasurePulse(self.timer.now()),
            Mode::MeasurePulse(ref start) => {
                // How many ticks have passed since we started measurement
                let ticks = start.elapsed();
                // What does these ticks mean?
                // let hz = self.timer.frequency().raw();
                // Calculation is `distance = seconds * 343.21 m/s * 0.5`
                // By doing some pre-calculations we can simply perform
                // the following to get millimeters:
                // max pulse length  = 38ms
                let distance_mm = 0; //(ticks * 171_605) / hz;
                                     // Update internal mode
                Mode::Measurement(Distance(distance_mm))
            }
            _ => return Err(SensorError::WrongMode),
        };
        Ok(())
    }

    // pub fn start_measurement(&mut self, clocks: &Clocks) -> Result<(), Pin::Error> {

    //     let timer = Timer::new(self.timer, clocks);
    //     timer.st

    // }

    // pub fn end_measurement(&mut self, clocks: &Clocks) -> Result<(), Pin::Error> {

    // }

    /// Trigger sensor starting a measurement
    pub fn trigger(&mut self) -> Result<(), Pin::Error> {
        self.pin.set_high()?;

        // let delay = self.timer.delay::<32>(clocks);

        self.delay.delay(1.micros());

        self.pin.set_low()?;

        // self.timer = delay.release().release();
        Ok(())
    }
}
