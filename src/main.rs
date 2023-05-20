#![no_std]
#![no_main]

use bit_field::BitField;
use bsp::entry;
use bsp::hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog, Clock};
use core::fmt::Write;
use core::ops::RangeInclusive;
use cortex_m::prelude::*;
use defmt::*;
use defmt_rtt as _;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, StrokeAlignment};
use embedded_graphics::{
    mono_font::{ascii::FONT_7X13, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::PwmPin;
use fugit::RateExtU32;
use heapless::String;
use panic_probe as _;
use rp2040_hal as hal;
use rp_pico as bsp;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
mod programs;
const THERMOCOUPLE_BITS: RangeInclusive<usize> = 2..=15;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    //led
    let mut led_pin = pins.led.into_push_pull_output();

    //spi
    let _spi_sclk = pins.gpio10.into_mode::<hal::gpio::FunctionSpi>();
    let mut cs_pin = pins.gpio13.into_push_pull_output();
    let _spi_miso = pins.gpio12.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<hal::gpio::FunctionSpi>();
    let spi = hal::Spi::<_, _, 8>::new(pac.SPI1);
    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    //PWM-setup
    info!("Set up PWM");
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let pwm = &mut pwm_slices.pwm7;
    pwm.default_config();
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel A on PWM7 to GPIO 14
    let channel = &mut pwm.channel_a;
    channel.output_to(pins.gpio14);
    info!("Set up I2C");

    //set up I2C for the LCD to display temps
    let sda_pin = pins.gpio6.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio7.into_mode::<hal::gpio::FunctionI2C>();
    let i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_7X13)
        .text_color(BinaryColor::On)
        .build();
    Text::with_baseline("TinyKiln!", Point::new(6, 5), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    let border_stroke = PrimitiveStyleBuilder::new()
        .stroke_color(BinaryColor::On)
        .stroke_width(3)
        .stroke_alignment(StrokeAlignment::Inside)
        .build();
    display
        .bounding_box()
        .into_styled(border_stroke)
        .draw(&mut display)
        .unwrap();
    display.flush().unwrap();

    //timers
    info!("Starting up timers");
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    //safety
    info!("Setting up check-loops");
    let switch = pins.gpio17.into_pull_up_input();

    let mut one_min = timer.get_counter() + 60000000;
    cs_pin.set_low().unwrap();
    let mut buf: [u8; 2] = [0, 0];
    delay.delay_ms(10);
    spi.transfer(&mut buf).unwrap();
    cs_pin.set_high().unwrap();
    let mut step: u8 = 1;
    let mut setpoint_c: f32 = convert_temp(buf); //set to curr temp pre-loop
    let mut reached: bool = false;
    let mut count_minutes: u8 = 0;
    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(50);
        led_pin.set_low().unwrap();
        delay.delay_ms(50);

        //get a temp reading
        cs_pin.set_low().unwrap();
        let mut buf: [u8; 2] = [0, 0];
        delay.delay_ms(10);
        spi.transfer(&mut buf).unwrap();
        cs_pin.set_high().unwrap();
        let thermocouple = convert_temp(buf);
        let mut s: String<128> = String::new();
        //check if safeties are in order
        if switch.is_high().unwrap() {
            //Switch NOK
            info!("Safety switch NOK");
            s.write_fmt(format_args!(
                "Temp: {}\nSetpt: {}\nSwitch NOK",
                thermocouple, setpoint_c
            ))
            .unwrap();
            channel.set_duty(0);
        } else {
            //switch OK

            //ok so everything's aight now, next thing we do is figuring out if we're at the C requested
            if thermocouple >= setpoint_c {
                //set reached to true
                reached = true;
            }
            //if the minute is over
            if one_min <= timer.get_counter() {
                //if we're at max_steps
                if step == programs::biscuit::get_steps() {
                    setpoint_c = 0.0;
                    //and unset reached
                    reached = false;
                    //and set step back to the max steps
                    step = programs::biscuit::get_steps();
                    s.write_fmt(format_args!(
                        "Cooldown\nTemp: {}\nSetpt: {}\nNo power",
                        thermocouple, setpoint_c
                    ))
                    .unwrap();
                } else {
                    //we check if the setpoint is reached
                    if reached {
                        //we check if this was at the end of the step
                        if setpoint_c >= programs::biscuit::get_max_temp(step) {
                            //we check if this is a stall step
                            if detect_stall(step) {
                                //because if it is, we need to count the minutes
                                count_minutes += 1;
                                //and if those minutes are higher than the duration_step
                                if count_minutes
                                    > ((programs::biscuit::get_duration_step(step) * 60.0) as u8)
                                {
                                    //we up the step
                                    step += 1;
                                    //we unset reached
                                    reached = false;
                                    //and minutes
                                    count_minutes = 0;
                                    //and we also have to set the new setpoint
                                    setpoint_c += get_cpm(step);
                                }
                            } else {
                                //we're at the end of our step, ladies and gents
                                //we up the step
                                step += 1;
                                //we unset reached
                                reached = false;
                                //and we also have to set the new setpoint
                                setpoint_c += get_cpm(step);
                            }
                        } else {
                            //then we'll up the SP
                            setpoint_c += get_cpm(step);
                            //and reset reached
                            reached = false;
                            //we should check if this setpoint is lower than the previous one, because then we're about to cool down
                            if programs::biscuit::get_max_temp(step) < setpoint_c {
                                //if it is, we'll just use that setpoint
                                setpoint_c = programs::biscuit::get_max_temp(step);
                            }
                        }
                    }
                }
                //it doesn't matter much if we didn't reach it, we'll just wait another minute
                one_min = timer.get_counter() + 60000000;
            }
            if thermocouple <= setpoint_c - 1.0 {
                if setpoint_c < 150.0 {
                    channel.set_duty(32767);
                } else {
                    channel.set_duty(65535);
                }
                //info!("full power");
                s.write_fmt(format_args!(
                    "Step: {}\nTemp: {}\nSetpt: {}\nFull power",
                    step, thermocouple, setpoint_c
                ))
                .unwrap();
            }
            if (setpoint_c - 1.0..setpoint_c).contains(&thermocouple) {
                channel.set_duty(32767);
                //info!("half power");
                s.write_fmt(format_args!(
                    "Step: {}\nTemp: {}\nSetpt: {}\nHalf power",
                    step, thermocouple, setpoint_c
                ))
                .unwrap();
            }
            if thermocouple >= setpoint_c {
                channel.set_duty(0);
                //info!("no power");
                s.write_fmt(format_args!(
                    "Step: {}\nTemp: {}\nSetpt: {}\nNo power",
                    step, thermocouple, setpoint_c
                ))
                .unwrap();
            }
            //thermocouple = setpoint_c.clone();
            println!(
                "Set {} min {} reached {} step {}",
                setpoint_c, count_minutes, reached, step
            );
        }
        display.clear();
        display
            .bounding_box()
            .into_styled(border_stroke)
            .draw(&mut display)
            .unwrap();

        Text::with_baseline(&s, Point::new(5, 5), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();
    }
}
//get the C per minute from the program
fn get_cpm(step: u8) -> f32 {
    if detect_stall(step) {
        0.0
    } else {
        (programs::biscuit::get_max_temp(step) - programs::biscuit::get_max_temp(step - 1))
            / (programs::biscuit::get_duration_step(step) * 60.0)
    }
}
//detect if there's a stall in the program (new step has the same C as the old one)
fn detect_stall(step: u8) -> bool {
    programs::biscuit::get_max_temp(step) == programs::biscuit::get_max_temp(step - 1)
}
//thermocouple stuff
fn convert_temp(buf: [u8; 2]) -> f32 {
    let raw = (buf[0] as u16) << 8 | (buf[1] as u16);
    convert(bits_to_i16(raw.get_bits(THERMOCOUPLE_BITS), 14, 4, 2))
}
fn bits_to_i16(bits: u16, len: usize, divisor: i16, shift: usize) -> i16 {
    let negative = bits.get_bit(len - 1);
    if negative {
        (bits << shift) as i16 / divisor
    } else {
        bits as i16
    }
}
fn convert(count: i16) -> f32 {
    let count = count as f32;
    count * 0.25
}
