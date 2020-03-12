// The MIT License (MIT)
//
// Copyright (c) 2017 duk.io
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//
// ----------------------------------------------------------------------------
//
// 12-Aug-2017
//
// Control firmware for automatic water level maintainer.
//
// Checks water level periodically. If low, will wait and check again to
// ensure level is not low due to periodic waterfall pumping. If still low,
// water valve is opened for a short period to add water. The valve open
// duration is set via a potentiometer.
//
// If water level is consistently measured low too many times, alarm is
// triggered and execution halts.
//
// Target
// ------
// ATTiny85
//
// GPIO
// ----
// All GPIO is setup on port B
// - OUT BUZZER_PIN      : Buzzer output. High = buzzzzz
// - OUT SOLENOID_PIN    : Water control solenoid. High = water flowing
// - OUT LED_GREEN_PIN   : Green status LED. Low = on
// - OUT LED_PIN_RED     : Red status LED. Low = on
// - IN  SENSOR_PIN      : Water level sensor is connected between this pin and
//                       : ground. Open circuit on water level low, short to
//                       : ground on water level okay.
//                       : pullup)
// - IN  POT_ADC_CHANNEL : Water fill duration input.
//                       : Duration = Vpin/Vcc * max_water_on_time_s

#include<stdbool.h>
#include<stdint.h>
#include<avr/io.h>
#include<avr/delay.h>

static void setupIo(void);
static void checkAndFill(void);
static bool isLevelLow(void);
static uint16_t getFillTime(void);
static void fill(void);
static uint8_t readAdc(uint8_t channel);
static void haltOnError(void);
static void ledFlashFast(uint8_t port, uint8_t n);
static void ledFlashSlow(uint8_t port, uint8_t n);
static void beep(uint8_t secs);
static void delayS(uint16_t secs);
/// Must be pin on port B
static void ledOn(uint8_t port);
/// Must be pin on port B
static void ledOff(uint8_t port);

// ** Note: All input/outputs must be PORTB **

#define BUZZER_PIN PB5
#define SENSOR_PIN PB4
#define SOLENOID_PIN PB2
#define LED_GREEN_PIN PB1
#define LED_PIN_RED PB0
#define POT_ADC_CHANNEL 3

// **

static const uint16_t check_interval_s = 30 * 60;
static const uint16_t dbl_confirm_interval_s = 90 * 60;
static const uint16_t max_water_on_time_s = 90;
static const uint8_t max_consec_files = 5;

int main(void)
{
    setupIo();
    while(1) {
        checkAndFill();
        delayS(check_interval_s);
    }
    return 0;
}

static void setupIo(void)
{
    // Setup inputs and outputs
    DDRB |= (1<<SOLENOID_PIN) | (1<<LED_GREEN_PIN) | (1<<LED_PIN_RED) | (1<<BUZZER_PIN);
    DDRB &= ~(1<<SENSOR_PIN);
    ledOff(LED_PIN_RED);
    ledOff(LED_GREEN_PIN);

    // Setup ADC
    ADMUX =
            // Vcc ref.
            (0<<REFS1) | (0<<REFS0) |
            // left shift result
            (1 << ADLAR);
    ADCSRA |=
            // 125 kHz sample rate (oscillator=8MHz, clock=1MHz, prescaler=8)
            (1<<ADPS1) | (1<<ADPS0) |
            // enable
            (1<<ADEN);
}

static void checkAndFill(void)
{
    static uint8_t consecutive_fills = 0;

    ledFlashSlow(LED_GREEN_PIN, 1);

    // If level is low, wait and check again in
    // case this is just the waterfall running.
    if (isLevelLow()) {
        ledFlashFast(LED_GREEN_PIN, 1);
        delayS(dbl_confirm_interval_s);
        ledFlashSlow(LED_GREEN_PIN, 2);
        if (isLevelLow()) {
            if (consecutive_fills >= max_consec_files) {
                haltOnError();
            }
            fill();
            consecutive_fills++;
        }
        else {
            consecutive_fills = 0;
            ledFlashFast(LED_GREEN_PIN, 3);
        }
    }
    else {
        consecutive_fills = 0;
        ledFlashFast(LED_GREEN_PIN, 3);
    }
}

static bool isLevelLow(void)
{
    // Enable pull up only for measurement to reduce sensor wear
    PORTB |= (1<<SENSOR_PIN);
    _delay_ms(1);
    // Float switch is open circuit if water is
    // low, short to ground if water is high.
    bool sensorVal = PINB & (1<<SENSOR_PIN);
    PORTB &= ~(1<<SENSOR_PIN);
    return sensorVal;
}

static uint16_t getFillTime(void)
{
    uint8_t potLevel = readAdc(POT_ADC_CHANNEL);
    return max_water_on_time_s * potLevel / 255;
}

static void fill(void)
{
    PORTB |= (1<<SOLENOID_PIN);
    ledOn(LED_GREEN_PIN);
    delayS(getFillTime());
    PORTB &= ~(1<<SOLENOID_PIN);
    ledOff(LED_GREEN_PIN);
}

// 8-bit read, valid channel = 0 -> 3
static uint8_t readAdc(uint8_t channel)
{
    ADMUX &= 0xf0;
    ADMUX |= channel;

    // Start conversion and wait for result
    ADCSRA |= (1<<ADSC);
    while ( (ADCSRA & (1<<ADSC)) );
    return ADCH;
}

static void haltOnError(void)
{
    ledOn(LED_PIN_RED);
    while(1) {
        delayS(60 * 60);
        beep(1);
    }
}

static void beep(uint8_t secs)
{
    for (int i = 0; i < (secs * 250); i++) {
        PORTB |= (1<<BUZZER_PIN);
        _delay_ms(2);
        PORTB &= ~(1<<BUZZER_PIN);
        _delay_ms(2);
    }
}

static void ledFlashFast(uint8_t pin, uint8_t n)
{
    for (int i = 0; i < n; i++) {
        ledOn(pin);
        _delay_ms(300);
        ledOff(pin);
        _delay_ms(300);
    }
}

static void ledFlashSlow(uint8_t pin, uint8_t n)
{
    for (int i = 0; i < n; i++) {
        ledOn(pin);
        _delay_ms(1000);
        ledOff(pin);
        _delay_ms(1000);
    }
}

static void delayS(uint16_t secs)
{
    for (int i = 0; i < secs; i++) {
        _delay_ms(1000);
    }
}

static void ledOn(uint8_t pin)
{
    PORTB &= ~(1<<pin);
}

static void ledOff(uint8_t pin)
{
    PORTB |= (1<<pin);
}
