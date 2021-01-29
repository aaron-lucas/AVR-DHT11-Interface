// ============================================================================
// Microcontroller interface to the DHT11 temperature and humidity sensor.
//
// dht11.c
//
// Created On: 2021/01/29
//
// Copyright (C) 2021  Aaron Lucas
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// aaronlucas98@gmail.com
// https://github.com/aaron-lucas
// ============================================================================

#include "dht11.h"

#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Timing thresholds for serial communication (in microseconds)
#define BIT_0_THRES_LO      26
#define BIT_0_THRES_HI      28
#define BIT_1_THRES_LO      30
#define BIT_1_THRES_HI      80

static enum data_status read_data(struct dht11 *sensor);
static uint8_t read_byte(struct dht11 *sensor);

enum pin_state {
    PIN_LOW  = 0,
    PIN_HIGH = 1
};

// Check whether the sensor's data pin matches a given state. Returns true if it
// does and false if it doesn't.
static inline bool data_pin_matches(struct dht11 *sensor, enum pin_state state) {
    // Cast to bool maps the logical AND operation to true or false rather than
    // a bit-shifted representation of the pin.
    return ((bool)state == (bool)(PINB & sensor->pin));
}

// Used for counting timer overflow interrupts for long delays. Should be reset
// to zero before each delay and need not be reset after each delay.
static volatile uint8_t overflow_count = 0;

void dht11_init(struct dht11 *sensor, uint8_t pin) {
    sensor->pin = (1 << pin);
    DDRB |= sensor->pin;
    PORTB |= sensor->pin;
    
    // Delay for 1 second to allow the sensor to stabilise
    // Use a prescaler of 16384 and wait for 4 overflows to occur which will
    // take 1/16000000 * 16384 * 4 * 256 = 1.05s
    overflow_count = 0;
    
    // Reset counter and enable interrupt on overflow
    TCNT1 = 0;
    TIMSK |= (1 << TOIE1);

    // Enable interrupts in case they are disabled
    sei();

    // Start timer and wait for four interrupt events
    TCCR1 = ((1 << CS13) | (1 << CS12) | (1 << CS11) | (1 << CS10));
    while (overflow_count < 4);
    
    // Disable counter and overflow interrupts. Global interrupts are not
    // disabled in case other interrupts are used.
    TCCR1 = 0;
    TIMSK &= ~(1 << TOIE1);
}

enum data_status dht11_update_data(struct dht11 *sensor) {
    // Send the start signal to the sensor to begin its measurement.
    // Hold the data pin low for >=18ms then wait for response.
    PORTB &= !(sensor->pin);

    // Use timer1 to delay 20ms
    //
    // Set timer1 prescaler to 2048 to enable the timer
    // At 16MHz, this counter is incremented every 128us. 156 increments
    // takes 19.968ms so start the counter at 100 and it will overflow to 0
    // after 156 increments.
    TCNT1 = 100;
    TCCR1 |= ((1 << CS13) | (1 << CS12));

    // Wait until timer overflows then stop the timer
    while (TCNT1);
    TCCR1 = 0;

    // Pull data pin high and set to input mode to await response
    PORTB |= sensor->pin;
    DDRB &= ~(sensor->pin);

    // Wait for sensor to acknowledge start signal
    while (data_pin_matches(sensor, PIN_HIGH));

    // Wait for sensor acknowledgement to complete
    while (data_pin_matches(sensor, PIN_LOW));
    while (data_pin_matches(sensor, PIN_HIGH));

    // Read bitstream from sensor
    enum data_status status = read_data(sensor);

    // Restore data pin to output and pull high
    DDRB |= sensor->pin;
    PORTB |= sensor->pin;

    return status;
}

static enum data_status read_data(struct dht11 *sensor) {
    // First four bytes are data and the 8 LSBs of their sum should match the
    // final checksum byte. Integer overflow has no effect on this.
    uint8_t sum = 0;
    for (uint8_t i = 0; i < BYTES_SENT - 1; i++) {
        uint8_t byte = read_byte(sensor);
        sensor->data.raw_data[i] = byte;
        sum += byte;
    }

    // Verify data transmission was correct
    uint8_t checksum = read_byte(sensor);
    sensor->data.bytes.checksum = checksum;
    
    if (sum == checksum) {
        return DATA_CORRECT;
    } else {
        return DATA_ERROR;
    }
}

static uint8_t read_byte(struct dht11 *sensor) {
    uint8_t mask = 0x80;    // Bits are transmitted starting with the MSB
    uint8_t byte = 0;
    do {
        // Wait for signal to go high
        while (data_pin_matches(sensor, PIN_LOW));

        // Use timer1 to count the duration for which the data line is high.
        // A prescaler value of 16 means the counter will increment every
        // microsecond
        TCNT1 = 0;
        TCCR1 |= ((1 << CS12) | (1 << CS10));

        // Wait for signal to go low then disable timer and check high-time
        while (data_pin_matches(sensor, PIN_HIGH));
        TCCR1 = 0;

        // Byte defaults to all 0 bits so only check if the bit transmitted was
        // 1. Otherwise assume it was a 0.
        if ((TCNT1 >= BIT_1_THRES_LO) && (TCNT1 <= BIT_1_THRES_HI)) {
            byte |= mask;
        }
    } while (mask >>= 1);

    return byte;
}

ISR(TIM1_OVF_vect) {
    overflow_count++;
}
