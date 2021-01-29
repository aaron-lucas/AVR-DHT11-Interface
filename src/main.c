// ============================================================================
// Microcontroller interface to the DHT11 temperature and humidity sensor.
//
// main.c
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

#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdbool.h>
#include <stdint.h>

#include "dht11.h"

#define SENSOR_PIN    3     // Pin used for serial communication with the DHT11
#define LED_PIN       4     // Pin used for LED strip indicator

#define NUM_OVERFLOWS 62    // Overflows to count before updating the sensor
                            // Gives a delay of 1.02s

// Setup timer0 to update the sensor at a regular interval (1s)
static void setup_update_interrupt(void);

static volatile uint8_t overflow_count = 0;
static volatile bool update_flag = false;

int main(void) {
    struct dht11 sensor;
    dht11_init(&sensor, SENSOR_PIN);

    setup_update_interrupt();

    while (1) {
        if (update_flag) {
            update_flag = false;
            dht11_update_data(&sensor);
        }
    }

    return 0;
}

static void setup_update_interrupt(void) {
    cli();
    TCCR0B |= ((1 << CS02) | (1 << CS00));  // Set prescaler to 1024
    TIMSK |= (1 << TOIE0);                  // Enable overflow interrupt
    sei();
}

ISR(TIM0_OVF_vect) {
    if (overflow_count++ == NUM_OVERFLOWS) {
        update_flag = true;
        overflow_count = 0;
    }
}
