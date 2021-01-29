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

#include <stdint.h>

// Number of bytes sent by the DHT11 sensor per transmission event
#define BYTES_SENT  5

// Indicates whether a data transfer was successful or if an error occurred.
enum data_status {
    DATA_CORRECT,
    DATA_ERROR
};

struct dht11_data {
    uint8_t temp_int;
    uint8_t temp_dec;
    uint8_t hum_int;
    uint8_t hum_dec;
    uint8_t checksum;
};

union dht11_data_u {
    uint8_t raw_data[BYTES_SENT];
    struct dht11_data bytes;
};

struct dht11 {
    uint8_t             pin;
    union dht11_data_u  data;
};

void dht11_init(struct dht11 *sensor, uint8_t pin);

enum data_status dht11_update_data(struct dht11 *sensor);
