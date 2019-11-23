/* ESP-WHAT - ESP Wireless Humidity And Temperature
 *
 * Copyright (C) 2017-2019 Daniel Collins <solemnwarning@solemnwarning.net>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the author nor the names of its contributors may
 *     be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef WHAT_H
#define WHAT_H

#include <os_type.h>

/* Interval to poll the sensors in milliseconds.
 * DHT11 datasheet: "Sampling period at intervals should be no less than 1 second"
*/
#define SENSOR_INTERVAL_MS 4000

/* Report data every x milliseconds. */
#define REPORT_INTERVAL_MS 30000

/* Number of samples to average reading across.
 * Too large will hide short spikes, too small will reduce accuracy.
*/
#define SENSOR_SAMPLES 32

/* Maximum allowed variation from the mode */
#define MAX_SENSOR_VARIATION 500

#define FLASH_ADDRESS 0x40200000
#define CONFIG_OFFSET 0x7A000

/* ESP-WHAT configuration structure.
 * Configuration should be written to flash at CONFIG_OFFSET during programming.
*/
struct what_config
{
	char wifi_ssid[32];
	char wifi_password[32];
	
	uint8_t sensor_id[8];
	
	uint32_t data_ip;
	uint16_t data_port;
	
	uint16_t tcp_debug_port;
	
	int16_t temp_low_recorded, temp_low_corrected;
	int16_t temp_high_recorded, temp_high_corrected;

	int16_t hum_low_recorded, hum_low_corrected;
	int16_t hum_high_recorded, hum_high_corrected;
} __attribute__((packed));

struct what_packet {
	char magic[8];         /* "ESP-WHAT" (no terminator) */
	uint8_t sensor_id[8];  /* Sensor ID, usually the Wi-Fi MAC followed by zeros. */
	
	int16_t temperature;   /* Temperature in hundredths of a degree celcius. */
	int16_t humidity;      /* Humidity in hundredths of a percent. */
} __attribute__((packed));

extern struct what_config config; /* main.c */

void debug_init(uint16_t port);

typedef enum {
	SENSOR_SUCCESS,
	SENSOR_TIMEOUT,
	SENSOR_CHECKSUM,
	SENSOR_IOERR,
	SENSOR_SYSERR,
} sensor_result_t;

int sensor_wake(void);
sensor_result_t ICACHE_FLASH_ATTR sensor_read(int16_t *temp, int16_t *hum);

int16_t num_mode(const int16_t *numbers, unsigned int n_numbers);
int16_t num_tosend(const int16_t *numbers, unsigned int n_numbers);
int16_t num_correct(int16_t value, int16_t low_recorded, int16_t low_actual, int16_t high_recorded, int16_t high_actual);

#endif /* !WHAT_H */
