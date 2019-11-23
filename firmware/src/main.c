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

#include <ets_sys.h>
#include <gpio.h>
#include <mem.h>
#include <os_type.h>
#include <osapi.h>
#include <user_interface.h>
#include <espconn.h>

#include "what.h"

#define SV_WAIT     32767
#define SV_TIMEOUT  32766
#define SV_CHECKSUM 32765
#define SV_IOERR    32764
#define SV_SYSERR   32763

struct what_config config;

static int16_t t_readings[SENSOR_SAMPLES];
static int16_t h_readings[SENSOR_SAMPLES];

static struct espconn conn;

static void _read_sensor_begin(void *arg);
static void _read_sensor_finish(void *arg);

static void _tx_data(void *arg);

static void ICACHE_FLASH_ATTR wifi_init()
{
	wifi_set_opmode(STATION_MODE);
	
	struct station_config stationConf;
	
	stationConf.bssid_set = 0;
	os_memcpy(&stationConf.ssid,     config.wifi_ssid,     sizeof(config.wifi_ssid));
	os_memcpy(&stationConf.password, config.wifi_password, sizeof(config.wifi_password));
	
	wifi_station_set_config(&stationConf);
	
	wifi_station_connect();
}

static void ICACHE_FLASH_ATTR _read_sensor_begin(void *arg)
{
	int delay_ms = sensor_wake();
	
	if(delay_ms == 0)
	{
		_read_sensor_finish(NULL);
	}
	else{
		static os_timer_t timer;
		
		os_timer_setfn(&timer, &_read_sensor_finish, NULL);
		os_timer_arm(&timer, delay_ms, false);
	}
}

static void ICACHE_FLASH_ATTR _read_sensor_finish(void *arg)
{
	static unsigned int sslot = 0;
	
	os_printf("Querying sensor... ");
	
	int16_t t_read, h_read;
	sensor_result_t r = sensor_read(&t_read, &h_read);
	
	switch(r)
	{
		case SENSOR_TIMEOUT:
			os_printf("TIMEOUT\n");
			
			t_readings[sslot] = h_readings[sslot] = SV_TIMEOUT;
			break;
		
		case SENSOR_CHECKSUM:
			os_printf("CHECKSUM FAILURE\n");
			
			t_readings[sslot] = h_readings[sslot] = SV_CHECKSUM;
			break;
		
		case SENSOR_IOERR:
			os_printf("I/O ERROR\n");
			
			t_readings[sslot] = h_readings[sslot] = SV_IOERR;
			break;
			
		case SENSOR_SYSERR:
			os_printf("SYSTEM ERROR\n");
			
			t_readings[sslot] = h_readings[sslot] = SV_SYSERR;
			break;
			
		default:
			if(config.temp_low_recorded != config.temp_high_recorded)
			{
				t_readings[sslot] = num_correct(t_read,
					config.temp_low_recorded,  config.temp_low_corrected,
					config.temp_high_recorded, config.temp_high_corrected);
				
				os_printf("%u.%02u°C (corrected to %u.%02u°C)",
					(unsigned)(t_read / 100),            (unsigned)(t_read % 100),
					(unsigned)(t_readings[sslot] / 100), (unsigned)(t_readings[sslot] % 100));
			}
			else{
				t_readings[sslot] = t_read;
				os_printf("%u.%02u°C", (unsigned)(t_read / 100), (unsigned)(t_read % 100));
			}
			
			if(config.hum_low_recorded != config.hum_high_recorded)
			{
				h_readings[sslot] = num_correct(h_read,
					config.hum_low_recorded,  config.hum_low_corrected,
					config.hum_high_recorded, config.hum_high_corrected);
				
				os_printf(" %u.%02u%% (corrected to %u.%02u%%)\n",
					(unsigned)(h_read / 100),            (unsigned)(h_read % 100),
					(unsigned)(h_readings[sslot] / 100), (unsigned)(h_readings[sslot] % 100));
			}
			else{
				h_readings[sslot] = h_read;
				os_printf(" %u.%02u%%\n", (unsigned)(h_read / 100), (unsigned)(h_read % 100));
			}
			
			break;
	}
	
	sslot = (sslot + 1) % SENSOR_SAMPLES;
}

static void ICACHE_FLASH_ATTR _tx_data(void *arg)
{
	int16_t t_value = num_tosend(t_readings, SENSOR_SAMPLES);
	int16_t h_value = num_tosend(h_readings, SENSOR_SAMPLES);
	
	os_printf("Transmitting sensor data... ");
	
	/* Check just the temperature value for error states. This is fine for
	 * now, but will need changing if we ever support sensors which are not
	 * combined temperature and humidity.
	*/
	
	switch(t_value)
	{
		case SV_WAIT:
			os_printf("Not enough data (skipping)\n");
			break;
			
		case SV_TIMEOUT:
			os_printf("Sensor timeout (skipping)\n");
			break;
			
		case SV_CHECKSUM:
			os_printf("Checksum error (skipping)\n");
			break;
			
		case SV_IOERR:
			os_printf("I/O error (skipping)\n");
			break;
			
		case SV_SYSERR:
			os_printf("System error (skipping)\n");
			break;
			
		default:
		{
			os_printf("%u.%02u°C %u.%02u%%\n",
				(unsigned)(t_value / 100), (unsigned)(t_value % 100),
				(unsigned)(h_value / 100), (unsigned)(h_value % 100));
			
			struct what_packet packet;
			
			memcpy(packet.magic, "ESP-WHAT", 8);
			memcpy(packet.sensor_id, config.sensor_id, 8);
			
			packet.temperature = t_value;
			packet.humidity    = h_value;
			
			espconn_send(&conn, (void*)(&packet), sizeof(packet));
			
			break;
		}
	};
}

void ICACHE_FLASH_ATTR user_init()
{
	/* The flash is mapped into our virtual address space, but accesses must be 32-bit aligned,
	 * so we copy the whole config from it into RAM here.
	*/
	config = *(const struct what_config*)(FLASH_ADDRESS + CONFIG_OFFSET);
	
	if(config.tcp_debug_port > 0)
	{
		/* Redirect debug console output to a TCP server. */
		debug_init(config.tcp_debug_port);
	}
	else{
		/* Set serial baud rate. */
		uart_div_modify(0, UART_CLK_FREQ / 115200);
	}
	
	os_printf("Starting up...\n");
	
	gpio_init();
	
	static os_timer_t poll_timer;
	static os_timer_t report_timer;
	
	for(unsigned i = 0; i < SENSOR_SAMPLES; ++i)
	{
		t_readings[i] = SV_WAIT;
		h_readings[i] = SV_WAIT;
	}
	
	uint8_t z8[8] = { 0,0,0,0,0,0,0,0 };
	if(memcmp(config.sensor_id, z8, 8) == 0)
	{
		/* If sensor_id is zero, use the our MAC. */
		wifi_get_macaddr(STATION_IF, config.sensor_id);
	}
	
	os_printf("Sensor ID: %02x%02x%02x%02x%02x%02x%02x%02x\n",
		(unsigned)(config.sensor_id[0]),
		(unsigned)(config.sensor_id[1]),
		(unsigned)(config.sensor_id[2]),
		(unsigned)(config.sensor_id[3]),
		(unsigned)(config.sensor_id[4]),
		(unsigned)(config.sensor_id[5]),
		(unsigned)(config.sensor_id[6]),
		(unsigned)(config.sensor_id[7]));
	
	os_printf("Initialising networking...\n");
	
	wifi_init();
	
	conn.type = ESPCONN_UDP;
	conn.proto.udp = (esp_udp*)(os_zalloc(sizeof(esp_udp)));
	
	espconn_create(&conn);
	
	os_memcpy(conn.proto.udp->remote_ip, &(config.data_ip), 4);
	conn.proto.udp->remote_port = config.data_port;
	conn.proto.udp->local_port = espconn_port();
	
	os_timer_setfn(&poll_timer, &_read_sensor_begin, NULL);
	os_timer_arm(&poll_timer, SENSOR_INTERVAL_MS, true);
	
	os_timer_setfn(&report_timer, &_tx_data, NULL);
	os_timer_arm(&report_timer, REPORT_INTERVAL_MS, true);
}
