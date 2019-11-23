/* ESP-WHAT - ESP Wireless Humidity And Temperature
 *
 * Copyright (C) 2017 Daniel Collins <solemnwarning@solemnwarning.net>
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

#include "what.h"

#define DHT_GPIO 0

struct dht11_packet
{
	uint8 h_integer, h_fraction;
	uint8 t_integer, t_fraction;
	uint8 checksum;
};

int ICACHE_FLASH_ATTR sensor_wake(void)
{
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);
	gpio_output_set(0, (1 << DHT_GPIO), (1 << DHT_GPIO), 0);
	
	return 20;
}

static bool ICACHE_FLASH_ATTR _wait_for_gpio(int want_state, uint32 max_usec, uint32 *at)
{
	uint32 start = system_get_time();
	uint32 now = start;
	
	while((now - start) < max_usec)
	{
		now = system_get_time();
		
		if(GPIO_INPUT_GET(DHT_GPIO) == want_state)
		{
			if(at != NULL)
			{
				*at = now;
			}
			
			return true;
		}
	}
	
	return false;
}

#define _wait_for_low(max_usec, at)  _wait_for_gpio(0, max_usec, at)
#define _wait_for_high(max_usec, at) _wait_for_gpio(1, max_usec, at)

static bool ICACHE_FLASH_ATTR _recv_bytes(void *buf, uint32 n_bytes)
{
	for(uint32 byte = 0; byte < n_bytes; ++byte)
	{
		for(uint8 bit = 0; bit < 8; ++bit)
		{
			uint32 went_high, went_low;
			
			if(!_wait_for_low(100, NULL)
				|| !_wait_for_high(100, &went_high)
				|| !_wait_for_low(100, &went_low))
			{
				return false;
			}
			
			uint32 high_duration = (went_low - went_high);
			
			if(high_duration >= 50)
			{
				*((uint8*)(buf) + byte) |= (1 << (7 - bit));
			}
		}
	}
	
	return true;
}

#ifdef SENSOR_TYPE_DHT11
sensor_result_t ICACHE_FLASH_ATTR sensor_read(int16_t *temp, int16_t *hum)
{
	struct dht11_packet packet;
	os_memset(&packet, 0, sizeof(packet));
	
	/* Switch GPIO to input mode */
	gpio_output_set(0, 0, 0, (1 << DHT_GPIO));
	
	/* Wait for the start signal from the DHT11 */
	if(!_wait_for_low(60, NULL))
	{
		return SENSOR_TIMEOUT;
	}
	
	if(!_wait_for_high(100, NULL)
		|| !_recv_bytes(&packet, sizeof(packet))
		|| !_wait_for_high(100, NULL)
		|| _wait_for_low(1000, NULL))
	{
		return SENSOR_IOERR;
	}
	
	if((uint8)(packet.h_integer + packet.h_fraction + packet.t_integer + packet.t_fraction)
		== packet.checksum)
	{
		*temp = ((int16_t)(packet.t_integer)) * 100;
		*hum  = ((int16_t)(packet.h_integer)) * 100;
		
		return SENSOR_SUCCESS;
	}
	else{
		return SENSOR_CHECKSUM;
	}
}
#endif /* SENSOR_TYPE_DHT11 */

#ifdef SENSOR_TYPE_DHT22
sensor_result_t ICACHE_FLASH_ATTR sensor_read(int16_t *temp, int16_t *hum)
{
	struct dht11_packet packet;
	os_memset(&packet, 0, sizeof(packet));
	
	/* Switch GPIO to input mode */
	gpio_output_set(0, 0, 0, (1 << DHT_GPIO));
	
	/* Wait for the start signal from the DHT11 */
	if(!_wait_for_low(60, NULL))
	{
		return SENSOR_TIMEOUT;
	}
	
	if(!_wait_for_high(100, NULL)
		|| !_recv_bytes(&packet, sizeof(packet))
		|| !_wait_for_high(100, NULL)
		|| _wait_for_low(1000, NULL))
	{
		return SENSOR_IOERR;
	}
	
	if((uint8)(packet.h_integer + packet.h_fraction + packet.t_integer + packet.t_fraction)
		== packet.checksum)
	{
		*temp = ((((int16_t)(packet.t_integer & 0x7F)) << 8) | (int16_t)(packet.t_fraction)) * 10;
		*hum  = ((((int16_t)(packet.h_integer & 0x7F)) << 8) | (int16_t)(packet.h_fraction)) * 10;
		
		if(packet.t_integer & 0x80)
		{
			*temp *= -1;
		}
		
		return SENSOR_SUCCESS;
	}
	else{
		return SENSOR_CHECKSUM;
	}
}
#endif /* SENSOR_TYPE_DHT22 */
