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

#include <os_type.h>
#include <stdlib.h>

#include "what.h"

/* Returns the "mode" from a list of 8-bit unsigned integers.
 *
 * If there is a tie, the return value will be one of the winners.
 * If n_numbers is zero, the return value is undefined.
*/
int16_t ICACHE_FLASH_ATTR num_mode(const int16_t *numbers, unsigned int n_numbers)
{
	int16_t mode_num, mode_count = 0;
	
	for(unsigned int i = 0; i < n_numbers; ++i)
	{
		if(mode_count > 0 && numbers[i] == mode_num)
		{
			/* Skip counting occurences of the current winning number. */
			continue;
		}
		
		unsigned int count = 0;
		
		for(unsigned int j = 0; j < n_numbers; ++j)
		{
			if(numbers[j] == numbers[i])
			{
				++count;
			}
		}
		
		if(count > mode_count)
		{
			mode_num   = numbers[i];
			mode_count = count;
		}
	}
	
	return mode_num;
}

/* Compute the "average" result from a list of sensor readings.
 *
 * The average is calculated by taking the mode of the readings, then the mean
 * of all readings that fall within MAX_SENSOR_VARIATION of the mean.
*/
int16_t ICACHE_FLASH_ATTR num_tosend(const int16_t *numbers, unsigned int n_numbers)
{
	int16_t mode = num_mode(numbers, n_numbers);
	
	int32_t sum = 0;
	unsigned int scount = 0;
	
	for(unsigned int i = 0; i < n_numbers; ++i)
	{
		if(abs((int)(numbers[i]) - mode) <= MAX_SENSOR_VARIATION)
		{
			sum += numbers[i];
			++scount;
		}
	}
	
	return (sum / scount);
}

/* Apply sensor reading correction.
 *
 * value         - Current sensor reading.
 * low_recorded  - Sensor reading at low calibraton point.
 * low_actual    - Corrected reading at low calibration point.
 * high_recorded - Sensor reading at high calibration point.
 * high_actual   - Corrected reading at high calibration point.
 *
 * Returns corrected value.
*/
int16_t num_correct(int16_t value, int16_t low_recorded, int16_t low_actual, int16_t high_recorded, int16_t high_actual)
{
	/* adj_per_x contains the averaged difference between actual and recorded values, per unit
	 * of value. The value is shifted into the MSBs of a wider type for improved precision.
	*/
	int32_t adj_per_x = ((int32_t)(high_actual - low_actual) << 16) / (high_recorded - low_recorded);
	
	/* Find which recorded value we are closest to, then take its actual value and adjust by
	 * adj_per_x to compensate for the difference between value and the recorded reference.
	*/
	if(value < (low_recorded + ((high_recorded - low_recorded) / 2)))
	{
		value = low_actual + (((int32_t)(value - low_recorded) * adj_per_x) >> 16);
	}
	else{
		value = high_actual + (((int32_t)(value - high_recorded) * adj_per_x) >> 16);
	}
	
	return value;
}
