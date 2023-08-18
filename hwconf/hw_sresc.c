/*
	Copyright 2023 Brian Silverman	brian@seemsreasonable.net

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "drv8320s.h"
#include "utils.h"
#include <math.h>
#include "mc_interface.h"

void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// LEDs
	palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// High-side gate command outputs.
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Low-side gate command outputs.
	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// Phase filters
	palSetPadMode(PHASE_FILTER_GPIO, PHASE_FILTER_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	PHASE_FILTER_OFF();

	// Current filter
	palSetPadMode(GPIOD, 2,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	CURRENT_FILTER_OFF();

	// Phase voltage sense. Channels 0-2 on all 3 ADCs.
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	// Phase current sense. Channels 10-12 on all 3 ADCs.
	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);

	// A FET temperature monitor. Channel 3 on all 3 ADCs.
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	// B FET temperature monitor. Channel 8 on ADC1 and ADC2.
	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);
	// C FET temperature monitor. Channel 9 on ADC1 and ADC2.
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);

	// Battery voltage sense. Channel 13 on all 3 ADCs.
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);

	// Unconnected pins, pull up to increase EMI tolerance.
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOC, 13, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOC, 14, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOC, 15, PAL_MODE_INPUT_PULLUP);

	// Fault pin. This defaults to JTDI, which is fine because it's not going to
	// be driven before we ENABLE_GATE.
	palSetPadMode(GPIOA, 15, PAL_MODE_INPUT_PULLUP);

	// ENABLE_GATE
	palSetPadMode(GPIOC, 12,
				  PAL_MODE_OUTPUT_PUSHPULL |
				  PAL_STM32_OSPEED_HIGHEST);
	ENABLE_GATE();

	drv8320s_init();
}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels
	// A voltage (SENS1)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
	// A current (CURR1)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	// Internal voltage reference.
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 3, ADC_SampleTime_15Cycles);
	// B FET temperature (TEMP_MOS_2)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 4, ADC_SampleTime_15Cycles);

	// ADC2 regular channels
	// B voltage (SENS2)
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);
	// B current (CURR2)
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	// C FET temperature (TEMP_MOS_3)
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 3, ADC_SampleTime_15Cycles);
	// A voltage (SENS1) (unused sample)
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 4, ADC_SampleTime_15Cycles);

	// ADC3 regular channels
	// C voltage (SENS3)
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);
	// C current (CURR2)
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	// A FET temperature (TEMP_MOS)
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, ADC_SampleTime_15Cycles);
	// Battery voltage (AN_IN)
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_15Cycles);

	// Injected channels (for mcpwm only)
	// A current (CURR1)
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
	// B current (CURR2)
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
	// C current (CURR2)
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
	// A current (CURR1)
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	// B current (CURR2)
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	// C current (CURR2)
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	// A current (CURR1)
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_15Cycles);
	// B current (CURR2)
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, ADC_SampleTime_15Cycles);
	// C current (CURR2)
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_15Cycles);
}

float hw_sresc_get_temp(void) {
	float t1 = NTC_TEMP_MOS1();
	float t2 = NTC_TEMP_MOS2();
	float t3 = NTC_TEMP_MOS3();
	float res = 0.0;

	if (t1 > t2 && t1 > t3) {
		res = t1;
	} else if (t2 > t1 && t2 > t3) {
		res = t2;
	} else {
		res = t3;
	}

	return res;
}

void hw_stop_i2c(void) {}
