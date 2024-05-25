/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se
	Copyright 2022 Jakub Tomczak

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "enc_pwm.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "hw.h"
#include "mc_interface.h"
#include "utils_math.h"
#include "timer.h"

#include <string.h>
#include <math.h>

#define PWM_TIM_PRESCALER 4
#define PWM_DUTY_MIN (16.0f / 4119.0f)
#define PWM_DUTY_MAX (4111.0f / 4119.0f)
#define PWM_DUTY_TOLERANCE (2.0f / 4119.0f)

// We configure the hardware timer like section 18.3.6 "PWM input mode" in the
// reference manual documents.
bool enc_pwm_init(PWM_config_t *cfg) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	// Enable timer clock
	HW_ENC_TIM_CLK_EN();

	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_ALTERNATE(HW_ENC_TIM_AF));

	// Time Base configuration
	// The timer's base clock is SYSTEM_CORE_CLOCK / 2.
	// This can be up to 0xFFFF. It just divides the clock for the counter itself.
	TIM_TimeBaseStructure.TIM_Prescaler = PWM_TIM_PRESCALER - 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	// This divides the clock used for the input filters in some filter modes.
	// This is divided from the base clock, not the prescaled one used for counting.
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(HW_ENC_TIM, &TIM_TimeBaseStructure);

	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	// Sampling from the divided (for input filters, not the counter prescaler)
	// clock, divided by 32, and require 8 identical samples.
	TIM_ICInitStructure.TIM_ICFilter = 15;
	TIM_PWMIConfig(HW_ENC_TIM, &TIM_ICInitStructure);

	// Now make the timer reset itself on the rising edge.
	// Note that this has to be TI2, because the indirect input selection for
	// the input capture channel itself doesn't apply. This means input capture
	// channel 1 needs to be the falling edge (configured above).
	TIM_SelectInputTrigger(HW_ENC_TIM, TIM_TS_TI2FP2);
	TIM_SelectSlaveMode(HW_ENC_TIM, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(HW_ENC_TIM, TIM_MasterSlaveMode_Enable);

	nvicEnableVector(HW_ENC_TIM_ISR_CH, 6);

	// Enable interrupt on capture
	TIM_ITConfig(HW_ENC_TIM, TIM_IT_CC2, ENABLE);

	// Enable timer
	TIM_Cmd(HW_ENC_TIM, ENABLE);

	cfg->state.pwm_error_rate = 0.0;

	return true;
}

void enc_pwm_deinit(PWM_config_t *cfg) {
	nvicDisableVector(HW_ENC_TIM_ISR_CH);
	TIM_ITConfig(HW_ENC_TIM, TIM_IT_CC2, DISABLE);
	TIM_DeInit(HW_ENC_TIM);
	cfg->state.last_enc_angle = 0.0;
	cfg->state.pwm_error_rate = 0.0;
	cfg->state.pwm_last_velocity = 0.0;
}

static float utils_calc_ratio(float low, float high, float val) {
	return (val - low) / (high - low);
}

void enc_pwm_routine(PWM_config_t *cfg) {
	uint32_t period = TIM_GetCapture2(HW_ENC_TIM);
	uint32_t on_time = TIM_GetCapture1(HW_ENC_TIM);
	float period_float = (float)period;
	float duty_cycle = (float)on_time / period_float;
	float period_seconds = period_float / (float)(SYSTEM_CORE_CLOCK / 2 / PWM_TIM_PRESCALER);
	if (duty_cycle > PWM_DUTY_MAX + PWM_DUTY_TOLERANCE ||
		duty_cycle < PWM_DUTY_MIN - PWM_DUTY_TOLERANCE) {
		++cfg->state.pwm_error_cnt;
		UTILS_LP_FAST(cfg->state.pwm_error_rate, 1.0f, period_seconds);
		cfg->state.pwm_last_period = -1;
		cfg->state.pwm_last_velocity = 0;
	} else {
		utils_truncate_number(&duty_cycle, PWM_DUTY_MIN, PWM_DUTY_MAX);
		float new_pwm_measured_enc_angle = utils_calc_ratio(PWM_DUTY_MIN, PWM_DUTY_MAX, duty_cycle) * 360.0f;
		cfg->state.last_enc_angle = new_pwm_measured_enc_angle;
		cfg->state.pwm_last_measurement_time = chVTGetSystemTime();
		if (cfg->state.pwm_last_period > 0) {
			cfg->state.pwm_last_velocity = utils_angle_difference(cfg->state.last_enc_angle, cfg->state.last_pwm_measured_enc_angle) / cfg->state.pwm_last_period;
			// The angle was measured at the beginning of the pulse, advance by the time it took us to receive it.
			cfg->state.last_enc_angle += cfg->state.pwm_last_velocity * period_seconds;
		}
		cfg->state.last_pwm_measured_enc_angle = new_pwm_measured_enc_angle;
		cfg->state.pwm_last_period = period_seconds;
		UTILS_LP_FAST(cfg->state.pwm_error_rate, 0.0f, period_seconds);
	}
}

float enc_pwm_read_deg(PWM_config_t *cfg) {
	__disable_irq();
	float pwm_last_velocity = cfg->state.pwm_last_velocity;
	systime_t pwm_last_measurement_time = cfg->state.pwm_last_measurement_time;
	__enable_irq();
	float result = cfg->state.last_enc_angle +
			(pwm_last_velocity * (float)chVTTimeElapsedSinceX(pwm_last_measurement_time) / (float)CH_CFG_ST_FREQUENCY);
	if (result < 0.0f) {
		result += 360.0f;
	}
	if (result > 360.0f) {
		result -= 360.0f;
	}
	return result;
}

float enc_pwm_get_error_rate(PWM_config_t *cfg) {
	__disable_irq();
	float pwm_error_rate = cfg->state.pwm_error_rate;
	__enable_irq();
	return pwm_error_rate;
}

float enc_pwm_time_since_reading(PWM_config_t *cfg) {
	__disable_irq();
	systime_t pwm_last_measurement_time = cfg->state.pwm_last_measurement_time;
	__enable_irq();
	return (float)chVTTimeElapsedSinceX(pwm_last_measurement_time) / (float)CH_CFG_ST_FREQUENCY;
}
