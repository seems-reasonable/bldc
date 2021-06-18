/*
 * Copyright 2019 Benjamin Vedder	benjamin@vedder.se
 *
 * This file is part of the VESC firmware.
 *
 * The VESC firmware is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   The VESC firmware is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *   */

#include "app.h"
#include "ch.h"
#include "hal.h"

#include "comm_can.h"
#include "commands.h"
#include "encoder.h"
#include "hw.h"
#include "mc_interface.h"
#include "terminal.h"
#include "timeout.h"
#include "utils.h"
#include "servo_dec.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define MIN_PULSES_WITHOUT_POWER		50

// Threads
static THD_FUNCTION(flipper_solenoid_thread, arg);
static THD_WORKING_AREA(flipper_solenoid_thread_wa, 2048);
static volatile bool ppm_rx = false;

// Private functions
static void servodec_func(void);

// Will be called after each output PWM cycle, in the interrupt.
static void control_callback(void);

// Terminal command implementations.
static void terminal_solenoid_pulse(int argc, const char **argv);
static void terminal_solenoid_plot(int argc, const char **argv);

// Private variables
static volatile bool is_running = false;
static volatile bool stop_now = true;
static volatile ppm_config config;
static volatile int pulses_without_power = 0;

// Coordination with the interrupt for sampling motor parameters
static volatile float avg_current_tot = 0.0;
static volatile float avg_voltage_tot = 0.0;
static volatile unsigned int sample_num = 0;

static bool do_plot = false;
static float plot_sample;
static MUTEX_DECL(plot_mutex);

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
  mc_interface_set_pwm_callback(control_callback);

  stop_now = false;
  do_plot = false;
  chThdCreateStatic(flipper_solenoid_thread_wa,
                    sizeof(flipper_solenoid_thread_wa), NORMALPRIO,
                    flipper_solenoid_thread, NULL);

  // Terminal commands for the VESC Tool terminal can be registered.
  terminal_register_command_callback("solenoid_pulse", "Pulse a solenoid",
                                     "[current_A] [time_S]",
                                     terminal_solenoid_pulse);
  terminal_register_command_callback("solenoid_plot", "Enable solenoid plotting",
                                     "[enable]", terminal_solenoid_plot);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
//
// Note that is called even if the custom application was not previously
// configured.
void app_custom_stop(void) {
  mc_interface_set_pwm_callback(0);
  terminal_unregister_callback(terminal_solenoid_pulse);
  terminal_unregister_callback(terminal_solenoid_plot);

  stop_now = true;
  while (is_running) {
    chThdSleepMilliseconds(1);
  }
}

void app_custom_configure(app_configuration *conf) {
	config = conf->app_ppm_conf;
	pulses_without_power = 0;
}

static void servodec_func(void) {
	ppm_rx = true;
	chSysLockFromISR();
	//chEvtSignalI(ppm_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

static THD_FUNCTION(flipper_solenoid_thread, arg) {
  (void)arg;

  chRegSetThreadName("App Flipper Solenoid");

  servodec_set_pulse_options(config.pulse_start, config.pulse_end, config.median_filter);
  servodec_init(servodec_func);
  is_running = true;

  for (;;) {
    // Check if it is time to stop.
    if (stop_now) {
      is_running = false;
      return;
    }

    chMtxLock(&plot_mutex);
    if (do_plot) {
      commands_plot_set_graph(0);
      commands_send_plot_points(
          plot_sample, mc_interface_get_tot_current());
      const float duty_cycle = mc_interface_get_duty_cycle_now();
      commands_plot_set_graph(1);
      commands_send_plot_points(plot_sample, duty_cycle);
      commands_plot_set_graph(2);
      commands_send_plot_points(
          plot_sample, duty_cycle * GET_INPUT_VOLTAGE());
      ++plot_sample;
    }
    chMtxUnlock(&plot_mutex);

	// Reset the timeout if we got a pulse since last time through.
	if (ppm_rx) {
		ppm_rx = false;
		timeout_reset();
	}

	float servo_val = servodec_get_servo(0);
	float servo_ms = utils_map(servo_val, -1.0, 1.0, config.pulse_start, config.pulse_end);
	static bool servoError = false;

	// Mapping with respect to center pulsewidth
	if (servo_ms < config.pulse_center) {
		servo_val = utils_map(servo_ms, config.pulse_start,
				config.pulse_center, -1.0, 0.0);
	} else {
		servo_val = utils_map(servo_ms, config.pulse_center,
				config.pulse_end, 0.0, 1.0);
	}

	if (servodec_get_time_since_update() > timeout_get_timeout_msec()) {
		pulses_without_power = 0;
		servoError = true;
	} else if (mc_interface_get_fault() != FAULT_CODE_NONE){
		pulses_without_power = 0;
	}

	// Apply deadband
	utils_deadband(&servo_val, config.hyst, 1.0);

	// Apply throttle curve
	servo_val = utils_throttle_curve(servo_val, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

	bool fire_now = servo_val > 0.5;

	if (fabsf(servo_val) < 0.001) {
		pulses_without_power++;
	}

	//Safe start : If startup, servo timeout or fault, check if idle has been verified for some pulses before driving the motor
	if (pulses_without_power < MIN_PULSES_WITHOUT_POWER && config.safe_start) {
		if (servoError) {
			continue;
		}
		fire_now = false;
	} else {
		servoError = false;
		pulses_without_power = MIN_PULSES_WITHOUT_POWER;
	}

	if (fire_now) {
		float current = 90;
		float time = 0.1;
        for (float t = 0.0; t < time; t += 0.002) {
          timeout_reset();
          mc_interface_set_current(current);
          chThdSleepMilliseconds(2);
        }

        mc_interface_set_current(0);
		pulses_without_power = 0;
	}

   chThdSleepMicroseconds(1000);
 }
}

static void control_callback(void) {
	avg_current_tot += mc_interface_get_tot_current();
	avg_voltage_tot += mc_interface_get_duty_cycle_now() * GET_INPUT_VOLTAGE();
	sample_num++;
}

static void start_plot(void) {
  chMtxLock(&plot_mutex);
  do_plot = true;
  plot_sample = 0;
  commands_init_plot("Sample", "");
  commands_plot_add_graph("Current");
  commands_plot_add_graph("Duty cycle");
  commands_plot_add_graph("Effective voltage");
  chMtxUnlock(&plot_mutex);
}

static void stop_plot(void) {
  chMtxLock(&plot_mutex);
  do_plot = false;
  chMtxUnlock(&plot_mutex);
}

#define CLI_MAX_CURRENT 220
#define CLI_MAX_TIME 0.25

static void terminal_solenoid_pulse(int argc, const char **argv) {
  if (argc == 3 || argc == 4 ) {
    bool pulse_plot = argc == 4 && strcmp(argv[3], "plot") == 0;

    float current = -1.0;
    float time = -1.0;
    sscanf(argv[1], "%f", &current);
    sscanf(argv[2], "%f", &time);

    commands_printf("Pulsing solenoid at %f A for %f seconds%s",
        (double)current, (double)time,
        pulse_plot ? " with plotting" : "");

    // Based on rotor_lock_openloop.
    if (current > 0.0 && time > 0.0 &&
        current <= mc_interface_get_configuration()->l_current_max &&
		current <= CLI_MAX_CURRENT * 1.01 && time < CLI_MAX_TIME * 1.01) {
      if (pulse_plot) {
        start_plot();
        chThdSleepMilliseconds(5);
      }
      if (time <= 1e-6) {
        timeout_reset();
        mc_interface_set_current(current);
        commands_printf("OK\n");
      } else {
        int print_div = 0;
        for (float t = 0.0; t < time; t += 0.002) {
          timeout_reset();
          mc_interface_set_current(current);
          chThdSleepMilliseconds(2);

          print_div++;
          if (print_div >= 200) {
            print_div = 0;
            commands_printf("T left: %.2f s", (double)(time - t));
          }
        }

        mc_interface_set_current(0);
        if (pulse_plot) {
          chThdSleepMilliseconds(20);
          stop_plot();
        }
        commands_printf("Done\n");
      }
    } else {
      commands_printf("Invalid argument(s).\n");
    }
  } else {
    commands_printf("This command requires two arguments.\n");
  }
}

static void terminal_solenoid_plot(int argc, const char **argv) {
  if (argc == 2) {
    if (argv[1][0] == '1') {
      start_plot();
      commands_printf("Solenoid plotting enabled.\n");
    } else {
      stop_plot();
      commands_printf("Solenoid plotting disabled.\n");
    }
  } else {
    commands_printf("This command requires one argument.\n");
  }
}

/**
 * Lock the solenoid with a current and sample the voltage and current to
 * calculate the solenoid resistance.
 *
 * @param current
 * The locking current.
 *
 * @param samples
 * The number of samples to take.
 *
 * @param stop_after
 * Stop motor after finishing the measurement. Otherwise, the current will
 * still be applied after returning. Setting this to false is useful if you want
 * to run this function again right away, without stopping the motor in between.
 *
 * @return
 * The calculated motor resistance.
 */
static float solenoid_measure_resistance(float current, int milliseconds, bool stop_after) {
	mc_interface_lock();

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	timeout_reset();
	timeout_configure(60000, 0.0);

	// Start ramping up current.
	mc_interface_set_current(current);
	// Wait for the current to rise and the solenoid to actuate.
	chThdSleepMilliseconds(40);

	// Sample
	avg_current_tot = 0.0;
	avg_voltage_tot = 0.0;
	sample_num = 0;

	int cnt = 0;
	while (sample_num < milliseconds) {
		chThdSleepMilliseconds(1);
		cnt++;
		// Timeout
		if (cnt > 10000) {
			break;
		}

		if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			mc_interface_set_current(0);

			timeout_configure(tout, tout_c);
			mc_interface_unlock();

			return 0.0;
		}
	}

	const float current_avg = avg_current_tot / (float)sample_num;
	const float voltage_avg = avg_voltage_tot / (float)sample_num;

	// Stop
	if (stop_after) {
		mc_interface_set_current(0);
	}

	// Enable timeout
	timeout_configure(tout, tout_c);
	mc_interface_unlock();

	return voltage_avg / current_avg;
}

// drive sin waves, after a few start sampling, take a few periods, then use
// filter_fft on the result
// Watch out for ramp step limit driving the sin waves
/**
 * Measure the motor inductance with short voltage pulses.
 *
 * @param duty
 * The duty cycle to use in the pulses.
 *
 * @param samples
 * The number of samples to average over.
 *
 * @param
 * The current that was used for this measurement.
 *
 * @return
 * The average d and q axis inductance in uH.
 */
static float solenoid_measure_inductance(float duty, int samples, float *curr, float *ld_lq_diff) {
	volatile motor_all_state_t *motor = motor_now();

	mc_sensor_mode sensor_mode_old = motor->m_conf->sensor_mode;
	float f_sw_old = motor->m_conf->foc_f_sw;
	float hfi_voltage_start_old = motor->m_conf->foc_hfi_voltage_start;
	float hfi_voltage_run_old = motor->m_conf->foc_hfi_voltage_run;
	float hfi_voltage_max_old = motor->m_conf->foc_hfi_voltage_max;
	bool sample_v0_v7_old = motor->m_conf->foc_sample_v0_v7;
	foc_hfi_samples samples_old = motor->m_conf->foc_hfi_samples;
	bool sample_high_current_old = motor->m_conf->foc_sample_high_current;

	mc_interface_lock();
	motor->m_control_mode = CONTROL_MODE_NONE;
	motor->m_state = MC_STATE_OFF;
	stop_pwm_hw(motor);

	motor->m_conf->foc_sensor_mode = FOC_SENSOR_MODE_HFI;
	motor->m_conf->foc_hfi_voltage_start = duty * GET_INPUT_VOLTAGE() * (2.0 / 3.0);
	motor->m_conf->foc_hfi_voltage_run = duty * GET_INPUT_VOLTAGE() * (2.0 / 3.0);
	motor->m_conf->foc_hfi_voltage_max = duty * GET_INPUT_VOLTAGE() * (2.0 / 3.0);
	motor->m_conf->foc_sample_v0_v7 = false;
	motor->m_conf->foc_hfi_samples = HFI_SAMPLES_32;
	motor->m_conf->foc_sample_high_current = false;

	update_hfi_samples(motor->m_conf->foc_hfi_samples, motor);

	chThdSleepMilliseconds(1);

	timeout_reset();
	mcpwm_foc_set_duty(0.0);
	chThdSleepMilliseconds(1);

	int ready_cnt = 0;
	while (!motor->m_hfi.ready) {
		chThdSleepMilliseconds(1);
		ready_cnt++;
		if (ready_cnt > 100) {
			break;
		}
	}

	if (samples < 10) {
		samples = 10;
	}

	float l_sum = 0.0;
	float ld_lq_diff_sum = 0.0;
	float i_sum = 0.0;
	float iterations = 0.0;

	for (int i = 0;i < (samples / 10);i++) {
		if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			motor->m_id_set = 0.0;
			motor->m_iq_set = 0.0;
			motor->m_control_mode = CONTROL_MODE_NONE;
			motor->m_state = MC_STATE_OFF;
			stop_pwm_hw(motor);

			motor->m_conf->foc_sensor_mode = sensor_mode_old;
			motor->m_conf->foc_f_sw = f_sw_old;
			motor->m_conf->foc_hfi_voltage_start = hfi_voltage_start_old;
			motor->m_conf->foc_hfi_voltage_run = hfi_voltage_run_old;
			motor->m_conf->foc_hfi_voltage_max = hfi_voltage_max_old;
			motor->m_conf->foc_sample_v0_v7 = sample_v0_v7_old;
			motor->m_conf->foc_hfi_samples = samples_old;
			motor->m_conf->foc_sample_high_current = sample_high_current_old;

			update_hfi_samples(motor->m_conf->foc_hfi_samples, motor);

			mc_interface_unlock();

			return 0.0;
		}

		timeout_reset();
		mcpwm_foc_set_duty(0.0);
		chThdSleepMilliseconds(10);

		float real_bin0, imag_bin0;
		float real_bin2, imag_bin2;
		float real_bin0_i, imag_bin0_i;

		motor->m_hfi.fft_bin0_func((float*)motor->m_hfi.buffer, &real_bin0, &imag_bin0);
		motor->m_hfi.fft_bin2_func((float*)motor->m_hfi.buffer, &real_bin2, &imag_bin2);
		motor->m_hfi.fft_bin0_func((float*)motor->m_hfi.buffer_current, &real_bin0_i, &imag_bin0_i);

		l_sum += real_bin0;
		ld_lq_diff_sum += 2.0 * sqrtf(SQ(real_bin2) + SQ(imag_bin2));
		i_sum += real_bin0_i;

		iterations++;
	}

	mcpwm_foc_set_current(0.0);

	motor->m_conf->foc_sensor_mode = sensor_mode_old;
	motor->m_conf->foc_f_sw = f_sw_old;
	motor->m_conf->foc_hfi_voltage_start = hfi_voltage_start_old;
	motor->m_conf->foc_hfi_voltage_run = hfi_voltage_run_old;
	motor->m_conf->foc_hfi_voltage_max = hfi_voltage_max_old;
	motor->m_conf->foc_sample_v0_v7 = sample_v0_v7_old;
	motor->m_conf->foc_hfi_samples = samples_old;
	motor->m_conf->foc_sample_high_current = sample_high_current_old;

	update_hfi_samples(motor->m_conf->foc_hfi_samples, motor);

	mc_interface_unlock();

	if (curr) {
		*curr = i_sum / iterations;
	}

	if (ld_lq_diff) {
		*ld_lq_diff = (ld_lq_diff_sum / iterations) * 1e6 * (2.0 / 3.0);
	}

	return (l_sum / iterations) * 1e6 * (2.0 / 3.0);
}

/**
 * Measure the motor inductance with short voltage pulses. The difference from the
 * other function is that this one will aim for a specific measurement current. It
 * will also use an appropriate switching frequency.
 *
 * @param curr_goal
 * The measurement current to aim for.
 *
 * @param samples
 * The number of samples to average over.
 *
 * @param *curr
 * The current that was used for this measurement.
 *
 * @return
 * The average d and q axis inductance in uH.
 */
static float solenoid_measure_inductance_current(float curr_goal, int samples, float *curr, float *ld_lq_diff) {
	float duty_last = 0.0;
	for (float i = 0.02;i < 0.5;i *= 1.5) {
		float i_tmp;
		solenoid_measure_inductance(i, 10, &i_tmp, 0);

		duty_last = i;
		if (i_tmp >= curr_goal) {
			break;
		}
	}

	float ind = solenoid_measure_inductance(duty_last, samples, curr, ld_lq_diff);
	return ind;
}
