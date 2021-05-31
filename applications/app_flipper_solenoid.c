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

    // Run your logic here. A lot of functionality is available in
    // mc_interface.h.
    // TODO(Brian): How does this get overriden by terminal commands?

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

static void control_callback(void) {}

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
