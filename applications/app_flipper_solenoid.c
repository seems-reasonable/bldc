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

// Some useful includes
#include "comm_can.h"
#include "commands.h"
#include "encoder.h"
#include "hw.h"
#include "mc_interface.h"
#include "terminal.h"
#include "timeout.h"
#include "utils.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Threads
static THD_FUNCTION(flipper_solenoid_thread, arg);
static THD_WORKING_AREA(flipper_solenoid_thread_wa, 2048);

// Will be called after each output PWM cycle, in the interrupt.
static void control_callback(void);

// Terminal command implementations.
static void terminal_solenoid_pulse(int argc, const char **argv);
static void terminal_solenoid_plot(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
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

void app_custom_configure(app_configuration *conf) { (void)conf; }

static THD_FUNCTION(flipper_solenoid_thread, arg) {
  (void)arg;

  chRegSetThreadName("App Flipper Solenoid");

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
    // TODO(Brian): Do the normal PWM input controls.
    // TODO(Brian): timeout_reset(); // Reset timeout if everything is OK.
    // TODO(Brian): How does this get overriden by terminal commands?

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
        current <= mc_interface_get_configuration()->l_current_max) {
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
