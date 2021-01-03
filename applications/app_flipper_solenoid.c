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

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
  mc_interface_set_pwm_callback(control_callback);

  stop_now = false;
  chThdCreateStatic(flipper_solenoid_thread_wa,
                    sizeof(flipper_solenoid_thread_wa), NORMALPRIO,
                    flipper_solenoid_thread, NULL);

  // Terminal commands for the VESC Tool terminal can be registered.
  terminal_register_command_callback("solenoid_pulse", "Pulse a solenoid",
                                     "[current_A] [time_S]",
                                     terminal_solenoid_pulse);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
//
// Note that is called even if the custom application was not previously
// configured.
void app_custom_stop(void) {
  mc_interface_set_pwm_callback(0);
  terminal_unregister_callback(terminal_solenoid_pulse);

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

    timeout_reset(); // Reset timeout if everything is OK.

    // Run your logic here. A lot of functionality is available in
    // mc_interface.h.
    // TODO(Brian): Do the normal PWM input controls.
    // TODO(Brian): How does this get overriden by terminal commands?

    chThdSleepMilliseconds(10);
  }
}

static void control_callback(void) {}

static void terminal_solenoid_pulse(int argc, const char **argv) {
  if (argc == 3) {
    float current = -1.0;
    float time = -1.0;
    sscanf(argv[1], "%f", &current);
    sscanf(argv[2], "%f", &time);

    commands_printf("Pulsing solenoid at %f A for %f seconds", (double)current,
                    (double)time);

    // Based on rotor_lock_openloop.
    if (current > 0.0 &&
        current <= mc_interface_get_configuration()->l_current_max) {
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
        commands_printf("Done\n");
      }
    } else {
      commands_printf("Invalid argument(s).\n");
    }
  } else {
    commands_printf("This command requires two arguments.\n");
  }
}
