// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
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

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "AP_RotaryEncoder.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

/* 
   see if the PX4 driver is available
*/
bool AP_RotaryEncoder::detect()
{
    int fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
    if (fd == -1) {
        hal.console->printf("PX4 PWM input driver is not available\n");
        return false;
    }
    close(fd);
    hal.console->printf("PX4 PWM input driver is available\n");
    return true;
}

void AP_RotaryEncoder::read(void)
{
    hal.console->printf("Updating reading...\n");
    if (_fd == -1) {
        hal.console->printf("PX4 PWM input driver is not available\n");
        return;
    }

    struct pwm_input_s pwm;
    uint16_t count = 0;
    uint32_t now = hal.scheduler->millis();

    _pulse_width    = 0;
    _period         = 0;

    while (::read(_fd, &pwm, sizeof(pwm)) == sizeof(pwm)) {
        _pulse_width    += pwm.pulse_width; // add each new value to itself for averaging of samples
        _period         += pwm.period;
        _last_pulse_time_ms = now;

        if (_good_sample_count > 1) {
            count++;
            _last_timestamp = pwm.timestamp;
        } else {
            _good_sample_count++;
        }

    }
    _pulse_width /= AP_ROTARYENCODER_AVERAGING_NUM_SAMPLES; // divide by the number of samples for averaging
    _period      /= AP_ROTARYENCODER_AVERAGING_NUM_SAMPLES;
    if (_period == 0 || _pulse_width >= _period){
        // prevent strange output
        _angle_cds = 0;
    } else {
        _angle_cds = 3599 * ((float)_pulse_width / (float)_period) - 1800; // outputs angle in centi-degrees, -1800 to +1800
    }
    hal.console->printf(PSTR("Pulse Width = %.0f\nPeriod = %.0f\nAngle = %.1f\n"), (double)_pulse_width, (double)_period, (double)_angle_cds/10);
    ioctl(_fd, SENSORIOCRESET, 0); // reset the sample buffer
    hal.console->printf("Update reading complete.\n");
}

void AP_RotaryEncoder::init(void)
{
    if (detect()) {
        hal.console->printf("Initializing...Creating new rotary encoder object...");

        _fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
        if (_fd == -1) {
            hal.console->printf("Unable to open PX4 PWM rangefinder\n");
            return;
        }

        // keep a queue of AP_ROTARYENCODER_AVERAGING_NUM_SAMPLES number of samples for averaging during the read cycle
        if (ioctl(_fd, SENSORIOCSQUEUEDEPTH, AP_ROTARYENCODER_AVERAGING_NUM_SAMPLES) != 0) {
            hal.console->printf("Failed to setup range finder queue\n");
            return;
        }

        hal.console->printf("Done!\n");
        return;
    }
}

#endif // CONFIG_HAL_BOARD
