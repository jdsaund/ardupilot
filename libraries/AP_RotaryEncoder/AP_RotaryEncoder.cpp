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
bool AP_RotaryEncoder::input_pin_free()
{
    int fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
    if (fd == -1) {
        hal.console->printf("PX4 PWM input driver is not available\n");
        return false;
    }
    close(fd);
    return true;
}

void AP_RotaryEncoder::read(void)
{
    if (_fd == -1) {
        hal.console->printf("PX4 PWM input driver is not available\n");
        return;
    }

    // zero the readings for each read cycle
    _pulse_width    = 0;
    _period         = 0;

    while (::read(_fd, &_pwm, sizeof(_pwm)) == sizeof(_pwm)) {
        // add each new value to itself for averaging of samples
        _pulse_width    += _pwm.pulse_width;
        _period         += _pwm.period;
    }

    // divide by the number of samples for averaging
    _pulse_width /= AP_ROTARYENCODER_AVERAGING_NUM_SAMPLES;
    _period      /= AP_ROTARYENCODER_AVERAGING_NUM_SAMPLES;

    // prevent strange output, then work out the angle
    if (_period == 0 || _pulse_width >= _period){
        _angle_cds = 0;
    } else {
        _angle_cds = 3599 * ((float)_pulse_width / (float)_period) - 1800; // outputs angle in centi-degrees, -1800 to +1800
    }
    hal.console->printf(PSTR("Pulse Width = %.0f\nPeriod = %.0f\nAngle = %.1f\n"), (double)_pulse_width, (double)_period, (double)_angle_cds/10);

    // reset the sample buffer
    ioctl(_fd, SENSORIOCRESET, 0);
}

void AP_RotaryEncoder::init(void)
{
    if (input_pin_free()) {
        _fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
        if (_fd == -1) {
            hal.console->printf("Unable to open Rotary Encoder\n");
            return;
        }

        // keep a queue of AP_ROTARYENCODER_AVERAGING_NUM_SAMPLES number of samples for averaging during the read cycle
        if (ioctl(_fd, SENSORIOCSQUEUEDEPTH, AP_ROTARYENCODER_AVERAGING_NUM_SAMPLES) != 0) {
            hal.console->printf("Failed to setup PWM sample queue\n");
            return;
        }
        return;
    }
}

#endif // CONFIG_HAL_BOARD
