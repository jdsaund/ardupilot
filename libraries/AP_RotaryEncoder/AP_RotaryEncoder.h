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
 
#ifndef __AP_ROTARYENCODER_H__
#define __AP_ROTARYENCODER_H__

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_pwm_input.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <uORB/topics/pwm_input.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <AP_HAL.h>

#define AP_ROTARYENCODER_AVERAGING_NUM_SAMPLES 4


extern const AP_HAL::HAL& hal;

/// @class      AP_RotaryEncoder
class AP_RotaryEncoder {
public:

    // Constructor
    AP_RotaryEncoder():
        _angle_cds(0)
        {};

    // static detection function
    static bool input_pin_free();

    void init();

    void read(void);

private:
    struct pwm_input_s _pwm;
    int _fd;
    uint32_t _pulse_width;
    uint32_t _period;
    int32_t _angle_cds;
};

#endif // __AP_ROTARYENCODER_H__
