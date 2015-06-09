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

#include "AP_Compound.h"
#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

//
// public methods
//

// init
void AP_Compound::Init()
{
        check_servo_map();

        // enable aux servos on init
        _flags.rudder_control = true;
        _flags.aileron_control = true;
        _flags.elevator_control = true;

        // keep thrust motor disabled until we arm
        _flags.thrust_control = false;

        // setup channel functions for aux servos
        _rudder_idx = RC_Channel_aux::k_rudder;
        _aileron_idx = RC_Channel_aux::k_aileron;
        _elevator_idx = RC_Channel_aux::k_elevator;
        _thrust_idx = RC_Channel_aux::k_motor;

        // move servo to its trim position
        RC_Channel_aux::set_radio_to_trim((RC_Channel_aux::Aux_servo_function_t) _rudder_idx);
        RC_Channel_aux::set_radio_to_trim((RC_Channel_aux::Aux_servo_function_t) _aileron_idx);
        RC_Channel_aux::set_radio_to_trim((RC_Channel_aux::Aux_servo_function_t) _elevator_idx);

        // keep thrust motor at minimum throttle
        RC_Channel_aux::set_radio_to_min((RC_Channel_aux::Aux_servo_function_t) _thrust_idx);
}

// enable - starts allowing signals to be sent to motors
void AP_Compound::enable()
{
        // enable thrust motor
        _flags.thrust_control = true;
}

// sends commands to the motors
void AP_Compound::output()
{
        // check servo map every three seconds to allow users to modify parameters
        uint32_t now = hal.scheduler->millis();
        if (now - _last_check_servo_map_ms > 3000) {
            check_servo_map();
            _last_check_servo_map_ms = now;
        }

        // write the results to the servos
        write_servo(_rudder_idx, _rudder_out);      // write output for rudder
        write_servo(_aileron_idx, _aileron_out);    // write output for aileron
        write_servo(_elevator_idx, _elevator_out);  // write output for elevator

        if (_flags.armed == true){
            // write the results to the motor
            write_servo(_thrust_idx, _thrust_out);  // write output for rudder
        } else {
            // keep the thrust motor at minimum throttle
            RC_Channel_aux::set_radio_to_min((RC_Channel_aux::Aux_servo_function_t) _thrust_idx);
        }
}

//
// protected methods
//

//
// private methods
//

// check_servo_map - detects which axis we control using the functions assigned to the servos in the RC_Channel_aux
// should be called periodically (i.e. 1hz or less)
void AP_Compound::check_servo_map()
{
    _flags.rudder_control = RC_Channel_aux::function_assigned(_rudder_idx);
    _flags.aileron_control = RC_Channel_aux::function_assigned(_aileron_idx);
    _flags.elevator_control = RC_Channel_aux::function_assigned(_elevator_idx);
    _flags.thrust_control = RC_Channel_aux::function_assigned(_thrust_idx);
}

// move_servo - moves servo with the given id to the specified output
void AP_Compound::write_servo(uint8_t function_idx, int16_t servo_out)
{
    RC_Channel_aux::set_radio((RC_Channel_aux::Aux_servo_function_t)function_idx, servo_out);
}
