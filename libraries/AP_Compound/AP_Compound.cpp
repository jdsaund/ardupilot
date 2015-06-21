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
// rate controller (body-frame) methods
//

// rate_controller_run - run lowest level rate controller and send outputs to the motors
// should be called at 100hz or more
void AP_Compound::rate_controller_run()
{
    if (_flags.servo_passthrough) {
        set_aileron(_passthrough_aileron);
        set_elevator(_passthrough_elevator);
        set_rudder(_passthrough_rudder);
    } else {
        rate_bf_to_motor_roll_pitch(_rate_bf_target.x, _rate_bf_target.y);
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

//
// body-frame rate controller
//

// rate_bf_to_motor_roll_pitch - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
void AP_Compound::rate_bf_to_motor_roll_pitch(float rate_roll_target_cds, float rate_pitch_target_cds)
{
    float roll_pd, roll_i, roll_ff;             // used to capture pid values
    float pitch_pd, pitch_i, pitch_ff;          // used to capture pid values
    float rate_roll_error, rate_pitch_error;    // simply target_rate - current_rate
    float roll_out, pitch_out;
    const Vector3f& gyro = _ahrs.get_gyro();     // get current rates

    // calculate error
    rate_roll_error = rate_roll_target_cds - gyro.x * AC_ATTITUDE_CONTROL_DEGX100;
    rate_pitch_error = rate_pitch_target_cds - gyro.y * AC_ATTITUDE_CONTROL_DEGX100;

    // input to PID controller
    _pid_aileron.set_input_filter_all(rate_roll_error);
    _pid_elevator.set_input_filter_all(rate_pitch_error);

    // call p and d controllers
    roll_pd = _pid_aileron.get_p() + _pid_aileron.get_d();
    pitch_pd = _pid_elevator.get_p() + _pid_elevator.get_d();

    // get roll i term
    roll_i = _pid_aileron.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_flags.limit_roll || ((roll_i>0&&rate_roll_error<0)||(roll_i<0&&rate_roll_error>0))){
        if (_flags.leaky_i){
            roll_i = ((AC_HELI_PID&)_pid_aileron).get_leaky_i(AP_COMPOUND_RATE_INTEGRATOR_LEAK_RATE);
        }else{
            roll_i = _pid_aileron.get_i();
        }
    }

    // get pitch i term
    pitch_i = _pid_elevator.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_flags.limit_pitch || ((pitch_i>0&&rate_pitch_error<0)||(pitch_i<0&&rate_pitch_error>0))){
        if (_flags.leaky_i) {
            pitch_i = ((AC_HELI_PID&)_pid_elevator).get_leaky_i(AP_COMPOUND_RATE_INTEGRATOR_LEAK_RATE);
        }else{
            pitch_i = _pid_elevator.get_i();
        }
    }

    roll_ff = aileron_feedforward_filter.apply(((AC_HELI_PID&)_pid_aileron).get_ff(rate_roll_target_cds), _dt);
    pitch_ff = elevator_feedforward_filter.apply(((AC_HELI_PID&)_pid_elevator).get_ff(rate_pitch_target_cds), _dt);

    // add feed forward and final output
    roll_out = roll_pd + roll_i + roll_ff;
    pitch_out = pitch_pd + pitch_i + pitch_ff;

    // constrain output and update limit flags
    if (fabsf(roll_out) > AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX) {
        roll_out = constrain_float(roll_out,-AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX,AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
        _flags.limit_roll = true;
    }else{
        _flags.limit_roll = false;
    }
    if (fabsf(pitch_out) > AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX) {
        pitch_out = constrain_float(pitch_out,-AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX,AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
        _flags.limit_pitch = true;
    }else{
        _flags.limit_pitch = false;
    }

    // output to motors
    set_aileron(roll_out);
    set_elevator(pitch_out);
}

void AP_Compound::passthrough_to_servos(int16_t roll_passthrough, int16_t pitch_passthrough, int16_t yaw_passthrough)
{
    _flags.servo_passthrough = true;

    // output to motors
    _passthrough_aileron = roll_passthrough;
    _passthrough_elevator = pitch_passthrough;
    _passthrough_rudder = yaw_passthrough;
}
