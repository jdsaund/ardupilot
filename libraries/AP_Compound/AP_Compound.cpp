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
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// init
void AP_Compound::Init()
{
    // ensure inputs are not passed through to servos
    _flags.servo_passthrough = false;

    // disable channels 9, 10 and 11 from being used by RC_Channel_aux
    RC_Channel_aux::disable_aux_channel(AP_COMPOUND_RC_CH_AIL);
    RC_Channel_aux::disable_aux_channel(AP_COMPOUND_RC_CH_ELE);
    RC_Channel_aux::disable_aux_channel(AP_COMPOUND_RC_CH_RUD);

    // swash servo initialisation
    _servo_ail.set_range(0,1000);
    _servo_ele.set_range(0,1000);
    _servo_rud.set_range(0,1000);

    // servo min/max values
    _servo_ail.radio_min = 1000;
    _servo_ail.radio_max = 2000;
    _servo_ele.radio_min = 1000;
    _servo_ele.radio_max = 2000;
    _servo_rud.radio_min = 1000;
    _servo_rud.radio_max = 2000;

    enable();

}

// enable - starts allowing signals to be sent to motors
void AP_Compound::enable()
{
    // enable output channels
    hal.rcout->enable_ch(AP_COMPOUND_RC_CH_AIL);    // ail servo
    hal.rcout->enable_ch(AP_COMPOUND_RC_CH_ELE);    // ele servo
    hal.rcout->enable_ch(AP_COMPOUND_RC_CH_RUD);    // rud servo
}

// sends commands to the motors
void AP_Compound::output()
{
        // write the results to the servos
        write_servos();
}

// rate_controller_run - run lowest level rate controller and send outputs to the motors
// should be called at 100hz or more
void AP_Compound::rate_controller_run()
{
    if (_flags.servo_passthrough) {
        set_aileron(_passthrough_aileron);
        set_elevator(_passthrough_elevator);
        set_rudder(_passthrough_rudder);
    } else {
        update_rate_bf_targets();
        rate_bf_to_motor_roll_pitch_yaw(_rate_bf_target.x, _rate_bf_target.y, _rate_bf_target.z);
    }
}

// rate_bf_to_motor_roll_pitch_yaw - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
void AP_Compound::rate_bf_to_motor_roll_pitch_yaw(float rate_roll_target_cds, float rate_pitch_target_cds, float rate_yaw_target_cds)
{
    float roll_pd, roll_i, roll_ff;             // used to capture pid values
    float pitch_pd, pitch_i, pitch_ff;          // used to capture pid values
    float yaw_pd, yaw_i, yaw_ff;                // used to capture pid values
    float rate_roll_error, rate_pitch_error, rate_yaw_error;    // simply target_rate - current_rate
    float roll_out, pitch_out, yaw_out;
    const Vector3f& gyro = _ahrs.get_gyro();     // get current rates

    // calculate error
    rate_roll_error = rate_roll_target_cds - gyro.x * AC_ATTITUDE_CONTROL_DEGX100;
    rate_pitch_error = rate_pitch_target_cds - gyro.y * AC_ATTITUDE_CONTROL_DEGX100;
    rate_yaw_error = rate_yaw_target_cds - gyro.z * AC_ATTITUDE_CONTROL_DEGX100;

    // input to PID controller
    _pid_aileron.set_input_filter_all(rate_roll_error);
    _pid_elevator.set_input_filter_all(rate_pitch_error);
    _pid_rudder.set_input_filter_all(rate_yaw_error);

    // call p and d controllers
    roll_pd = _pid_aileron.get_p() + _pid_aileron.get_d();
    pitch_pd = _pid_elevator.get_p() + _pid_elevator.get_d();
    yaw_pd = _pid_rudder.get_p() + _pid_rudder.get_d();

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

    // get yaw i term
    yaw_i = _pid_rudder.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_flags.limit_yaw || ((yaw_i>0&&rate_yaw_error<0)||(yaw_i<0&&rate_yaw_error>0))){
        if (_flags.leaky_i) {
            yaw_i = ((AC_HELI_PID&)_pid_rudder).get_leaky_i(AP_COMPOUND_RATE_INTEGRATOR_LEAK_RATE);
        }else{
            yaw_i = _pid_rudder.get_i();
        }
    }

    roll_ff = aileron_feedforward_filter.apply(((AC_HELI_PID&)_pid_aileron).get_vff(rate_roll_target_cds), _dt);
    pitch_ff = elevator_feedforward_filter.apply(((AC_HELI_PID&)_pid_elevator).get_vff(rate_pitch_target_cds), _dt);
    yaw_ff = rudder_feedforward_filter.apply(((AC_HELI_PID&)_pid_rudder).get_vff(rate_yaw_target_cds), _dt);

    // add feed forward and final output
    roll_out = roll_pd + roll_i + roll_ff;
    pitch_out = pitch_pd + pitch_i + pitch_ff;
    yaw_out = yaw_pd + yaw_i + yaw_ff;

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
    if (fabsf(yaw_out) > AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX) {
        yaw_out = constrain_float(yaw_out,-AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX,AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
        _flags.limit_yaw = true;
    }else{
        _flags.limit_yaw = false;
    }

    // output to motors
    set_aileron(roll_out);
    set_elevator(pitch_out);
    set_rudder(yaw_out);
}

void AP_Compound::passthrough_to_servos(int16_t roll_passthrough, int16_t pitch_passthrough, int16_t yaw_passthrough)
{
    _flags.servo_passthrough = true;

    // output to motors
    _passthrough_aileron = roll_passthrough;
    _passthrough_elevator = pitch_passthrough;
    _passthrough_rudder = yaw_passthrough;
}

void AP_Compound::write_servos()
{
    // servo outputs
    _servo_ail.servo_out = _aileron_out + (_servo_ail.radio_trim-1500) + 500;
    _servo_ele.servo_out = _elevator_out + (_servo_ele.radio_trim-1500) + 500;
    _servo_rud.servo_out = _rudder_out + (_servo_rud.radio_trim-1500) + 500;

    // use servo_out to calculate pwm_out and radio_out
    _servo_ail.calc_pwm();
    _servo_ele.calc_pwm();
    _servo_rud.calc_pwm();

    // actually move the servos
    hal.rcout->write(AP_COMPOUND_RC_CH_AIL, _servo_ail.radio_out);
    hal.rcout->write(AP_COMPOUND_RC_CH_ELE, _servo_ele.radio_out);
    hal.rcout->write(AP_COMPOUND_RC_CH_RUD, _servo_rud.radio_out);
}

