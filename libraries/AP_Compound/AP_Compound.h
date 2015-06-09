// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_COMPOUND_H__
#define __AP_COMPOUND_H__

#include <AP_Common/AP_Common.h>
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include <RC_Channel/RC_Channel_aux.h>
#include <AP_Motors/AP_Motors.h>

// servo update rate
#define AP_COMPOUND_SPEED_DEFAULT     125 // default output rate to the servos

/// @class      AP_Compound
class AP_Compound {
public:

    // Constructor
    AP_Compound(uint16_t    loop_rate,
                uint16_t    speed_hz = AP_COMPOUND_SPEED_DEFAULT):
                _loop_rate(loop_rate),
                _speed_hz(speed_hz),
                _last_check_servo_map_ms(0),
                _rudder_out(0),
                _thrust_out(0),
                _rudder_idx(RC_Channel_aux::k_none)

        {
            // initialise flags
            _flags.rudder_control = false;
            _flags.thrust_control = false;
            _flags.armed          = false;
        };

    // init
    void Init();

    // enable - starts allowing signals to be sent to motors
    void enable();

    // output - sends commands to the motors
    void output();

    void set_rudder(int16_t yaw_in){_rudder_out = yaw_in;};
    void set_throttle(int16_t thrust_in){_thrust_out = thrust_in;};

    void set_arm_status(bool arm_status){ _flags.armed = arm_status;};

protected:

private:

    void check_servo_map();
    void write_servo(uint8_t function_idx, int16_t servo_out);

    // flags bitmask
    struct flags_type {
        bool    rudder_control          : 1;
        bool    thrust_control          : 1;
        bool    armed                   : 1;
    } _flags;

    uint16_t            _loop_rate;                 // rate at which output() function is called (normally 400hz)
    uint16_t            _speed_hz;                  // speed in hz to send updates to motors
    uint32_t            _last_check_servo_map_ms;   // system time of latest call to check_servo_map function
    int16_t             _rudder_out;                // rudder output
    int16_t             _thrust_out;                // rudder output

    RC_Channel_aux::Aux_servo_function_t    _rudder_idx; //fixed wing surfaces
    RC_Channel_aux::Aux_servo_function_t    _thrust_idx; // thrust motor
};
#endif // __AP_COMPOUND_H__
