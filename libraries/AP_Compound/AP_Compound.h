// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_COMPOUND_H__
#define __AP_COMPOUND_H__

#include <AP_Common.h>
#include <RC_Channel.h>     // RC Channel Library
#include <RC_Channel_aux.h>

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
        _rudder_idx(RC_Channel_aux::k_none),
        _last_check_servo_map_ms(0)
        {
            // initialise flags
            _flags.rudder_control = false;
        };

    // init
    void Init();

    // enable - starts allowing signals to be sent to motors
    void enable();

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // output - sends commands to the motors
    void output();

protected:

private:

    void check_servo_map();
    void write_servo(uint8_t function_idx, int16_t servo_out);

    RC_Channel_aux::Aux_servo_function_t    _rudder_idx; //fixed wing surfaces

    // flags bitmask
    struct flags_type {
        bool    rudder_control          : 1;
    } _flags;

    uint16_t            _loop_rate;                 // rate at which output() function is called (normally 400hz)
    uint16_t            _speed_hz;                  // speed in hz to send updates to motors
    int16_t             _rudder_out;                // rudder output
    uint32_t            _last_check_servo_map_ms;   // system time of latest call to check_servo_map function

};
#endif // __AP_COMPOUND_H__
