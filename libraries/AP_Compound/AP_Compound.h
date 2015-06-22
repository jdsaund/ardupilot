// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_COMPOUND_H__
#define __AP_COMPOUND_H__

#include <AP_Common.h>
#include <RC_Channel.h>
#include <RC_Channel_aux.h>
#include <AP_Motors.h>
#include <AC_AttitudeControl.h>
#include <Filter.h>
#include <AC_HELI_PID.h>

// servo update rate
#define AP_COMPOUND_SPEED_DEFAULT               125 // default output rate to the servos in hz
#define AP_COMPOUND_LEAKY_I_USE                 1
#define AP_COMPOUND_RATE_INTEGRATOR_LEAK_RATE   0.02f
#define AP_COMPOUND_RATE_FF_FILTER              10.0f

#define AP_COMPOUND_RC_CH_AIL                        CH_9
#define AP_COMPOUND_RC_CH_ELE                        CH_10
#define AP_COMPOUND_RC_CH_RUD                        CH_11

/// @class      AP_Compound
class AP_Compound {
public:

    // Constructor
    AP_Compound(uint16_t loop_rate,
                AC_AttitudeControl& attitude_control,
                AP_AHRS &ahrs,
                const AP_Vehicle::MultiCopter &aparm,
                AP_Motors& motors,
                AC_HELI_PID& pid_aileron,
                AC_HELI_PID& pid_elevator,
                AC_HELI_PID& pid_rudder,
                RC_Channel& ail_servo,
                RC_Channel& ele_servo,
                RC_Channel& rud_servo):

                _servo_ail(ail_servo),
                _servo_ele(ele_servo),
                _servo_rud(rud_servo),
                _ahrs(ahrs),
                _aparm(aparm),
                _motors(motors),
                _attitude_control(attitude_control),
                _pid_aileron(pid_aileron),
                _pid_elevator(pid_elevator),
                _pid_rudder(pid_rudder),
                _loop_rate(loop_rate),
                _speed_hz(AP_COMPOUND_SPEED_DEFAULT),
                _last_check_servo_map_ms(0),
                _aileron_out(0),
                _elevator_out(0),
                _rudder_out(0),
                _thrust_out(0),
                _dt(AC_ATTITUDE_100HZ_DT),
                _passthrough_aileron(0), _passthrough_elevator(0), _passthrough_rudder(0),
                aileron_feedforward_filter(AP_COMPOUND_RATE_FF_FILTER),
                elevator_feedforward_filter(AP_COMPOUND_RATE_FF_FILTER),
                rudder_feedforward_filter(AP_COMPOUND_RATE_FF_FILTER)
                {
                    //AP_Param::setup_object_defaults(this, var_info);

                    // initialise flags
                    _flags.armed            = false;
                    _flags.limit_roll       = false;
                    _flags.limit_pitch      = false;
                    _flags.limit_yaw        = false;
                    _flags.leaky_i          = AP_COMPOUND_LEAKY_I_USE;
                    _flags.servo_passthrough = false;
                }

    // init
    void Init();

    void set_update_rate( uint16_t speed_hz );

    // enable - starts allowing signals to be sent to motors
    void enable();

    // output - sends commands to the motors
    void output();

    void set_arm_status(bool arm_status)    { _flags.armed = arm_status;};

    // rate_controller_run - run lowest level body-frame rate controller and send outputs to the motors
    // should be called at 100hz or more
    virtual void rate_controller_run();

    void passthrough_to_servos(int16_t roll_passthrough, int16_t pitch_passthrough, int16_t yaw_passthrough);

    void write_servos();
protected:

private:

    //
    // body-frame rate controller
    //
    // rate_bf_to_motor_roll_pitch - ask the rate controller to calculate the motor outputs to achieve the target body-frame rate (in centi-degrees/sec) for roll, pitch and yaw
    // outputs are sent directly to motor class
    void rate_bf_to_motor_roll_pitch(float rate_roll_target_cds, float rate_pitch_target_cds);

    // send the output from the attitude controller to the motors
    void set_aileron(int16_t roll_in)       {_aileron_out = roll_in;};
    void set_elevator(int16_t pitch_in)     {_elevator_out = pitch_in;};
    void set_rudder(int16_t yaw_in)         {_rudder_out = yaw_in;};
    void set_thrust(int16_t thrust_in)      {_thrust_out = thrust_in;};

    // external objects
    const AP_AHRS&      _ahrs;
    const AP_Vehicle::MultiCopter &_aparm;
    AP_Motors&          _motors;
    AC_AttitudeControl& _attitude_control;
    AC_PID&             _pid_aileron;
    AC_PID&             _pid_elevator;
    AC_PID&             _pid_rudder;
    RC_Channel&         _servo_ail;
    RC_Channel&         _servo_ele;
    RC_Channel&         _servo_rud;

    // internal objects
    uint16_t            _loop_rate;                 // rate at which output() function is called (normally 400hz)
    uint16_t            _speed_hz;                  // speed in hz to send updates to motors
    uint32_t            _last_check_servo_map_ms;   // system time of latest call to check_servo_map function
    int16_t             _aileron_out;               // aileron output
    int16_t             _elevator_out;              // elevator output
    int16_t             _rudder_out;                // rudder output
    int16_t             _thrust_out;                // rudder output
    Vector3f            _rate_bf_target;            // body-rate rate targets for roll,pitch,yaw
    float               _dt;                        // time delta in seconds
    int16_t             _passthrough_aileron;
    int16_t             _passthrough_elevator;
    int16_t             _passthrough_rudder;

    // LPF filters to act on Rate Feedforward terms to linearize output.
    // Due to complicated aerodynamic effects, feedforwards acting too fast can lead
    // to jerks on rate change requests.
    LowPassFilterFloat aileron_feedforward_filter;
    LowPassFilterFloat elevator_feedforward_filter;
    LowPassFilterFloat rudder_feedforward_filter;

    // flags bitmask
    struct flags_type {
        uint8_t    armed                   : 1;
        uint8_t    limit_roll              : 1;
        uint8_t    limit_pitch             : 1;
        uint8_t    limit_yaw               : 1;
        uint8_t    leaky_i                 : 1;
        uint8_t    servo_passthrough       : 1;
    } _flags;

};
#endif // __AP_COMPOUND_H__
