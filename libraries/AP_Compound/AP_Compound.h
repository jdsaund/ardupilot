// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_COMPOUND_H__
#define __AP_COMPOUND_H__

#include <AP_Common/AP_Common.h>

// servo update rate
#define AP_COMPOUND_SPEED_DEFAULT     125 // default output rate to the servos

/// @class      AP_Compound
class AP_Compound {
public:

    // Constructor
    AP_Compound(uint16_t loop_rate, uint16_t speed_hz = AP_COMPOUND_SPEED_DEFAULT);

protected:

    uint16_t            _loop_rate;                 // rate at which output() function is called (normally 400hz)
    uint16_t            _speed_hz;                  // speed in hz to send updates to motors

};
#endif // __AP_COMPOUND_H__
