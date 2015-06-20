/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Compound heli variables and functions

#include "Copter.h"

#if COMPOUND == ENABLED

void Copter::compound_radio_passthrough(void)
{
    compound.set_rudder(channel_yaw->control_in);
    compound.set_aileron(channel_roll->control_in);
    compound.set_elevator(channel_pitch->control_in);
    compound.set_thrust(channel_throttle->control_in);
}
#endif  // COMPOUND == ENABLED
