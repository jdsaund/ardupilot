/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Compound copter variables and functions

#include "compound.h"

#if COMPOUND == ENABLED
static void compound_radio_passthrough()
{
    compound.passthrough_to_servos(channel_roll->control_in, channel_pitch->control_in, channel_yaw->control_in);
}
#endif  // COMPOUND == ENABLED
