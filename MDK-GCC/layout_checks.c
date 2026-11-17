#include "AutoGimbal.h"
#include "remote_control.h"
/* Protocol layouts must remain identical when switching from Keil to GCC. */
_Static_assert(sizeof(PackToVision_Def) == 28, "Vision packet must be 28 bytes");
_Static_assert(sizeof(RC_ctrl_t) == 24, "Remote control structure layout changed");
_Static_assert(PITCH_SOFT_MIN_ECD < PITCH_SOFT_MAX_ECD, "Pitch soft stops are reversed");
_Static_assert(PITCH_SOFT_MAX_ECD < 8192, "Pitch encoder stop is out of range");
_Static_assert(PITCH_MID_HOME_ECD >= PITCH_SOFT_MIN_ECD &&
               PITCH_MID_HOME_ECD <= PITCH_SOFT_MAX_ECD,
               "Pitch middle-switch home must stay inside the soft stops");
