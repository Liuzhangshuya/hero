#include "AutoGimbal.h"
#include "remote_control.h"
#include <float.h>
/* Protocol layouts must remain identical when switching from Keil to GCC. */
_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24,
               "Vision protocol requires IEEE 754 binary32 floats");
_Static_assert(VISION_PACKET_SIZE == 29 && VISION_CRC_OFFSET == 27,
               "SP command must be 27 payload bytes plus CRC16");
_Static_assert(sizeof(PackToVision_Def) == 28, "Vision packet must be 28 bytes");
_Static_assert(sizeof(RC_ctrl_t) == 24, "Remote control structure layout changed");
_Static_assert(PITCH_SOFT_MIN_ECD < PITCH_SOFT_MAX_ECD, "Pitch soft stops are reversed");
_Static_assert(PITCH_SOFT_MAX_ECD < 8192, "Pitch encoder stop is out of range");
_Static_assert(PITCH_MID_HOME_ECD >= PITCH_SOFT_MIN_ECD &&
               PITCH_MID_HOME_ECD <= PITCH_SOFT_MAX_ECD,
               "Pitch middle-switch home must stay inside the soft stops");
