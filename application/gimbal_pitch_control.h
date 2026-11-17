#ifndef GIMBAL_PITCH_CONTROL_H
#define GIMBAL_PITCH_CONTROL_H

#include <math.h>

/* An outward request at a stop is rejected, never turned into reverse motion.
 * Targets outside the travel range may still move back toward the valid range.
 */
static __inline float gimbal_pitch_limit_increment(float relative, float error,
    float add, float lower, float upper)
{
    float target;
    if (!isfinite(relative) || !isfinite(error) || !isfinite(add) ||
        !isfinite(lower) || !isfinite(upper) || lower >= upper)
        return 0.0f;
    target = relative + error;
    if (add > 0.0f)
    {
        if (target >= upper) return 0.0f;
        if (add > upper - target) return upper - target;
    }
    else if (add < 0.0f)
    {
        if (target <= lower) return 0.0f;
        if (add < lower - target) return lower - target;
    }
    return add;
}

static __inline float gimbal_pitch_step_toward(float error, float max_step)
{
    if (!isfinite(error) || !isfinite(max_step) || max_step <= 0.0f)
        return 0.0f;
    if (error > max_step) return max_step;
    if (error < -max_step) return -max_step;
    return error;
}
#endif
