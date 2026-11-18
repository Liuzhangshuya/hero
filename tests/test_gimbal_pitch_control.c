#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "gimbal_pitch_control.h"

static void close_to(float a, float b) { assert(fabsf(a-b) < 0.00001f); }
int main(void)
{
    const float lo=-0.3f, hi=0.3f;
    close_to(gimbal_pitch_limit_increment(0,0,0.1f,lo,hi),0.1f);
    close_to(gimbal_pitch_limit_increment(0,0,-0.1f,lo,hi),-0.1f);
    close_to(gimbal_pitch_limit_increment(hi,0,0.1f,lo,hi),0);
    close_to(gimbal_pitch_limit_increment(lo,0,-0.1f,lo,hi),0);
    close_to(gimbal_pitch_limit_increment(0.5f,0,0.1f,lo,hi),0);
    close_to(gimbal_pitch_limit_increment(0.5f,0,-0.1f,lo,hi),-0.1f);
    close_to(gimbal_pitch_limit_increment(-0.5f,0,-0.1f,lo,hi),0);
    close_to(gimbal_pitch_limit_increment(-0.5f,0,0.1f,lo,hi),0.1f);
    close_to(gimbal_pitch_limit_increment(0.25f,0,0.1f,lo,hi),0.05f);
    close_to(gimbal_pitch_limit_increment(-0.25f,0,-0.1f,lo,hi),-0.05f);
    close_to(gimbal_pitch_limit_increment(0,0,0.1f,hi,lo),0);
    close_to(gimbal_pitch_limit_increment(NAN,0,0.1f,lo,hi),0);
    close_to(gimbal_pitch_limit_increment(0,0,INFINITY,lo,hi),0);
    close_to(gimbal_pitch_step_toward(0.1f,0.02f),0.02f);
    close_to(gimbal_pitch_step_toward(-0.1f,0.02f),-0.02f);
    close_to(gimbal_pitch_step_toward(0.01f,0.02f),0.01f);
    uint32_t random=0x12345678U;
    for (unsigned i=0;i<100000;i++)
    {
        random=random*1664525U+1013904223U;
        float pos=((int)(random%20001)-10000)/10000.0f;
        random=random*1664525U+1013904223U;
        float add=((int)(random%20001)-10000)/100000.0f;
        float actual=gimbal_pitch_limit_increment(pos,0,add,lo,hi);
        assert(actual*add>=0.0f); /* Rejected requests cannot reverse. */
        assert(fabsf(actual)<=fabsf(add)+0.000001f);
        if (pos>=lo && pos<=hi) assert(pos+actual>=lo-0.000001f && pos+actual<=hi+0.000001f);
    }
    puts("Pitch limit and startup-ramp regression checks passed (100000 samples).");
    return 0;
}
