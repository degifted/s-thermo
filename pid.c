/**
 * Copyright (c) 2015, Dmitriy Gorokh
 * Based on implementation of Daniel Strother < http://danstrother.com/ >
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   - Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   - Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   - The name of the author may not be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <math.h>
#include "pid.h"

const float k_p   = 0.0115;
const float k_i   = 0.000045;
const float k_d   = 0.175;
#define k_delay 50

float pid_prev[k_delay];
float pid_int;

uint8_t pid_prev_index;

float pid_prev_update(float prev)
{
    float popped = pid_prev[pid_prev_index];
    pid_prev[pid_prev_index] = prev;

    pid_prev_index++;
    if(pid_prev_index >= k_delay)
        pid_prev_index = 0;

    return popped;
}

void pid_reset(float temp)
{
    uint8_t i;
    
    for(i=0;i<k_delay;i++)
        pid_prev[i] = temp;

    pid_int = 0;
    pid_prev_index = 0;
}

// PID algorithm based on information presented in Tim Wescott's "PID wihout a PhD" article
float pid_update(float temp, float target)
{
    float error, derivative;
    float command;

    error       = target - temp;
    derivative  = pid_prev_update(temp) - temp;

    command     = error      * k_p;
    command    += pid_int    * k_i;
    command    += derivative * k_d;

    //printf("p=%f, i=%f, d=%f, error=%f\n", error      * k_p, pid_int    * k_i, derivative * k_d, error);

    // only update integral if output is not saturated (or if change would reduce saturation)
    if( (command >= 0 && command <= 1) || (command > 0 && error < 0) || (command < 0 && error > 0) )
        pid_int     += error;

    if(command < 0)
        command     = 0;
    else if(command > 1)
        command     = 1;

    return command;
}
