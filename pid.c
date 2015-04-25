/**
 * Copyright (c) 2015, Dmitriy Gorokh
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
#include "config.h"

float delayLine[PID_D_INTERVAL];
float integral;
float prevTemp;
uint8_t delayLineHeadIdx;

float getDelayedValue(float currValue)
{
    float delayedValue = delayLine[delayLineHeadIdx];
    delayLine[delayLineHeadIdx++] = currValue;
    delayLineHeadIdx %= PID_D_INTERVAL;
    return delayedValue;
}

void resetPID(float currTemp)
{
    uint8_t i;
    
    for(i = 0; i < PID_D_INTERVAL; i++)
        delayLine[i] = currTemp;

    integral = 0;
    delayLineHeadIdx = 0;
    prevTemp = currTemp;
}

// Returns:
//  0                                   ramping down
//  TRIAC_MODULATOR_RESOLUTION          ramping up
//  >0..<TRIAC_MODULATOR_RESOLUTION     continuous PID regulation
//  -1                                  regulation error
int updatePID(float currTemp, float targetTemp)
{
    float error, derivative, output;

    error = targetTemp - currTemp;

    // apply simple moving average LPF filter to the derivative
    prevTemp = (currTemp + prevTemp) / 2;
    derivative = getDelayedValue(prevTemp) - prevTemp;
    prevTemp = currTemp;

    // check if current temperature:
    //  - is not changing too fast
    //  - do not exceed maximum allowed overheat
    //  - is withing regulation limits
    if ((fabsf(derivative) > MAXIMUM_TEMPERATURE_CHANGE_RATE) ||
        (error < -1 * MAXIMUM_ALLOWED_OVERHEAT) ||
        (currTemp > MAXIMUM_TEMPERATURE)){
        return -1;
    }

    derivative = derivative > PID_D_P_LIMIT / PID_D ? PID_D_P_LIMIT / PID_D : derivative;
    derivative = derivative < PID_D_N_LIMIT / PID_D ? PID_D_N_LIMIT / PID_D : derivative;

    // turn off and reset PID in case of overheating (should not happen under normal workflow)
    if (error < -1 * PID_UPPER_REGULATION_LIMIT){
        resetPID(currTemp);
        return 0;
    }

    // only use integral if PD regulation is settled down 
    if (fabsf(error) < PID_INTEGRATOR_BAND){
        integral += error;
        integral = integral > PID_I_P_LIMIT / PID_I ? PID_I_P_LIMIT / PID_I : integral;
        integral = integral < PID_I_N_LIMIT / PID_I ? PID_I_N_LIMIT / PID_I : integral;
    }

    output = (error       * PID_P
            + integral    * PID_I
            + derivative  * PID_D)
                          / (PID_A / TRIAC_MODULATOR_RESOLUTION);

    // apply output clipping
    output = output < 0 ? 0 : output;
    output = output > TRIAC_MODULATOR_RESOLUTION ? TRIAC_MODULATOR_RESOLUTION : output;

    return lround(output);
}
