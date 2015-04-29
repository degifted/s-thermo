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

#ifndef CONFIG_H_
#define CONFIG_H_

#define     CFG_PRESET                          1        // 1 - hot plate, 5 liter crock
                                                         // 2 - immersion heater, 10 liter water bath
                                                         // 3 - ...

// Begin of default settings

#define     TRIAC_MODULATOR_RESOLUTION          100      // Resolution of the triac modulator
                                                         // 100 means that power can be
                                                         // regulated in 1% steps.
#define     PREHEAT_ENERGY                      700      // Amount of energy for preheat
                                                         // per each degree of an error.
                                                         // 700 means 7 seconds of full power per each degree.
#define     PREHEAT_START_THRESHOLD             7        // Minimum difference between target and current
                                                         // temperature at which the preheat starts.
#define     PREHEAT_STOP_THRESHOLD              3        // A safety threshold of the error at which
                                                         // the preheat switches off (abnormal situation).
#define     PID_A                               100      // PID output attenuation (a reciprocal to gain).
#define     PID_P                               6        // PID proportional coefficient.
#define     PID_I                               0.016    // PID integral coefficient.
#define     PID_D                               23       // PID derivative coefficient.
#define     PID_D_INTERVAL                      118      // PID time interval on which the derivative
                                                         // is calculated. 118 × 0.76 ÷ 60 = 1.5 minutes
#define     PID_I_P_LIMIT                       5        // Upper and lower limits of the integral and
#define     PID_I_N_LIMIT                      -5        // derivative components. Beyond these limits
#define     PID_D_P_LIMIT                       10       // the corresponding value is saturated.
#define     PID_D_N_LIMIT                      -60        
#define     PID_INTEGRATOR_BAND                 2        // PID error boundaries inside which the
                                                         // integrator starts working.
#define     PID_UPPER_REGULATION_LIMIT          1        // PID maximum negative error, beyond which
                                                         // the PID regulator is switched off (abnormal situation).
#define     MAXIMUM_TEMPERATURE_CHANGE_RATE     20       // Maximum allowed speed of temperature change per
                                                         // PID_D_INTERVAL. If the temperature is changing too
                                                         // fast, there is something wrong with the setup.
#define     MAXIMUM_TEMPERATURE                 95       // Temperature limits for the set point.
#define     MINIMUM_TEMPERATURE                 25
#define     MAXIMUM_ALLOWED_OVERHEAT            5        // If the temperature goes beyond that limit,
                                                         // the regulation process must be aborted.
// End of default settings

#if    CFG_PRESET == 1

#elif  CFG_PRESET == 2

#define PREHEAT_ENERGY 700

#elif  CFG_PRESET == 3

#else
 #error "Invalid preset"
#endif

#endif
