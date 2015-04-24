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

#define     TRIAC_MODULATOR_RESOLUTION          100
#define     PREHEAT_ENERGY                      800 // amount of energy for preheat per each degree of an error
#define     PID_P                               6//2.6//0.026
#define     PID_I                               0.014//0.014//0.00012
#define     PID_D                               24//25//0.16
#define     PID_A                               100
#define     PID_D_WINDOW                        160//160 // 160 = 2 minutes
#define     PID_I_P_LIMIT                       350
#define     PID_I_N_LIMIT                      -350
#define     PID_D_P_LIMIT                       1
#define     PID_D_N_LIMIT                      -4
#define     PID_INTEGRATOR_BAND                 2
#define     PID_UPPER_REGULATION_LIMIT          1
#define     MAXIMUM_TEMPERATURE_CHANGE_RATE     10
#define     MAXIMUM_TEMPERATURE                 99
#define     MINIMUM_TEMPERATURE                 20
#define     MAXIMUM_ALLOWED_OVERHEAT            5
