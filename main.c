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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "lcd.h"
#include "ds18b20.h"
#include "pid.h"
#include "config.h"
#include "version.h"

#define buzzerToggle()  PORTB ^= _BV(1)
#define buzzerOff()     PORTB &= ~_BV(1)
#define beeperOn()      TCCR2 |= (1 << CS21)
#define beeperOff()     TCCR2 &= ~(1 << CS21); \
                        buzzerOff()
#define heaterOn()      PORTD |= _BV(0)
#define heaterOff()     PORTD &= ~_BV(0)
#define heaterPower(p)  currPower = (p)
#define button()        !(PIND & (1<<PD5))
#define encA()          !(PIND & (1<<PD3))
#define encB()          !(PIND & (1<<PD4))

typedef enum {          STATE_OFF,
                        STATE_AUTO,
                        STATE_MANUAL,
                        STATE_FAILURE,
                        STATE_SYSTEMINFO
            } state_t;

volatile int8_t         targetTemp;
float                   currTemp;
char                    lcdBuf[2][20];
state_t                 currState = STATE_OFF;
int8_t                  cnt1 = 0;
int16_t                 cnt2 = 0;
uint32_t                totalPowerConsumed = 0;
int16_t                 preheatPower = 0;
int16_t                 powerDebt = 0;
uint16_t                secondsElapsed = 0;
int16_t                 currPower = 0;
uint8_t                 buttonIsPressed = 0;

uint8_t EEMEM           NonVolatileTargetTemp = 57;



void updateLCD(void){
    sprintf(lcdBuf[0], "%dC%3d.%d",
        targetTemp,
        (int)currTemp,
        (int)((float)(currTemp - (int)currTemp) * 10));

    switch (currState){
        case STATE_MANUAL:
            sprintf(lcdBuf[0], "M%5d.%d",
                (int)currTemp,
                (int)((float)(currTemp - (int)currTemp) * 10));
            sprintf(lcdBuf[1], "%-5d%3d",
                (int)totalPowerConsumed, currPower);
            break;

        case STATE_SYSTEMINFO:
            sprintf(lcdBuf[1], "%s",
                GIT_VERSION);
            break;

        case STATE_AUTO:
            sprintf(lcdBuf[1], "%d%c%02d %3d",
                secondsElapsed / 3600,
                secondsElapsed & 3 ? ':' : ' ',
                (secondsElapsed / 60) % 60,
                currPower);
            break;

        case STATE_FAILURE:
            sprintf(lcdBuf[1], "FAILURE ");
            break;

        case STATE_OFF:
            sprintf(lcdBuf[1], "%-5dOFF",
                (int)(totalPowerConsumed / TRIAC_MODULATOR_RESOLUTION));
            break;
    }

    //lcd_clrscr();
    lcd_gotoxy(0, 0);
    lcd_puts(lcdBuf[0]);
    lcd_gotoxy(0, 1);
    lcd_puts(lcdBuf[1]);
}

// Triac modulator
ISR (INT0_vect)
{
    powerDebt += TRIAC_MODULATOR_RESOLUTION - currPower;
    if (currPower > 0){
        if (powerDebt >= TRIAC_MODULATOR_RESOLUTION){
            powerDebt -= TRIAC_MODULATOR_RESOLUTION;
            heaterOff();
        }else{
            heaterOn();
            totalPowerConsumed++;
        }
    } else {
        heaterOff();
        powerDebt = 0;
    }
}

// Encoder processing
ISR (INT1_vect)
{    
    if (encB()){
        if (currState == STATE_MANUAL){
            if (currPower > 0){
                heaterPower(currPower - 1);
            }
        } else if (targetTemp > MINIMUM_TEMPERATURE){
            targetTemp--;
            resetPID(currTemp);
        }
    } else {
        if (currState == STATE_MANUAL){
            if (currPower < TRIAC_MODULATOR_RESOLUTION){
                heaterPower(currPower + 1);
            }
        } else if (targetTemp < MAXIMUM_TEMPERATURE){
            targetTemp++;
            resetPID(currTemp);
        }
    }
    updateLCD();
}

// Beeper
ISR (TIMER2_COMP_vect)
{
    if (cnt2++ & (1 << 10))
        buzzerToggle();
}

// Realtime clock
ISR (TIMER1_COMPA_vect)
{
    if (cnt1++ == 100)
        cnt1 = 0;
    if (secondsElapsed && !cnt1){
        secondsElapsed++;
    }
}

// Button processing
ISR (TIMER0_OVF_vect) {
    if (button() && !buttonIsPressed){
        buttonIsPressed = 1;
        if (currState == STATE_OFF){
            if (NonVolatileTargetTemp != targetTemp)
                eeprom_write_byte(&NonVolatileTargetTemp, targetTemp);
            currState = STATE_AUTO;
            secondsElapsed = 1;
            totalPowerConsumed = 0;
            preheatPower = (targetTemp - currTemp > PREHEAT_THRESHOLD) ? (PREHEAT_ENERGY * (targetTemp - currTemp)) : 0;
            heaterPower(0);
            resetPID(currTemp);
        } else {
            currState = STATE_OFF;
            beeperOff();
            secondsElapsed = 0;
            heaterPower(-1);
        }
    } else if (!button() && buttonIsPressed){
        buttonIsPressed = 0;
    } else if (button() && buttonIsPressed){
        if (buttonIsPressed < 255)
            buttonIsPressed++;
        if (buttonIsPressed == 100){
            secondsElapsed = 0;
            totalPowerConsumed = 0;
            currState = STATE_MANUAL;
            heaterPower(0);
        }
        if (buttonIsPressed == 200){
            currState = STATE_SYSTEMINFO;
        }
    }
}

int main(void)
{
    PORTD |=  (1<<PD3) | (1<<PD4) | (1<<PD5);
    PORTD &= ~(1<<PD1);
    DDRD  &= ~((1<<PD2) | (1<<PD3) | (1<<PD4) | (1<<PD5));
    DDRD  |=  (1<<PD0) | (1<<PD1);
    DDRB  |=  (1<<PB1);

    MCUCR = (1 << ISC11) | (1 << ISC01) | (1 << ISC00);
    GICR = (1 << INT1) | (1 << INT0);

    TIMSK = (1 <<TOIE0);
    TCCR0 = (1<<CS00) | (1<<CS02);

    OCR1A = 14999;
    TCCR1B |= (1 << WGM12);
    TIMSK |= (1 << OCIE1A);
    TCCR1B |= (1 << CS11); 

    OCR2 = 255;
    TCCR2 |= (1 << WGM21);
    TIMSK |= (1 << OCIE2);

    wdt_enable(WDTO_2S);
    lcd_init(LCD_DISP_ON);
    targetTemp = eeprom_read_byte(&NonVolatileTargetTemp);
    sei();

    while (1) {
        currTemp = ds18b20_gettemp();
        if (currState == STATE_AUTO){
            if (preheatPower > totalPowerConsumed){
                heaterPower(TRIAC_MODULATOR_RESOLUTION);       // preheat on maximum power
                if (currTemp >= targetTemp)                    // that should not happen
                    preheatPower = 0;
            } else {
                heaterPower(updatePID(currTemp, targetTemp));
            }
            if (currPower < 0){
                currState = STATE_FAILURE;
                secondsElapsed = 0;
                heaterPower(-1);
                beeperOn();
            }
        }
        updateLCD();
        wdt_reset();
    }
}

