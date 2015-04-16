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


#define TRIAC_MODULATOR_RESOLUTION  200


#define buzzer_toggle() PORTB ^= _BV(1)
#define buzzer_off()    PORTB &= ~_BV(1)
#define beeper_on()     TCCR2 |= (1 << CS21)
#define beeper_off()    TCCR2 &= ~(1 << CS21); \
                        buzzer_off()
#define heater_on()     PORTD |= _BV(0)
#define heater_off()    PORTD &= ~_BV(0)
#define heater_power(p) powerDebt = 0; \
                        currPower = (p)
#define button()        !(PIND & (1<<PD5))
#define enc_A()         !(PIND & (1<<PD3))
#define enc_B()         !(PIND & (1<<PD4))

volatile float          currTemp;
volatile uint8_t        targetTemp;
uint8_t EEMEM           NonVolatileTargetTemp = 60;
char                    lcdBuf[50];
char const              *msg = "Off     ";
int8_t                  cnt1 = 0;
int8_t                  cnt2 = 0;
int16_t                 cnt3 = 0;
int16_t                 powerDebt = 0;
uint16_t                secondsElapsed = 0;
int16_t                 currPower = -1;
uint8_t                 buttonIsPressed = 0;
int                     currTempRounded;

void updateLCD(void){
    lcd_gotoxy(0, 0);
    if (msg == (char*)"M")
        sprintf(lcdBuf, "    %d.%d\nM    %03d", (int)currTemp, (int)((float)(currTemp - (int)currTemp) * 10), currPower);
    else
        sprintf(lcdBuf, "%dC %d.%d\n%s%d%c%02d %03d", targetTemp, (int)currTemp, (int)((float)(currTemp - (int)currTemp) * 10), msg, secondsElapsed / 3600, secondsElapsed & 3 ? ':' : ' ',  (secondsElapsed / 60) % 60, currPower);
    lcd_puts(lcdBuf);
}

// Triac modulator
ISR (INT0_vect)
{
    powerDebt += TRIAC_MODULATOR_RESOLUTION - currPower;
    if (powerDebt < 0)
        powerDebt = 0;
    if (currPower > 0){
        if (powerDebt >= TRIAC_MODULATOR_RESOLUTION){
            powerDebt -= TRIAC_MODULATOR_RESOLUTION;
            heater_off();
        }else{
            heater_on();
        }
    } else {
        heater_off();
        powerDebt = 0;
    }
}

// Encoder processing
ISR(INT1_vect)
{    
    if (enc_B()){
        if (msg == (char*)"M"){
            if (currPower > 0){
                heater_power(currPower - 1);
            }
        } else if (targetTemp > 25)
            targetTemp--;
    } else {
        if (msg == (char*)"M"){
            if (currPower < TRIAC_MODULATOR_RESOLUTION){
                heater_power(currPower + 1);
            }
        } else if (targetTemp < 99)
            targetTemp++;
    }
    updateLCD();
}

// Beeper
ISR (TIMER2_COMP_vect)
{
    cnt2++;
    cnt3++;
    if (cnt2 == 10){
        cnt2 = 0;
        if (cnt3 & (1 << 12))
            buzzer_toggle();
    }
}

// Realtime clock
ISR (TIMER1_COMPA_vect)
{
    cnt1++;
    if (cnt1 == 100)
        cnt1 = 0;
    if (secondsElapsed && !cnt1){
        secondsElapsed++;
    }
}

// Button processing
ISR(TIMER0_OVF_vect) {
    if (button() && !buttonIsPressed){
        buttonIsPressed = 1;
        if (msg == (char*)"Ready   "){
            msg = "";
            beeper_off();
            secondsElapsed = 1;
        } else if ((msg == (char*)"") || (msg == (char*)"M") || (msg == (char*)"Failure ") || (msg == (char*)"Preheat ") || (msg == (char*)"Cooling ")){
            msg = "Off     ";
            beeper_off();
            secondsElapsed = 0;
            heater_power(-1);
        } else if (msg == (char*)"Off     "){
            if (NonVolatileTargetTemp != targetTemp)
                eeprom_write_byte(&NonVolatileTargetTemp, targetTemp);
            msg = "";
            secondsElapsed = 0;
            heater_power(0);
            pid_reset(currTemp);
        }
    } else if (!button() && buttonIsPressed){
        buttonIsPressed = 0;
    } else if (button() && buttonIsPressed){
        if (msg != (char*)"M")
            if (++buttonIsPressed == 100){
                msg = "M";
                heater_power(0);
            }
    }
}

int main(void)
{
    PORTD |= (1<<PD3) | (1<<PD4) | (1<<PD5);
    DDRD &= ~((1<<PD2) | (1<<PD3) | (1<<PD4) | (1<<PD5));
    DDRB |= (1<<PB1);
    DDRD |= (1<<PD0) | (1<<PD1);
    PORTD &= ~(1<<PD1);

    MCUCR = (1 << ISC11) | (1 << ISC01) | (1 << ISC00);
    GICR = (1 << INT1) | (1 << INT0);

    TIMSK =1 <<TOIE0;
    TCCR0 = (1<<CS00) | (1<<CS02);

    OCR1A = 14999;
    TCCR1B |= (1 << WGM12);
    TIMSK |= (1 << OCIE1A);
    TCCR1B |= (1 << CS11); 

    OCR2 = 62;
    TCCR2 |= (1 << WGM21);
    TIMSK |= (1 << OCIE2);
    //TCCR2 |= (1 << CS21);

/*
    OCR1A = 0x01FF;
    // set PWM for 50% duty cycle @ 10bit

    TCCR1A |= (1 << COM1A1);
    // set none-inverting mode

    TCCR1A |= (1 << WGM11) | (1 << WGM10);
    // set 10bit phase corrected PWM Mode

    TCCR1B |= (1 << CS11);
    // set prescaler to 8 and starts PWM
*/

    wdt_enable(WDTO_1S);

    lcd_init(LCD_DISP_ON);
    lcd_clrscr();

    targetTemp = eeprom_read_byte(&NonVolatileTargetTemp);

    sei();

    while (1) {
        currTemp = ds18b20_gettemp();
        currTempRounded = lround(currTemp);
        if ((currTempRounded > targetTemp + 5) && (msg == (char*)"")){
            if (secondsElapsed){
                msg = "Failure ";
                heater_power(-1);
                beeper_on();
            }else{
                msg = "Cooling ";
                heater_power(-1);
            }
        } else if ((currTempRounded < targetTemp - 4) && (msg == (char*)"")){
            if (secondsElapsed){
                msg = "Failure ";
                heater_power(-1);
                beeper_on();
            }else{
                msg = "Preheat ";
                heater_power(255);
            }
        } else if (msg == (char*)"Cooling "){
            msg = "Ready   ";
            heater_power(0);
            beeper_on();
        } else if ((currTempRounded == targetTemp - 3) && ((msg == (char*)"Preheat "))){
            msg = "Ready   ";
            heater_power(0);
            beeper_on();
        } else if ((currTempRounded == targetTemp + 2) && ((msg == (char*)"Cooling "))){
            msg = "Ready   ";
            heater_power(0);
            beeper_on();
        } else if ((currTempRounded > targetTemp) && (msg == (char*)"Preheat ")){
            msg = "Failure ";
            heater_power(-1);
            beeper_on();
        } else if ((currTempRounded < targetTemp) && (msg == (char*)"Cooling ")){
            msg = "Failure ";
            heater_power(-1);
            beeper_on();
        } else if (!secondsElapsed && (msg == (char*)"")){
            secondsElapsed = 1;
        }
        if ((currPower >= 0) && (currPower <= TRIAC_MODULATOR_RESOLUTION) && (msg != (char*)"M")){
            heater_power(pid_update(currTemp, targetTemp) * TRIAC_MODULATOR_RESOLUTION);
        }

        updateLCD();
        wdt_reset();
    }
}

