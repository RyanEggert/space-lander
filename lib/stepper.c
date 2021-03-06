/*
** Copyright (c) 2016, Mason del Rosario
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**
**     1. Redistributions of source code must retain the above copyright
**        notice, this list of conditions and the following disclaimer.
**     2. Redistributions in binary form must reproduce the above copyright
**        notice, this list of conditions and the following disclaimer in the
**        documentation and/or other materials provided with the distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
** LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
** CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
** INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
** CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
** ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
** POSSIBILITY OF SUCH DAMAGE.
*/
#include <p24FJ128GB206.h>
#include <stdbool.h>
#include "common.h"
#include "stepper.h"
#include "ui.h"
#include "stops.h"
#include "pin.h"

_ST st_d;

void init_st(void) {
    // init stepper object on oc5 w/ 50% duty cycle
    st_init(&st_d, &D[0], &D[1], &D[2], &D[3], &oc5, 0x7FFF, &es_x_l, &es_x_r);
    init_ui();
}

void st_init(_ST *self, _PIN *pin1, _PIN *pin2, _PIN *pin3, _PIN *pin4, _OC *oc, uint16_t duty_cyc, _ESTOP *endstop_min, _ESTOP *endstop_max) {
    self->dir = 0;
    self->speed = 0;
    self->step_size = 0;
    self->state = 0;
    self->duty_cyc = duty_cyc;
    self->pins[0] = pin1;   // STEP
    self->pins[1] = pin2;   // DIR
    self->pins[2] = pin3;   // SLP+RST
    self->pins[3] = pin4;   // ENABLE+PFD
    self->oc = oc;
    self->stop_min = endstop_min;
    self->stop_max = endstop_max;
    int i;
    for (i=0; i<=3; i++) {
        pin_digitalOut(self->pins[i]);
    }
    oc_pwm(self->oc, self->pins[0], NULL, self->speed, 0);
    // OC5CON2 = 0x000F; //synchronize to timer5
    // OC7CON2 = 0x000F;
    st_state(&st_d, self->state);      // turn on controller
    // st_step_size(&st_d, 0); // full step
}

void st_state(_ST *self, uint8_t state) {
    self->state = state;
    if (state) { // 1 = Turn stepper drive on
        pin_write(self->pins[2], 1);
        pin_write(self->pins[3], 0);
        oc_free(self->oc);
        oc_pwm(self->oc, self->pins[0], NULL, self->speed, self->duty_cyc);
    }
    else {  // 0 = Turn stepper drive off
        pin_write(self->pins[2], 0);
        pin_write(self->pins[3], 1);
        oc_free(self->oc);
        oc_pwm(self->oc, self->pins[0], NULL, self->speed, 0);
    }
}

void st_speed(_ST *self, float speed) {
    /*
    speed = 1.6 deg/step / (360 deg/rev) * freq step/sec
    = (0.004/8)*freq rev/sec
    */

    if ((self->stop_min->hit == true) && (self->dir == 0)) {
        // If endstop is hit and we're moving towards it,
        // then set speed to zero. Movement in this direction is not
        // allowed.
        speed = 0;
    } else if ((self->stop_max->hit == true) && (self->dir == 1)) {
        // If endstop is hit and we're moving towards it,
        // then set speed to zero. Movement in this direction is not
        // allowed.
        speed = 0;
    }  // Else no endstops are triggered. Proceed normally

    if (speed > 0) {
        // If new speed is greater than zero,
        if (self->speed != speed) {  // and if new speed is different,
            oc_free(self->oc);  // then stop the pwm signal
            oc_pwm(self->oc, self->pins[0], NULL, speed, self->duty_cyc);
            // and start again at a new frequency, specified by speed.
        }
    }
    else {
        // If speed is zero (or less), stop the motor.
        oc_free(self->oc);
    }
    self->speed = speed;
}

void st_direction(_ST *self, uint8_t dir) {
    if (dir == self->dir) {  // if specified direction is same as curr. dir.,
        return;  // Exit. We do not need to update the direction
    }

    if ((self->stop_min->hit == true) && (dir == 0)) {
        // If endstop is hit and we specify moving towards it,
        // then do not change direction. Movement in the specified direction is
        // not allowed.
        return;
    } else if ((self->stop_max->hit == true) && (dir == 1)) {
        // If endstop is hit and we specify moving towards it,
        // then do not change direction. Movement in the specified direction is
        // not allowed.
        return;
    }

    if (dir) {
        // pin_set(self->pins[1]);
        pin_write(self->pins[1], 1);
    }
    else {
        // pin_clear(self->pins[1]);
        pin_write(self->pins[1], 0);
    }
    // if (self->dir != dir) {
    //     oc_free(self->oc);
    //     oc_pwm(self->oc, self->pins[0], NULL, self->speed, self->duty_cyc);
    //     // OC5CON2 = 0x000F; //synchronize to timer5
    //     // OC7CON2 = 0x000F;
    // }
    self->dir = dir;
    // pin_clear(self->pins[!dir]);
}

void st_stop(_ST *self) {
    st_speed(self, 0);
}

void st_manual_init(_ST *self, uint16_t man_pseudo_freq) {
    oc_free(self->oc);
    pin_digitalOut(self->pins[0]);
    pin_clear(self->pins[0]);
    self->manual_pseudo_freq = man_pseudo_freq;
    self->manual_count = 0;
    self->_manual_toggle_st = 0;
    self->_pseudo_freq_count = 0;
}

void st_manual_toggle(_ST *self){
    if ((self->stop_min->hit == true) && (self->dir == 0)) {
        // If endstop is hit and we specify moving towards it,
        // then do not change direction. Movement in the specified direction is
        // not allowed.
        return;
    } else if ((self->stop_max->hit == true) && (self->dir == 1)) {
        // If endstop is hit and we specify moving towards it,
        // then do not change direction. Movement in the specified direction is
        // not allowed.
        return;
    }

    if(self->_pseudo_freq_count == self->manual_pseudo_freq) {
        pin_toggle(self->pins[0]);
        self->_manual_toggle_st = (self->_manual_toggle_st + 1) % 2;
        self->_pseudo_freq_count = 0;
    } else {
        self->_pseudo_freq_count += 1;
    }
    
    // Increment step counter if rising edge
    if((self->_manual_toggle_st == 1) && (self->_pseudo_freq_count == 0)) {
        // Rising edge
        self->manual_count += 1;
    } 
}

void st_manual_exit(_ST *self) {
    oc_free(self->oc);
    oc_pwm(self->oc, self->pins[0], NULL, self->speed, 0);
}

void st_check_stops(_ST *self) {
    uint8_t dmin = stop_read(self->stop_min);
    uint8_t dmax = stop_read(self->stop_max);
    if ((dmin == true) || (dmax == true)) {  // If either dmin or dmax are true,
        st_stop(self);  // then a stop has just been hit. Stop the motor.
    }
}
// void st_step_size(_ST *self, uint8_t size) {
//     self->step_size = size;
//     if (size == 0) {    // full step
//         // pin_clear(self->pins[6]);
//         // pin_clear(self->pins[7]);
//         pin_write(self->pins[6], 0);
//         pin_write(self->pins[7], 0);
//     }
//     else if (size == 1) {   // half step
//         // pin_set(self->pins[6]);
//         // pin_clear(self->pins[7]);
//         pin_write(self->pins[6], 1);
//         pin_write(self->pins[7], 0);
//     }
//     else if (size == 2) {   // quarter step
//         // pin_clear(self->pins[6]);
//         // pin_set(self->pins[7]);
//         pin_write(self->pins[6], 0);
//         pin_write(self->pins[7], 1);
//     }
//     else if (size == 3) {   // quarter step
//         // pin_set(self->pins[6]);
//         // pin_set(self->pins[7]);
//         pin_write(self->pins[6], 1);
//         pin_write(self->pins[7], 1);
//     }
// }