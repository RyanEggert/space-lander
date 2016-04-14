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
#include "common.h"
#include "stepper.h"
#include "ui.h"

_ST st_d;

void init_st(void) {
    // init stepper object on oc5 w/ 50% duty cycle
    st_init(&st_d, &D[0], &D[1], &D[2], &D[3], &oc5, 0x7FFF);
    init_ui();
}

void st_init(_ST *self, _PIN *pin1, _PIN *pin2, _PIN *pin3, _PIN *pin4, _OC *oc, uint16_t duty_cyc) {
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

    int i;
    for (i=0; i<=8; i++) {
        pin_digitalOut(self->pins[i]);
    }
    oc_pwm(self->oc, self->pins[0], NULL, self->speed, 0);
    OC5CON2 = 0x000F; //synchronize to timer5
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
    // speed = 1.6 deg/step / (360 deg/rev) * freq step/sec
    //       = 0.004*freq rev/sec
    if (speed > 0) {
        if (self->speed != speed) {
            oc_free(self->oc);
            oc_pwm(self->oc, self->pins[0], &timer5, speed, self->duty_cyc);
            // OC5CON2 = 0x000F; //synchronize to timer5
            // OC7CON2 = 0x000F;
        }
    }
    else {
        oc_free(self->oc);
    }
    // else {
        // oc_free(self->oc);
    // }
    self->speed = speed;
}

void st_direction(_ST *self, uint8_t dir) {
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