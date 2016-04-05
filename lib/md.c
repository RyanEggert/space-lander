/*
** Copyright (c) 2016, Evan Dorsky
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
#include "md.h"

_MD md1, md2, mdp;

void init_md(void) {
    md_init(&md1, &D[6], &D[7], 1e3, &oc7);
}

void md_init(_MD *self, _PIN *pin1, _PIN *pin2, uint16_t freq, _OC *oc) {
    self->dir = 0;
    self->speed = 0;
    self->freq = freq;
    self->pins[0] = pin1;
    self->pins[1] = pin2;
    self->braked = 0;
    self->oc = oc;

    oc_pwm(self->oc, self->pins[0], &timer5, freq, 0);
    OC5CON2 = 0x000F; //synchronize to timer5
    OC7CON2 = 0x000F;
    pin_digitalOut(self->pins[1]);
    pin_clear(self->pins[1]);
}

void md_free(_MD *self) {
    // clear owned pins/oc's
}

void md_brake(_MD *self) {
    self->braked = 1;
    pin_write(self->pins[0], 0);
    pin_write(self->pins[1], 0);
}

void md_run(_MD *self) {
    self->braked = 0;
    md_velocity(self, self->speed, self->dir);
}

void md_speed(_MD *self, uint16_t speed) {
    self->speed = speed;
    
    if (!self->braked) {
        pin_write(self->pins[self->dir], speed);
    }
}

void md_direction(_MD *self, uint8_t dir) {
    if (self->dir == dir) {
        return;
    }
    self->dir = dir;

    oc_free(self->oc);
    oc_pwm(self->oc, self->pins[dir], &timer5, self->freq, self->speed);
    OC5CON2 = 0x000F; //synchronize to timer5
    OC7CON2 = 0x000F;
    pin_clear(self->pins[!dir]);
}

void md_velocity(_MD *self, uint16_t speed, uint8_t dir) {
    md_speed(self, speed);
    md_direction(self, dir);
}