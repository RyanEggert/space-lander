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
#ifndef _ST_H_
#define _ST_H_

#include <stdint.h>
#include "pin.h"
#include "oc.h"
#include "timer.h"
#include "stops.h"

void init_st(void);

typedef struct {
    uint8_t dir;
    float speed;
    uint16_t step_size;
    uint8_t state;
    uint16_t duty_cyc;
    _PIN *pins[4];
    _OC *oc;
    _ESTOP *stop_min;  // minimum limit switch (endstop)
    _ESTOP *stop_max;  // maximum limit switch (endstop)
} _ST;

extern _ST st_d;

void st_init(_ST *self, _PIN *pin1, _PIN *pin2, _PIN *pin3, _PIN *pin4, _OC *oc, uint16_t duty_cyc);

void st_speed(_ST *self, float speed);
void st_state(_ST *self, uint8_t state);
void st_mode(_ST *self, uint16_t mode);
void st_direction(_ST *self, uint8_t dir);
// void st_step_size(_ST *self, uint8_t size);

#endif
