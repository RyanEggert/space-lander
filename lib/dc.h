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
#ifndef LIB_DCM_H_
#define LIB_DCM_H_

#include <stdint.h>
#include "pin.h"
#include "oc.h"
#include "timer.h"

void init_dcm(void);

typedef struct {
    uint8_t dir;
    uint16_t speed;
    uint16_t freq;
    uint8_t mode;  // 0 for two-wire, 1 for 1-wire control
    _PIN *PWM;
    _PIN *DIR;
    _OC *oc;
} _DCM;

extern _DCM dc1, dc2;

void dc_init(_DCM *self, _PIN *pwm_pin, _PIN *dir_pin, uint16_t freq, _OC *oc);
void dc_free();

void dc_speed(_DCM *self, uint16_t speed);
void dc_direction(_DCM *self, uint8_t dir);
void dc_velocity(_DCM *self, uint16_t speed, uint8_t dir);

#endif  // LIB_DCM_H_
