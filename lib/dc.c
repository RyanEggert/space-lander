#include <p24FJ128GB206.h>
#include "common.h"
#include "dc.h"

_MD md1, md2, mdp;

void init_md(void) {
    md_init(&md1, &D[8], &D[7], 1e3, &oc7);
}

void md_init(_MD *self, _PIN *pin1, _PIN *pin2, uint16_t freq, _OC *oc) {
    self->dir = 0;
    self->speed = 0;
    self->freq = freq;
    self->pins[0] = pin1;
    self->pins[1] = pin2;
    self->braked = 0;
    self->oc = oc;

    if (self != &mdp) {
        oc_pwm(self->oc, self->pins[0], &timer5, freq, 0);
        OC5CON2 = 0x000F; //synchronize to timer5
        OC7CON2 = 0x000F;
        pin_digitalOut(self->pins[1]);
        pin_clear(self->pins[1]);
    }
}

void md_free(_MD *self) {
    // clear owned pins/oc's
}

void md_brake(_MD *self) {
    self->braked = 1;
    if (self == &mdp) {
        md_brake(&md1);
        md_brake(&md2);
    } else {
        pin_write(self->pins[0], 0);
        pin_write(self->pins[1], 0);
    }
}

void md_run(_MD *self) {
    self->braked = 0;
    if (self == &mdp) {
        md_run(&md1);
        md_run(&md2);
    } else {
        md_velocity(self, self->speed, self->dir);
    }
}

void md_speed(_MD *self, uint16_t speed) {
    self->speed = speed;

    if (self == &mdp) {
        md_speed(&md1, self->speed);
        md_speed(&md2, self->speed);
    } else if (!self->braked) {
        pin_write(self->pins[self->dir], speed);
    }
}

void md_direction(_MD *self, uint8_t dir) {
    if (self->dir == dir) {
        return;
    }
    self->dir = dir;

    if (self == &mdp) {
        md_direction(&md1, dir);
        md_direction(&md2, dir);
    } else {
        oc_free(self->oc);
        oc_pwm(self->oc, self->pins[dir], &timer5, self->freq, self->speed);
        OC5CON2 = 0x000F; //synchronize to timer5
        OC7CON2 = 0x000F;
        pin_clear(self->pins[!dir]);
    }
}

void md_velocity(_MD *self, uint16_t speed, uint8_t dir) {
    md_speed(self, speed);
    md_direction(self, dir);
}