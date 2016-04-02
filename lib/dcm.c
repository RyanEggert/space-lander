#include <p24FJ128GB206.h>
#include "common.h"
#include "dcm.h"

_DCM dcm1, dcm2;

uint16_t dcm_locked_antiphase_speed(uint16_t std_speed, uint8_t dir) {
    /*
    Returns a speed 'centered' around 32768 for use in locked-antiphase control
    mode. e.g.,
    32768 = dcm_locked_antiphase_speed(0, 1) = dcm_locked_antiphase_speed(0, 0)
    65535 = dcm_locked_antiphase_speed(65535, 1)
    0 = dcm_locked_antiphase_speed(65535, 0)
    */
    if (std_speed = 0) {
        return 32768;
    } else if (dir == 1) {
        return (std_speed / 2) + 32768;
    } else {
        return 32767 - (std_speed / 2);
    }
}

void dcm_init(_DCM *self, _PIN *pin_PWM, _PIN *pin_DIR, uint16_t freq, uint8_t mode, _OC *oc) {
    self->dir = 0;
    self->speed = 0;
    self->freq = freq;
    self->pin_PWM = pin_PWM;
    self->pin_DIR = pin_DIR;
    self->mode = mode;
    self->oc = oc;
    if (mode == 1) {
        // Locked-antiphase PWM control mode
        oc_pwm(self->oc, self->pin_DIR, NULL, freq, dcm_locked_antiphase_speed(0, self -> dir));
        pin_digitalOut(self->pin_PWM);
        pin_set(self->pin_PWM);
    } else {
        // Sign-magnitude PWM control mode
        oc_pwm(self->oc, self->pin_PWM, NULL, freq, 0);
        pin_digitalOut(self->pin_DIR);
        pin_clear(self->pin_DIR);
    }
}

void init_dcm(void) {
    dcm_init(&dcm1, &D[8], &D[7], 1e3, 0, &oc7);
}

void dcm_run(_DCM *self) {
    dcm_velocity(self, self->speed, self->dir);
}

void dcm_speed(_DCM *self, uint16_t speed) {
    self->speed = speed;

    if (self->mode == 1) {
        // Locked-antiphase PWM control mode
        uint16_t offset_speed = dcm_locked_antiphase_speed(speed, self->dir);
        pin_write(self->pin_DIR, offset_speed);
    } else {
        // Sign-magnitude PWM control mode
        pin_write(self->pin_PWM, speed);
    }

}

void dcm_direction(_DCM *self, uint8_t dir) {
    if (self->dir == dir) {
        return;
    }
    self->dir = dir;
    if (self->mode == 1) {
        // Locked-antiphase PWM control mode
        // Recalculate and write speed.
        uint16_t offset_speed = dcm_locked_antiphase_speed(self->speed, dir);
        pin_write(self->pin_DIR, offset_speed);
    } else {
        // Sign-magnitude PWM control mode
        pin_write(self->pin_DIR, dir);
    }
}

void dcm_velocity(_DCM *self, uint16_t speed, uint8_t dir) {
    dcm_speed(self, speed);
    dcm_direction(self, dir);
}

void dcm_stop(_DCM *self) {
    /*
    Stops motor.
    */
    dcm_speed(self, 0);
}