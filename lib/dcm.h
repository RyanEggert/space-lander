#ifndef LIB_DCM_H_
#define LIB_DCM_H_

#include <stdint.h>
#include "pin.h"
#include "oc.h"
#include "timer.h"
#include "stops.h"

void init_dcm(void);

typedef struct {
    uint8_t dir;
    uint16_t speed;
    uint16_t freq;
    uint8_t mode;   // 0: sign-magnitude PWM; 1: locked-antiphase PWM control
    _PIN *pin_PWM;  // Pin connected to motor driver's "PWM" input
    _PIN *pin_DIR;  // Pin connected to motor driver's "DIR" input
    _OC *oc;
    _ESTOP *stop_min;  // minimum limit switch (endstop)
    _ESTOP *stop_max;  // maximum limit switch (endstop)
} _DCM;


extern _DCM dcm1, dcm2;

void dcm_init(_DCM *self, _PIN *pin_PWM, _PIN *pin_DIR, uint16_t freq, uint8_t mode, _OC *oc, _ESTOP *endstop_min, _ESTOP *endstop_max);

void dcm_run(_DCM *self);
void dcm_stop(_DCM *self);

void dcm_speed(_DCM *self, uint16_t speed);
void dcm_direction(_DCM *self, uint8_t dir);
void dcm_velocity(_DCM *self, uint16_t speed, uint8_t dir);
void dcm_check_stops(_DCM *self);

#endif  // LIB_DCM_H_
