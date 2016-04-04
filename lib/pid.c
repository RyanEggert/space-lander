#include "common.h"
#include "pid.h"

// void pid_init(_PID_FL *self, float Kp, float Ki, float Kd, ) {
//     self -> Kp = Kp;
//     self -> K


//     self -> position = 0;
//     self -> set_point = 0;
//     self -> integ_min = integ_min;
//     self -> integ_max = integ_max;

// }

float PID_FL_control(_PID_FL *self) {
    float error = self->set_point - self->position;
    float deriv = (self->position - self->_prev_position) / self->dt;

    self->_integ_state += error;

    if (self->_integ_state > self->integ_max) {
        self->_integ_state = self->integ_max;
    } else if (self->_integ_state < self->integ_min) {
        self->_integ_state = self->integ_min;
    };

    float pterm = self->Kp * error;
    float iterm = self->Ki * self->_integ_state;
    float dterm = self->Kd * deriv;

    self->_prev_position = self->position;
    return pterm + iterm + dterm;
}

uint32_t PID_U32_control(_PID_U32 *self) {
    uint32_t error = self->set_point - self->position;
    float deriv = (self->position - self->_prev_position) / ((float)(self->dt));

    self->_integ_state += error;

    if (self->_integ_state > self->integ_max) {
        self->_integ_state = self->integ_max;
    } else if (self->_integ_state < self->integ_min) {
        self->_integ_state = self->integ_min;
    };

    uint32_t pterm = self->Kp * error;
    uint32_t iterm = self->Ki * self->_integ_state;
    uint32_t dterm = self->Kd * deriv;

    self->_prev_position = self->position;
    return pterm + iterm + dterm;
}