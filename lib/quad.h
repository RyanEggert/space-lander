// quad.h
#ifndef ROCKET_CONTROL_CUSTOM_LIB_QUAD_H_
#define ROCKET_CONTROL_CUSTOM_LIB_QUAD_H_

#include <stdint.h>
#include "pin.h"
#include "timer.h"


typedef void (*QReader)(void);
typedef void (*QTimerReader)(_TIMER *timer);

// Quadrature Encoder
typedef struct {
    unsigned char a_curr;
    unsigned char a_prev;
    unsigned char b_curr;
    unsigned char b_prev;
    uint8_t identifier;
    uint8_t encoder_read;
    uint32_t count;
    QReader read;
    QTimerReader timer_read;
    _PIN* A;
    _PIN* B;
} _QUAD;

extern _QUAD quad1, quad2;

void quad_read(_QUAD *self);

void init_quad(void);

void quad_init(_QUAD *self, _PIN *in_A, _PIN *in_B);

void quad_every(_QUAD *self, _TIMER *timer, float interval);

void quad_debug(_QUAD *self, _PIN *lut3, _PIN *lut2, _PIN *lut1, _PIN *lut0);

void quad_reset_counter(_QUAD *self);

#endif  // ROCKET_CONTROL_CUSTOM_LIB_QUAD_H_
