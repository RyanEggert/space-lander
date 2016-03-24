// quad.h
#ifndef _QUAD_H_
#define _QUAD_H_

#include <stdint.h>
#include "pin.h"
#include "timer.h"

void init_oc(void);

int8_t quad_lut [] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
// LUT index is  a'b`ab, where a and b are the current 1-bit binary reading of
// the quadrature encoder's A and B pins, respectively, and a' and b' are the 
// previous 1-bit binary readings of the quadrature encoder's A and B pins. 

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

void quad_debug(_QUAD *self, _PIN *lut3, _PIN *lut2, _PIN *lut1, _PIN *lut0 );

void quad_reset_counter(_QUAD *self);

#endif