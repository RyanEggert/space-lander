// quad.c
#include <p24FJ128GB206.h>
#include "common.h"
#include "pin.h"
#include "timer.h"
#include "quad.h"
#include "stdio.h"


int8_t quad_lut [] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
// LUT index is  a'b`ab, where a and b are the current 1-bit binary reading of
// the quadrature encoder's A and B pins, respectively, and a' and b' are the
// previous 1-bit binary readings of the quadrature encoder's A and B pins.

_QUAD quad1, quad2;

void quad_read(_QUAD *self) {
    disable_interrupts();
    pin_set(&D[2]);
    self -> a_curr = pin_read(self -> A);
    self -> b_curr = pin_read(self -> B);
    unsigned char latest_read = (self -> a_curr << 1) + self -> b_curr;
    self -> encoder_read = ((self -> encoder_read << 2) + latest_read) & 0xF;
    int8_t delta = quad_lut[self -> encoder_read];
    self -> delta = delta;
    if (self -> counter == 0) {
        if (delta == -1) {
            self -> overflow += -1;
        }
    }

    self -> counter += delta;

    if (self -> counter == 0) {
        if (delta == 1) {
            self -> overflow += 1;
        }
    }
    
    self -> a_prev = self -> a_curr;
    self -> b_prev = self -> b_curr;
    pin_clear(&D[2]);
    enable_interrupts();
}

void _timer_read_quad1(_TIMER *timer) {
    quad_read(&quad1);
}

void _timer_read_quad2(_TIMER *timer) {
    quad_read(&quad2);
}

void _read_quad1(void) {
    quad_read(&quad1);
}

void _read_quad2(void) {
    quad_read(&quad2);
}

void init_quad_fxns(void) {
    /*
    Attaches the proper reading functions to each quadrature encoder object.
    */
    quad1.read = _read_quad1;
    quad2.read = _read_quad2;
    quad1.timer_read = _timer_read_quad1;
    quad2.timer_read = _timer_read_quad2;
}

void init_quad_ids(void) {
    /*
    Gives each quadrature encoder object a numerical identifier.
    */
    quad1.identifier = 1;
    quad2.identifier = 2;
}

void init_quad(void) {
    /*
    Call this to initialize this quadrature library.
    */
    init_quad_ids();
    init_quad_fxns();
}

void quad_init(_QUAD *self, _PIN *in_A, _PIN *in_B) {
    /*
    Initializes a quadrature encoder object connected to two digital input pins,
    in_A and in_B.
    */
    self -> A = in_A;
    self -> B = in_B;
    self -> a_prev = 0;
    self -> b_prev = 0;
    self -> overflow = 0;
    self -> counter = 4000;

    pin_digitalIn(in_A);
    pin_digitalIn(in_B);
}

void quad_every(_QUAD *self, _TIMER *timer, float interval) {
    /*
    Given an initialized _QUAD object, configures it to utilize the given
    _TIMER's interrupts to read the _QUAD at the specified time interval.
    */
    timer_every(timer, interval, self -> timer_read);
}


void quad_debug(_QUAD *self, _PIN *lut3, _PIN *lut2, _PIN *lut1, _PIN *lut0 ) {
    /*
    Given a quadrature encoder object and four configured digital output pins,
    writes the value used to index the LUT across the four pins as follows--
        LUT Index Bit   |  Source
        [LSB --> MSB]   |
        ****************************************
              0 (LSB)   | most recent value of B
              1         | most recent value of A
              2         | previously read value of B
              3 (MSB)   | previously read value of A
    */
    pin_write(lut0, self -> b_curr); // b0
    pin_write(lut1, self -> a_curr); // b1
    pin_write(lut2, self -> b_prev); // b2
    pin_write(lut3, self -> a_prev); // b3
}

void quad_reset_counter(_QUAD *self) {
    /*
    Resets the counter associated with the given quadrature encoder to zero.
    */
    self -> counter = 0;
}

float quad_meas_speed(_QUAD *self, float interval) {
    static uint32_t prev_ticks = 0;
    uint32_t current_ticks = self->counter;
    float speed = (current_ticks - prev_ticks)/interval; // Ticks per second
    prev_ticks = current_ticks;
    return speed;
}