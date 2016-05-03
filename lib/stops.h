#ifndef LIB_STOPS_H_
#define LIB_STOPS_H_

#include <stdint.h>
#include <stdbool.h>
#include "pin.h"
#include "timer.h"

void init_stops(void);

typedef struct {
    bool hit;  // true if triggered/depressed, else false
    volatile uint16_t last_state;
    volatile uint16_t curr_state;
    volatile uint16_t counter;
    _PIN *pin;  // Pin connected to motor driver's "PWM" input
} _ESTOP;

extern _ESTOP es_x_l, es_x_r, es_y_top, es_y_bot, es_landing;

void stop_init(_ESTOP *self, _PIN *pin);

bool stop_read(_ESTOP *self);

#endif  // LIB_STOPS_H_
