#include <p24FJ128GB206.h>
#include <stdbool.h>
#include "common.h"
#include "pin.h"
#include "stops.h"

_ESTOP es_x_l, es_x_r, es_y_top, es_y_bot, es_landing;


void stop_init(_ESTOP *self, _PIN *pin) {
    self->hit = false;
    self->state = 0x0;

}

void init_stops(void) {
    /*
    Initialize enstop libary.
    */

    // Gantry X-Axis (stepper) endstops
    stop_init(&es_x_l, &D[6]);
    stop_init(&es_x_r, &D[7]);

    // Gantry Y-Axis (dc motor) endstops
    stop_init(&es_y_top, &D[4]);
    stop_init(&es_y_bot, &D[12]);

    // Landing pad endstop
    stop_init(&es_landing, &D[13]);

}

bool stop_read(_ESTOP *self) {
    /*
    Reads endstop with debouncing. This should be called at a fixed time
    interval (e.g., with timer interrupts). Returns a tri-state boolean
    (uint8_t) value representing observed change. Returns 0 (false) if button has
    transitioned from high to low, 1 (true) if button has transitioned from low
    to high, 2 (third state) if no state change has occurred.
    */
    self->state = (self->state << 1) | pin_read(self->pin) | 0xe000;
    uint8_t ret = 2;  // Retunrs 2 if no state change.
    if (self->state == 0xf000) {
        self->hit = true;
        ret = true;
    } else if (self->state == 0xefff) {
        self->hit = false;
        ret = false;
    }
    return ret;
}
