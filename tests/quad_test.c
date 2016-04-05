#include <p24FJ128GB206.h>
#include "config.h"
#include "common.h"
#include "pin.h"
#include "ui.h"
#include "timer.h"
#include "uart.h"
#include "quad.h"
// #include "quad_test.h"
#include <stdio.h>
#include <stdlib.h>

void setup() {
    printf("START\n\r");
    timer_setPeriod(&timer1, .55);  // Timer for LED operation/status blink
    timer_setPeriod(&timer2, 0.5);
    timer_setPeriod(&timer3, 0.01);
    timer_start(&timer1);
    timer_start(&timer2);
    timer_start(&timer3);

    // General use debugging output pin
    pin_digitalOut(&D[2]);
}

int16_t main(void) {
    // printf("Starting Rocket Controller...\r\n");
    init_clock();
    init_ui();
    init_timer();
    init_uart();
    init_quad();
    quad_init(&quad1, &D[8], &D[9]);
    quad_every(&quad1, &timer5, 0.0000875);
    // quad_every(&quad1, &timer5, 1.00);
    setup();
    uint16_t counter = 0;
    while (1) {
        if (timer_flag(&timer1)) {
            // Blink green light to show normal operation.
            timer_lower(&timer1);
            led_toggle(&led2);
            // printf("BLINK\n\r");
        }
        if (timer_flag(&timer2)) {
            timer_lower(&timer2);
            // printf("12 13: %d %d\n\r", pin_read(&D[8]), pin_read(&D[9]));
            if (quad1.counter > 50) {
                led_on(&led1);
            } else {
                led_off(&led1);
            }

            if (quad1.overflow < 0) {
                led_on(&led3);
            } else {
                led_off(&led3);
            }
        }
        if (timer_flag(&timer3)) {

            timer_lower(&timer3);
            // printf("S.C: %d\n\r", quad1.count);
        }
    }
}
