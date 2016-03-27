#include <p24FJ128GB206.h>
#include "config.h"
#include "common.h"
#include "pin.h"
#include "ui.h"
#include "timer.h"
#include "uart.h"
#include "i2c.h"
#include "servo.h"
#include "servo_test.h"
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
    init_i2c();

    init_servo_driver(&sd1, &i2c3, 16000., 0x0);
    init_servo(&orientation_servo, &sd1, 0);
    init_servo(&speed_ind_servo, &sd1, 1);
    setup();
    servo_driver_wake(&sd1);    // servo_driver_set_pwm_freq(&sd1, 60);
    servo_set(&orientation_servo, 1000, 0);
    servo_set(&speed_ind_servo, 3000, 0);
    uint16_t counter = 0;
    while (1) {
        if (timer_flag(&timer1)) {
            // Blink green light to show normal operation.
            timer_lower(&timer1);
            led_toggle(&led2);
            counter = (counter + 10) % 800;
            servo_set(&speed_ind_servo, 1000 + counter, 0);
            servo_set(&orientation_servo, 2000 - counter, 0);
        }
        if (timer_flag(&timer2)) {
            timer_lower(&timer2);
        }
        if (timer_flag(&timer3)) {
            timer_lower(&timer3);
            uint16_t switch2 = !sw_read(&sw2);
            uint16_t switch3 = !sw_read(&sw3);
            // led_write(&led1, switch2);
            led_write(&led3, switch3);

            pin_write(&D[10], switch2);
            pin_write(&D[11], switch3);

        }
    }
}
