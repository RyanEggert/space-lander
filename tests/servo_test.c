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

#define SERVOMIN 500
#define SERVOMAX 1800
void setup() {
    printf("START\n\r");
    timer_setPeriod(&timer1, .55);  // Timer for LED operation/status blink
    timer_setPeriod(&timer2, 0.5);
    timer_setPeriod(&timer3, 0.01);
    timer_setPeriod(&timer4, 0.001);
    timer_start(&timer1);
    timer_start(&timer2);
    timer_start(&timer3);
    timer_start(&timer4);

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
    uint16_t servo_test [] = {SERVOMIN, SERVOMAX};
    uint16_t servo_test2 [] = {1000, 1500};
    init_servo_driver(&sd1, &i2c3, 16000., 0x0);
    init_servo(&orientation_servo, &sd1, 0);
    init_servo(&speed_ind_servo, &sd1, 1);
    init_servo(&servo0, &sd1, 2);
    init_servo(&servo1, &sd1, 3);
    setup();
    servo_driver_wake(&sd1);    // servo_driver_set_pwm_freq(&sd1, 60);
    // servo_set(&orientation_servo, 1000, 0);
    // servo_set(&speed_ind_servo, 3000, 0);
    uint16_t counter = 0;
    uint16_t counter2 = 0;
    uint16_t servo_cmd = 500;
    while (1) {
        if (timer_flag(&timer1)) {
            // Blink green light to show normal operation.
            timer_lower(&timer1);
            led_toggle(&led2);
            // counter = (counter + 10) % 800;
            // counter = (counter + 1) % 3;
            // servo_set(&orientation_servo, servo_test[counter], 0);
            // servo_set(&orientation_servo, 2000 - counter, 0);
            // servo_set(&servo0, 1000 + counter, 0);
            // servo_set(&servo0, 1000 + counter, 0);
        }
        if (timer_flag(&timer2)) {
            timer_lower(&timer2);
            counter = (counter +1) % 2;
            servo_set(&speed_ind_servo, servo_test[counter], 0);
        }
        if (timer_flag(&timer3)) {
            timer_lower(&timer3);
            servo_cmd += 5;
            if (servo_cmd > SERVOMAX) {
                servo_cmd = SERVOMIN ;
            }

            servo_set(&orientation_servo, servo_cmd, 0);
            // printf("servo: %d\n\r", servo_cmd);
        }
        if (timer_flag(&timer4)) {
            timer_lower(&timer4);
            counter2 = (counter2 +1) % 2;
            servo_set(&servo0, servo_test2[counter], 0);
        }
    }
}
