#include <p24FJ128GB206.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "config.h"
#include "common.h"
#include "pin.h"
#include "ui.h"
#include "timer.h"
#include "uart.h"
#include "oc.h"
#include "md.h"

_PIN *MOTOR_DIR;
_PIN *FWD, *REV;


void setup() {
    timer_setPeriod(&timer1, 1);  // Timer for LED operation/status blink
    timer_setPeriod(&timer3, .01);
    timer_start(&timer1);
    timer_start(&timer3);

}

int16_t main(void) {
    init_clock();
    init_ui();
    init_timer();
    init_uart();
    init_pin();
    init_oc();
    // ***init_md:  may need to change md.c so it utilizes less DIO pins
    init_md();
    setup();

    uint16_t counter = 0;
    uint8_t rec_msg [64];
    uint64_t msg;
    char is_recip = 0;

    // Initialize motor vars/pins
    uint16_t MOTOR_SPEED = 0xC000;
    uint16_t MOTOR_DIR_TRACK = 0;   // tracks vert dir of rocket
    MOTOR_DIR = &D[0];
    FWD = &D[2];
    REV = &D[13];
    pin_digitalOut(MOTOR_DIR);

    uint8_t direction = 1;

    const uint16_t MOTOR_SPEED_LIMIT = 0xFFF0;
    const uint16_t GRAV_VAL = 0x00F0;
    const uint16_t THRUST_VAL = GRAV_VAL * 2;
    uint16_t mot_speed = 0;
    int mot_dir = 0;
    pin_clear(MOTOR_DIR);
    while (1) {
        if (timer_flag(&timer3)) {
            timer_lower(&timer3);
            // Handle rocket thrust
            if (pin_read(FWD)) { // Thrust on
                if (MOTOR_DIR_TRACK == 0) {  // rocket falling
                    if (MOTOR_SPEED > THRUST_VAL) {  // nonzero velocity
                        MOTOR_SPEED = MOTOR_SPEED - THRUST_VAL;
                    }
                    else {  // zero velocity
                        MOTOR_DIR_TRACK = 1;
                        MOTOR_SPEED = 0;
                    }
                }
                else { // rocket rising
                    if (MOTOR_SPEED < MOTOR_SPEED_LIMIT) {
                        MOTOR_SPEED = MOTOR_SPEED + THRUST_VAL;
                    }
                }
                // pin_set(MOTOR_DIR);
                // if (MOTOR_SPEED < MOTOR_SPEED_LIMIT) {
                //     MOTOR_SPEED = MOTOR_SPEED + THRUST_VAL;
                // }
                // md_speed(&mdp, MOTOR_SPEED);
                led_on(&led2);
            }
            else { // no thrust
                if (MOTOR_DIR_TRACK == 0) { // rocket falling
                    if (MOTOR_SPEED < MOTOR_SPEED_LIMIT) {
                        MOTOR_SPEED = MOTOR_SPEED + GRAV_VAL;
                    }
                }
                else {  // rocket rising
                    if (MOTOR_SPEED > THRUST_VAL) {
                        MOTOR_SPEED = MOTOR_SPEED - GRAV_VAL;
                    }
                    else {
                        MOTOR_DIR_TRACK = 0;
                        MOTOR_SPEED = GRAV_VAL;
                    }
                }
                led_off(&led2);
            }
        }
        if (MOTOR_DIR_TRACK) {
            pin_set(MOTOR_DIR);
        }
        else {
            pin_clear(MOTOR_DIR);
        }
        md_speed(&mdp, MOTOR_SPEED);
    }
}
