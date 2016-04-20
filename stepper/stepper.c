#include <p24FJ128GB206.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "uart.h"
#include "config.h"
#include "common.h"
#include "ui.h"
#include "timer.h"
#include "spi.h"
// #include "pin.h"
#include "oc.h"
#include "usb.h"
// #include "md.h"
#include "stepper.h"

#define SET_VALS    0   // Vendor request that receives 2 unsigned integer values
#define GET_VALS    1   // Vendor request that returns 2 unsigned integer values 

// declare D I/O pins
_PIN *STEP, *DIR, *SLP, *ENABLE, *PFD, *RST, *MS1, *MS2;

_PIN *LEFT, *RIGHT;

typedef void (*STATE_HANDLER_T)(void);

void wait(void);
void drive(void);

STATE_HANDLER_T state, last_state;
uint16_t counter = 0;
uint16_t countLimit = 2500;
uint16_t dir = 0;
uint8_t timerCounter = 0;
uint8_t timerCountLimit = 20;
uint8_t timerDir = 1;
uint8_t timerIncr = 1;
uint16_t val1, val2;

_LED *green_led, *red_led, *blue_led;

void wait(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        pin_toggle(DIR);
        led_on(red_led);
    }

    // Perform state tasks

    // Check for state transitions
    if (timer_flag(&timer2)) {
        timer_lower(&timer2);
        counter = counter + 1;
    }
    if (counter > countLimit) {
        state = drive;
        counter = 0;
    }


    if (state != last_state) {  // if we are leaving the state, do clean up stuff
        led_off(red_led);
    }
}

void drive(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        led_on(green_led);
    }

    // Perform state tasks
    // Step motor
    if (timer_flag(&timer2)) {
        timer_lower(&timer2);
        // if (timerCounter >= timerCountLimit) {
        //     timerCounter = 0;
            pin_toggle(STEP);
        //     if (timerDir == 1) {
        //         // switch timerDir
        //         if (timerCountLimit <= 0) {
        //             timerCountLimit = 490;
        //             timerDir = 0;                    
        //         }
        //         else {
        //             timerCountLimit = timerCountLimit + timerIncr;
        //         }
        //     }
        //     else if (timerDir == 0) {
        //         // switch timerDir
        //         if (timerCountLimit >= 500) {
        //             timerCountLimit = 490;
        //             timerDir = 1;                    
        //         }
        //         else {
        //             timerCountLimit = timerCountLimit - timerIncr;
        //         }
        //     }
        // }
        // else {
        //     timerCounter = timerCounter + 1;
        // }
        counter = counter + 1;
    // }
    if (counter > countLimit) {
        state = wait;
        counter = 0;
    }

    // Check for state transitions

    if (state != last_state) {  // if we are leaving the state, do clean up stuff
        led_off(green_led);
    }
}
}

void VendorRequests(void) {
    WORD temp;
    switch (USB_setup.bRequest) {
        case SET_VALS:
            state = USB_setup.wValue.w;
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case GET_VALS:
            temp.w = val1;
            BD[EP0IN].address[0] = temp.b[0];
            BD[EP0IN].address[1] = temp.b[1];
            temp.w = val2;
            BD[EP0IN].address[2] = temp.b[0];
            BD[EP0IN].address[3] = temp.b[1];
            BD[EP0IN].bytecount = 4;    // set EP0 IN byte count to 4
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;            
        default:
            USB_error_flags |= 0x01;    // set Request Error Flag
    }
}

void VendorRequestsIn(void) {
    switch (USB_request.setup.bRequest) {
        default:
            USB_error_flags |= 0x01;                    // set Request Error Flag
    }
}

void VendorRequestsOut(void) {
    switch (USB_request.setup.bRequest) {
        default:
            USB_error_flags |= 0x01;                    // set Request Error Flag
    }
}

int16_t main(void) {
    init_clock();
    init_ui();
    init_spi();
    init_oc();
    init_pin();
    init_uart();
    init_timer();
    // init_md();
    init_st();

    LEFT = &D[12];
    RIGHT = &D[13];

    timer_setPeriod(&timer2, .001);
    timer_start(&timer2);
    timer_setPeriod(&timer3, 1.0);
    timer_start(&timer3);

    // // Comment these pins later
    // STEP = &D[0];
    // DIR = &D[1];
    // SLP = &D[2];
    // ENABLE = &D[3];
    // // PFD not initializing properly...
    // PFD = &D[4];
    // RST = &D[5];
    // MS1 = &D[6];
    // MS2 = &D[7];

    // pin_digitalOut(STEP);
    // pin_digitalOut(DIR);
    // pin_digitalOut(SLP);
    // pin_digitalOut(ENABLE);
    // pin_digitalOut(PFD);
    // pin_digitalOut(RST);
    // pin_digitalOut(MS1);
    // pin_digitalOut(MS2);

    // pin_clear(STEP);
    // pin_set(DIR);
    // pin_set(SLP);
    // pin_clear(ENABLE);
    // pin_clear(PFD);
    // pin_set(RST);
    // pin_clear(MS1);
    // pin_clear(MS2);

    // InitUSB();                              // initialize the USB registers and serial interface engine
    // while (USB_USWSTAT!=CONFIG_STATE) {     // while the peripheral is not configured...
    //     ServiceUSB();                       // ...service USB requests
    // }

    green_led = &led2;
    red_led = &led1;
    blue_led = &led3;

    state = wait;
    last_state = (STATE_HANDLER_T)NULL;

    float freq = 100.0;
    float freq_lim = 1000.0;
    uint8_t dir = 0;
    float accel = 10.0;
    st_state(&st_d, 1);

    while (1) {
        // state();
        if (timer_flag(&timer3)) {
            timer_lower(&timer3);
            if (pin_read(LEFT) || pin_read(RIGHT)) {
                if (freq < freq_lim) {
                    freq += accel;
                }
            }
            else {
                if (freq != 100.0) {
                    freq = 100.0;
                }
            }
        }
        if (timer_flag(&timer2)) {
            timer_lower(&timer2);
            if (pin_read(LEFT)) {
                // if (freq < freq_lim) {
                //     freq = freq+accel;
                // }
                if (dir == 1) {
                    st_direction(&st_d, 0);
                    dir = 0;
                }
                st_speed(&st_d, freq);
                led_on(&led1);
                led_off(&led3);
            }
            else if (pin_read(RIGHT)) {
                // if (freq < freq_lim) {
                //     freq = freq+accel;
                // }
                if (dir == 0) {
                    st_direction(&st_d, 1);
                    dir = 1;
                }
                // st_direction(&st_d, 1);
                st_speed(&st_d, freq);
                led_off(&led1);
                led_on(&led3);
            }
            else {
                // if (freq != 100.0) {
                //     freq = 100.0;
                // }
                st_speed(&st_d, 0);
                led_off(&led1);
                led_off(&led3);
            }
        }
    }

}
