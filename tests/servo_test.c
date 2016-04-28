#include <p24FJ128GB206.h>
#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "common.h"
#include "pin.h"
#include "ui.h"
#include "timer.h"
#include "usb.h"
#include "i2c.h"
#include "servo.h"
#include "servo_test.h"


uint8_t RC_TXBUF[1024], RC_RXBUF[1024];

#define SET_STATE    0   // Vendor request that receives 2 unsigned integer values
#define GET_VALS    1   // Vendor request that returns 2 unsigned integer values 
#define GET_ROCKET_INFO 2
#define DEBUG_UART_BUFFERS 3
#define GET_QUAD_INFO 4
#define COMMAND_DCMOTOR 5

#define DEBUG_SERVO_SET_POS 60
#define DEBUG_SERVO_SET_FREQ 61

uint16_t rocket_state;
uint16_t rocket_speed, rocket_tilt;
uint8_t throttle, tilt; //commands
uint8_t rocketstuff[64], rec_msg[64];
uint8_t cmd, value;
uint16_t val1, val2;

void VendorRequests(void) {
    disable_interrupts();
    WORD temp;
    WORD temp2;
    WORD32 temp32;
    switch (USB_setup.bRequest) {
    case SET_STATE:
        // state = USB_setup.wValue.w;
        BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        break;

    case GET_ROCKET_INFO:
        temp.w = rocket_tilt;
        BD[EP0IN].address[0] = temp.b[0];
        BD[EP0IN].address[1] = temp.b[1];
        temp.w = rocket_speed;
        BD[EP0IN].address[2] = temp.b[0];
        BD[EP0IN].address[3] = temp.b[1];
        temp.w = rocket_state;
        BD[EP0IN].address[4] = temp.b[0];
        BD[EP0IN].address[5] = temp.b[1];
        BD[EP0IN].bytecount = 6;    // set EP0 IN byte count to 4
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        break;

    case DEBUG_SERVO_SET_POS:
        temp.w = USB_setup.wValue.w;   // Commanded position
        temp2.w = USB_setup.wIndex.w;  // Servo driver index
        servo_usb_set(&sd1, temp2.b[0], temp.w);
        BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        break;

    case DEBUG_SERVO_SET_FREQ:
        temp.w = USB_setup.wValue.w;
        
        servo_driver_configure(&sd1, ((float)(temp.w)) / 10);
        BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        break;

    default:
        USB_error_flags |= 0x01;    // set Request Error Flag
    }
    enable_interrupts();
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

void setup() {
    timer_setPeriod(&timer1, 1);  // Timer for LED operation/status blink
    // timer_setPeriod(&timer2, 0.01);  // Timer for UART servicing
    // timer_setPeriod(&timer3, 0.01);
    timer_start(&timer1);
    // timer_start(&timer2);
    // timer_start(&timer3);
}

int16_t main(void) {
    // printf("Starting Rocket Controller...\r\n");
    init_clock();
    init_ui();
    init_pin();
    init_timer();
    init_i2c();
    setup();
    init_servo_driver(&sd1, &i2c3, 16000., 0x0);
    init_servo(&servo4, &sd1, 0);
    InitUSB();
    U1IE = 0xFF; //setting up ISR for USB requests
    U1EIE = 0xFF;
    IFS5bits.USB1IF = 0; //flag
    IEC5bits.USB1IE = 1; //enable
    // uint32_t pid_command;
    servo_set(&servo4, 1500, 0);
    while (1) {
        if (timer_flag(&timer1)) {
            // Blink green light to show normal operation.
            timer_lower(&timer1);
            led_toggle(&led2);
            // servo_set(&servo4, 1500, 0);
        }
    }
}
