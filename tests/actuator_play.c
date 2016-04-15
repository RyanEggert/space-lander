#include <p24FJ128GB206.h>
#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "common.h"
#include "pin.h"
#include "ui.h"
#include "timer.h"
#include "uart.h"
#include "quad.h"
#include "i2c.h"
#include "servo.h"
#include "stepper.h"
#include "usb.h"
#include "oc.h"
#include "dcm.h"

uint8_t RC_TXBUF[1024], RC_RXBUF[1024];

// Handle flying state
#define LANDED 0
#define CRASHED 1
//flying is already defined as 2
#define READY 3 //rocket has been zeroed
#define TESTING 4

#define SET_STATE    0   // Vendor request that receives 2 unsigned integer values
#define GET_VALS    1   // Vendor request that returns 2 unsigned integer values 
#define GET_ROCKET_INFO 2
#define DEBUG_UART_BUFFERS 3
#define GET_QUAD_INFO 4
#define COMMAND_DCMOTOR 5  // Ven. req. to set dir/speed of DC motor

#define DEBUG_SERVO_SET_POS 60  // Ven. req. to set position of specified servo
#define DEBUG_SERVO_SET_FREQ 61  // Ven. req. to set pwm freq. of driver board

// endstop pins
_PIN *Y_END_TOP, *Y_END_BOT, *X_END_L, *X_END_R;

uint16_t rocket_speed, rocket_tilt;
uint16_t counter;
uint8_t throttle, tilt; //commands
uint8_t rocketstuff[64], rec_msg[64];
uint8_t cmd, value;
uint16_t val1, val2;

// kinematic model vals
uint16_t thrust_val = 0x0100;
uint16_t grav_val = 0x0100;

// stepper vars
uint16_t stepper_count = 0;
uint16_t stepper_lim = 1000;
uint8_t stepper_state = 0;  // 0 = drive to X_END_L, 1 = drive to middle, 2 = stop
float stepper_speed = 0;
uint16_t stepper_speed_int;

// dc motor vars
uint16_t motor_state;
uint16_t motor_dir_track = 0;
uint16_t motor_speed = 0;
uint16_t motor_speed_limit = 0x7FFF;
uint16_t motor_deadband = 19000;  // will find once gantry is built

// tilt vals (maxes out at 4096; may use smaller range)
uint16_t tilt_max = 4095;
uint16_t tilt_min = 0;

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
    case DEBUG_UART_BUFFERS:
        temp.w = uart1.TXbuffer.head;
        BD[EP0IN].address[0] = temp.b[0];
        BD[EP0IN].address[1] = temp.b[1];
        temp.w = uart1.TXbuffer.tail;
        BD[EP0IN].address[2] = temp.b[0];
        BD[EP0IN].address[3] = temp.b[1];
        temp.w = uart1.TXbuffer.count;
        BD[EP0IN].address[4] = temp.b[0];
        BD[EP0IN].address[5] = temp.b[1];

        temp.w = uart1.RXbuffer.head;
        BD[EP0IN].address[6] = temp.b[0];
        BD[EP0IN].address[7] = temp.b[1];
        temp.w = uart1.RXbuffer.tail;
        BD[EP0IN].address[8] = temp.b[0];
        BD[EP0IN].address[9] = temp.b[1];
        temp.w = uart1.RXbuffer.count;
        BD[EP0IN].address[10] = temp.b[0];
        BD[EP0IN].address[11] = temp.b[1];
        BD[EP0IN].bytecount = 12;    // set EP0 IN byte count to 4
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        break;
    case GET_ROCKET_INFO:
        temp.w = rocket_tilt;
        BD[EP0IN].address[0] = temp.b[0];
        BD[EP0IN].address[1] = temp.b[1];
        temp.w = rocket_speed;
        BD[EP0IN].address[2] = temp.b[0];
        BD[EP0IN].address[3] = temp.b[1];
        temp.w = motor_dir_track;
        BD[EP0IN].address[4] = temp.b[0];
        BD[EP0IN].address[5] = temp.b[1];
        temp.w = motor_speed;
        BD[EP0IN].address[6] = temp.b[0];
        BD[EP0IN].address[7] = temp.b[1];
        temp.w = stepper_speed;
        BD[EP0IN].address[8] = temp.b[0];
        BD[EP0IN].address[9] = temp.b[1];
        BD[EP0IN].bytecount = 6;    // set EP0 IN byte count to 4
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        break;
    case GET_QUAD_INFO:
        temp32.ul = quad1.counter;
        BD[EP0IN].address[0] = temp32.b[0];
        BD[EP0IN].address[1] = temp32.b[1];
        BD[EP0IN].address[2] = temp32.b[2];
        BD[EP0IN].address[3] = temp32.b[3];
        temp.w = quad1.overflow;
        BD[EP0IN].address[4] = temp.b[0];
        BD[EP0IN].address[5] = temp.b[1];
        BD[EP0IN].bytecount = 6;    // set EP0 IN byte count to 4
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        break;
    case COMMAND_DCMOTOR:
        dcm_velocity(&dcm1, USB_setup.wValue.w, USB_setup.wIndex.w);
        BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
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
    timer_setPeriod(&timer2, 0.5);  // Timer for UART servicing
    timer_setPeriod(&timer3, 0.01);
    timer_start(&timer1);
    timer_start(&timer2);
    timer_start(&timer3);

    // // DC MOTOR + QUAD ENCODER
    // dcm_init(&dcm1, &D[10], &D[11], 1e3, 0, &oc7);
    // quad_init(&quad1, &D[8], &D[9]); // quad1 uses pins D8 & D9
    // quad_every(&quad1, &timer5, 0.0000875); // quad1 will use timer5 interrupts

    // General use debugging output pin
    // pin_digitalOut(&D[2]);

    // // Init endstop switches
    // Y_END_TOP = &D[4];
    // Y_END_BOT = &D[12];
    // X_END_L = &D[6];
    // X_END_R = &D[7];

    // pin_digitalIn(Y_END_TOP);
    // pin_digitalIn(Y_END_BOT);
    // pin_digitalIn(X_END_L);
    // pin_digitalIn(X_END_R);

    throttle, tilt = 0;
    val1, val2 = 8;
    rocket_tilt = 500;

}

int16_t main(void) {
    // printf("Starting Rocket Controller...\r\n");
    init_clock();
    init_ui();
    init_oc();
    init_pin();
    init_timer();
    // init_st();      // if this is first, then D[1] - D[3] don't work as outputs
    // init_quad();
    // init_stepper();
    // init_dcm();
    init_i2c();
    setup();
    init_servo_driver(&sd1, &i2c3, 16000., 0x0);
    led_on(&led3);
    // init_servo(&servo0, &sd1, 0); Only necessary for custom-named servos
    // oc_pwm(&oc1, &D[4], &timer4, 3000, 32000);led_on(&led3);
    InitUSB();
    U1IE = 0xFF; //setting up ISR for USB requests
    U1EIE = 0xFF;
    IFS5bits.USB1IF = 0; //flag
    IEC5bits.USB1IE = 1; //enable


    // dcm_velocity(&dcm1, 64000, 1);
    while (1) {
        if (timer_flag(&timer1)) {
            // Blink green light to show normal operation.
            timer_lower(&timer1);
            led_toggle(&led2);
            led_toggle(&led1);
        }
    }
}
