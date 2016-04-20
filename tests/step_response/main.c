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
#include "main.h"
#include "usb.h"
#include "oc.h"
#include "dcm.h"
#include "pid.h"
#include "msgs.h"

uint8_t RC_TXBUF[1024], RC_RXBUF[1024];

#define SET_STATE    0   // Vendor request that receives 2 unsigned integer values
#define GET_VALS    1   // Vendor request that returns 2 unsigned integer values 
#define GET_ROCKET_INFO 2
#define DEBUG_UART_BUFFERS 3
#define GET_QUAD_INFO 4
#define COMMAND_DCMOTOR 5

uint16_t rocket_state;
uint16_t rocket_speed, rocket_tilt;
uint8_t throttle, tilt; //commands
uint8_t rocketstuff[64], rec_msg[64];
uint8_t cmd, value;
uint16_t val1, val2;

void VendorRequests(void) {
    disable_interrupts();
    WORD temp;
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
        temp.w = rocket_state;
        BD[EP0IN].address[4] = temp.b[0];
        BD[EP0IN].address[5] = temp.b[1];
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
        temp.w = quad1.meas_speed;
        BD[EP0IN].address[6] = temp.b[0];
        BD[EP0IN].address[7] = temp.b[1];
        BD[EP0IN].bytecount = 8;    // set EP0 IN byte count to 4
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        break;
    case COMMAND_DCMOTOR:
        dcm_velocity(&dcm1, USB_setup.wValue.w, USB_setup.wIndex.w);
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
    timer_setPeriod(&timer2, 0.01);  // Timer for UART servicing
    timer_setPeriod(&timer3, 0.01);
    timer_start(&timer1);
    timer_start(&timer2);
    timer_start(&timer3);

    // DC MOTOR + QUAD ENCODER
    dcm_init(&dcm1, &D[10], &D[11], 1e3, 0, &oc7);
    quad_init(&quad1, &D[8], &D[9]); // quad1 uses pins D12 & D13
    quad_every(&quad1, &timer5, 0.0000875); // quad1 will use timer5 interrupts
    
    // General use debugging output pin
    pin_digitalOut(&D[2]);
}

int16_t main(void) {
    // printf("Starting Rocket Controller...\r\n");
    init_clock();
    init_ui();
    init_pin();
    init_timer();
    init_uart();
    init_quad();
    init_oc();
    init_dcm();
    setup();
    // oc_pwm(&oc1, &D[4], &timer4, 3000, 32000);
    uint16_t counter = 0;
    uint64_t msg;
    char is_recip = 0;
    uint32_t prev_count = quad1.counter;
    int32_t diff_count;
    uint32_t curr_count;
    float dcm1_speed;
    InitUSB();
    U1IE = 0xFF; //setting up ISR for USB requests
    U1EIE = 0xFF;
    IFS5bits.USB1IF = 0; //flag
    IEC5bits.USB1IE = 1; //enable
    while (1) {

        if (timer_flag(&timer1)) {
            // Blink green light to show normal operation.
            timer_lower(&timer1);
            led_toggle(&led2);
        }

        if (timer_flag(&timer2)) {
            timer_lower(&timer2);
            led_toggle(&led1);
            curr_count = quad1.counter;
            diff_count = curr_count - prev_count;
            prev_count = curr_count;
            // dcm1_speed = quad_meas_speed(&quad1, 0.5);
            quad1.meas_speed = abs(diff_count);
            // printf("Diff: %d\n\r", diff_count);
        }
    }
}
