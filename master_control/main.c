#include <p24FJ128GB206.h>
#include "config.h"
#include "common.h"
#include "pin.h"
#include "ui.h"
#include "timer.h"
#include "uart.h"
#include "usb.h"
#include <stdio.h>
#include <stdlib.h>

uint8_t RC_TXBUF[1024], RC_RXBUF[1024];

#define GET_ROCKET_VALS 0
#define SET_ROCKET_STATE 1
#define SEND_ROCKET_COMMANDS 2

#define IDLE 0
#define RESET 1
#define FLYING 2

#define TILT_LEFT 1
#define TILT_RIGHT 2
#define NO_TILT 0

#define SET_STATE    0   // Vendor request that receives 2 unsigned integer values
#define GET_VALS    1   // Vendor request that returns 2 unsigned integer values 
#define GET_ROCKET_INFO 2  // Vendor request that returns rocket state, speed, and tilt

//Throttle and Orientation will be encoded in Value.

uint16_t rocket_state;
uint16_t rocket_speed, rocket_tilt;
uint8_t rec_msg[64], tx_msg[64];

void VendorRequests(void) {
    WORD temp;
    switch (USB_setup.bRequest) {
        case SET_STATE:
            // state = USB_setup.wValue.w;
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case GET_VALS:
            temp.w = rocket_tilt;
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

void UART_ctl(uint8_t cmd, uint8_t value){
    sprintf(tx_msg, "%01x%01x\r", value, cmd); //value could be state or command
    uart_puts(&uart1, tx_msg);
    if (cmd == GET_ROCKET_VALS){
        // led_toggle(&led2);
        uart_gets(&uart1, rec_msg, 128);
        led_toggle(&led3);
        uint32_t decoded_msg = (uint32_t)strtol(rec_msg, NULL, 16);
        // printf("RM: %s \n\r", rec_msg);
        // printf("DM: %d \n\r", decoded_msg);
        rocket_speed = (uint16_t)((decoded_msg & 0xffff00000000) >> 32);
        rocket_tilt = (uint16_t)((decoded_msg & 0x0000ffff0000) >> 16);
        // rocket_state = (uint16_t)(decoded_msg & 0x00000000ffff);
        rocket_state = decoded_msg;
    }
}

void setup_uart() {    
    uart_open(&uart1, &D[1], &D[0], NULL, NULL, 19200., 'N', 1, 
              0, RC_TXBUF, 1024, RC_RXBUF, 1024);
}

void setup() {
    timer_setPeriod(&timer1, 1);  // Timer for LED operation/status blink
    timer_setPeriod(&timer2, 0.5); 
    timer_start(&timer1);
    timer_start(&timer2);

    setup_uart();
    rocket_tilt, rocket_speed = 0;
}

int16_t main(void) {
    // printf("Starting Master Controller...\r\n");
    init_clock();
    init_ui();
    init_timer();
    init_uart();
    setup();
    uint16_t counter = 0;
    uint8_t status_msg [64];
    led_off(&led2);
    led_off(&led3);

    InitUSB();
    U1IE = 0xFF; //setting up ISR for USB requests
    U1EIE = 0xFF;
    IFS5bits.USB1IF = 0; //flag
    IEC5bits.USB1IE = 1; //enable

    while (1) {
        if (timer_flag(&timer1)) {
            // Blink green light to show normal operation.
            timer_lower(&timer1);
            led_toggle(&led1);
        }
        if (timer_flag(&timer2)) {
            // Transmit UART data

            // UART_ctl(SEND_ROCKET_COMMANDS, 0b11);
            // UART_ctl(SET_ROCKET_STATE, IDLE);
            UART_ctl(GET_ROCKET_VALS,0b0);
            
            // if (rocket_state == 15) {
            //     led_on(&led1);
            // }

            // else{
            //     led_off(&led1);
            // }

            // if(rocket_speed ==15){
            //     led_on(&led2);
            // }

            // else{
            //     led_off(&led2);
            // }

             if(rocket_tilt==15){
                led_on(&led2);
            }

            else{
                led_off(&led2);
            }
       
       
        }   
    }
}
