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
        uart_gets(&uart1, rec_msg, 64);
        led_toggle(&led3);
        uint64_t decoded_msg = (uint64_t)strtoll(rec_msg, NULL, 16);
        rocket_speed = (decoded_msg & 0xffff00000000) >> 32;
        rocket_tilt = (decoded_msg & 0x0000ffff0000) >> 16;
        rocket_state = decoded_msg & 0x00000000ffff;
    }
}

void setup_uart() {
    // uart_open(&uart3, &AJTX, &AJRX, NULL, NULL, 115200., 'N', 1, 
    //           // 0, TXBUF, 1024, RXBUF, 1024);

    // uart_open(_UART *self, _PIN *TX, _PIN *RX, _PIN *RTS, _PIN *CTS, 
    //            float baudrate, int8_t parity, int16_t stopbits, 
    //            uint16_t TXthreshold, uint8_t *TXbuffer, uint16_t TXbufferlen, 
    //            uint8_t *RXbuffer, uint16_t RXbufferlen)

    // uart_open(&uart2, &D[0], &D[1], NULL, NULL, 115200., 'N', 1, 
    //           0, RC_TXBUF, 1024, RC_RXBUF, 1024);
    // uart_open(&uart3, &AJTX, &AJRX, NULL, NULL, 115200., 'N', 1, 
    //           0, TXBUF, 1024, RXBUF, 1024);

    pin_init(&AJTX, (uint16_t *)&PORTG, (uint16_t *)&TRISG, 
             (uint16_t *)NULL, 6, -1, 8, 21, (uint16_t *)&RPOR10);
    pin_init(&AJRX, (uint16_t *)&PORTG, (uint16_t *)&TRISG, 
             (uint16_t *)NULL, 7, -1, 0, 26, (uint16_t *)&RPOR13);
    uart_open(&uart1, &AJTX, &AJRX, NULL, NULL, 19200., 'N', 1, 
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
            timer_lower(&timer2);
            counter ++;
            // sprintf(status_msg, "This is message #%d.\r", counter);
            // uint8_t addr = counter % 4;
            // uint8_t cmd = 28;
            // sprintf(status_msg, "%02x%02x\r", cmd, addr);
            // uart_puts(&uart1, status_msg);
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
