#include <p24FJ128GB206.h>
#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "common.h"
#include "pin.h"
#include "ui.h"
#include "timer.h"
#include "uart.h"
#include "usb.h"
#include "msgs.h"

uint8_t MC_TXBUF[1024], MC_RXBUF[1024];

#define SET_STATE    0   // Vendor request that receives 2 unsigned integer values
#define GET_VALS    1   // Vendor request that returns 2 unsigned integer values 
#define GET_ROCKET_INFO 2
#define DEBUG_UART_BUFFERS 3

uint16_t rocket_state;
uint16_t rocket_speed, rocket_tilt;
uint8_t throttle, tilt; //commands
uint8_t rocketstuff[64], rec_msg[64];
uint8_t cmd, value;
uint16_t val1, val2;

void VendorRequests(void) {
    WORD temp;
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


void UARTrequests() {
    uart_gets(&uart1, rec_msg, 64);
    uint32_t decoded_msg = (uint32_t)strtol(rec_msg, NULL, 16);
    cmd = decoded_msg & 0x0f;
    value = (decoded_msg & 0xf0) >> 4;
    switch (cmd) {
    case GET_ROCKET_VALS:
        //speed, orientation
        sprintf(rocketstuff, "%02x%02x%02x\r", rocket_speed, rocket_tilt, rocket_state);
        uart_puts(&uart1, rocketstuff);
        break;
    case SET_ROCKET_STATE:
        rocket_state = value;
        break;
    case SEND_ROCKET_COMMANDS:
        throttle = value & 0b01;
        tilt = (value & 0b10) >> 1;
        break;
    }
}

void setup_uart() {
    /*
    Configures UART for communications.
    Uses uart1 for inter-PIC communications. Rx on D[1], Tx on D[0].
    Automatically uses uart2 for stdout, stderr to PC via audio jack.
    */
    uart_open(&uart1, &D[0], &D[1], NULL, NULL, 19200., 'N', 1,
              0, MC_TXBUF, 1024, MC_RXBUF, 1024);
}

void setup() {
    timer_setPeriod(&timer1, 1);  // Timer for LED operation/status blink
    timer_setPeriod(&timer2, 0.5);  // Timer for UART servicing
    timer_start(&timer1);
    timer_start(&timer2);

    setup_uart();
    throttle, tilt = 0;
    val1, val2 = 8;
    rocket_tilt = 15;

}

int16_t main(void) {
    // printf("Starting Rocket Controller...\r\n");
    init_clock();
    init_ui();
    init_timer();
    init_uart();
    setup();
    uint16_t counter = 0;
    uint64_t msg;
    char is_recip = 0;

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
            UARTrequests();
        }
    }
}
