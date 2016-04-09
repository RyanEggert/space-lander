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
#include "int.h"

#define SET_STATE    0   // Vendor request that receives 2 unsigned integer values
#define GET_VALS    1   // Vendor request that returns 2 unsigned integer values 
#define GET_ROCKET_INFO 2  // Vendor request that returns rocket state, speed, and tilt

#define IDLE 0 //game states
#define RESET 1
#define FLYING 2

#define LANDED 0
#define CRASHED 1
//flying is already defined as 2
#define READY 3 //rocket has been zeroed


uint8_t RC_TXBUF[1024], RC_RXBUF[1024];

typedef void (*STATE_HANDLER_T)(void);

void idle(void);
void reset(void);
void flying(void);
void win(void);
void lose(void);

STATE_HANDLER_T state, last_state;


//Throttle and Orientation will be encoded in Value.

uint8_t trials;
uint16_t rocket_state, counter, coin;
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

void UART_ctl(uint8_t cmd, uint8_t value) {
    sprintf(tx_msg, "%01x%01x\r", value, cmd); //value could be state or command
    uart_puts(&uart1, tx_msg);
    if (cmd == GET_ROCKET_VALS) {
        uart_gets(&uart1, rec_msg, 64);
        uint32_t decoded_msg = (uint32_t)strtol(rec_msg, NULL, 16);
        rocket_speed = (uint16_t)((decoded_msg & 0xFF0000) >> 16);
        rocket_tilt = (uint16_t)((decoded_msg & 0xFF00) >> 8);
        rocket_state = decoded_msg & 0xFF;
    }
}

void setup_uart() {
    /*
    Configures UART for communications.
    Uses uart1 for inter-PIC communications. Rx on D[0], Tx on D[1].
    Automatically uses uart2 for stdout, stderr to PC via audio jack.
    */
    uart_open(&uart1, &D[1], &D[0], NULL, NULL, 19200., 'N', 1,
              0, RC_TXBUF, 1024, RC_RXBUF, 1024);
}

void idle(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        trials = 0;
        led_on(&led1);
    }

    coin = pin_read(&D[2]); //digital read of the coin acceptor.

    //Note: might be a good idea to add input conditioner later on.

    // Perform state tasks

    // Check for state transitions
    if (coin == 0) {
        state = reset;
    }

    if (state != last_state) {
        led_off(&led1);  // if we are leaving the state, do clean up stuff
    }
}

void reset(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        led_on(&led2);
    }

    // Perform state tasks

    // Check for state transitions

    if (trials == 3){
        state = idle;
    }

    if (rocket_state == READY) {
        state = flying;
    }

    if (state != last_state) {
        led_off(&led2);  // if we are leaving the state, do clean up stuff
    }
}

void flying(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
    }

    // Perform state tasks

    // Check for state transitions
    if (rocket_state == CRASHED) {
        state = lose;
    }

    if (rocket_state == LANDED){
        state = win;
    }

    if (state != last_state) {  // if we are leaving the state, do clean up stuff
    }
}

void lose(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        timer_start(&timer1);
        counter = 0;
    }

    if (timer_flag(&timer1)) {
        timer_lower(&timer1);
        counter++;
    }

    // Check for state transitions
    if (counter == 10) {
        state = reset;
    }

    if (state != last_state) {
        timer_stop(&timer1);
        trials++;  // if we are leaving the state, do clean up stuff
    }
}

void win(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        timer_start(&timer1);
        counter = 0;
    }

    if (timer_flag(&timer1)) {
        timer_lower(&timer1);
        counter++;
    }

    // Check for state transitions
    if (counter == 10) {
        state = reset;
    }

    if (state != last_state) {
        timer_stop(&timer1);
        trials++;  // if we are leaving the state, do clean up stuff
    }
}

void blue() {
    led_toggle(&led3);
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
    init_pin();
    init_int(); //initialize ext interrupt lib

    setup();
    pin_digitalIn(&D[2]);

    pin_digitalIn(&D[3]);
    int_attach(&int4, &D[3], 0, blue);

    InitUSB();
    U1IE = 0xFF; //setting up ISR for USB requests
    U1EIE = 0xFF;
    IFS5bits.USB1IF = 0; //flag
    IEC5bits.USB1IE = 1; //enable
    state = idle;
    last_state = (STATE_HANDLER_T)NULL;
    led_off(&led1);

    while (1) {
        state();
    }
}

