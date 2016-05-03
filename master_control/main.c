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

#define SET_STATE    0   // Vendor request that receives 2 unsigned integer values
#define GET_VALS    1   // Vendor request that returns 2 unsigned integer values 
#define GET_ROCKET_INFO 2 // Vendor request that returns rocket state, speed, and tilt
#define DEBUG_UART_BUFFERS 3
#define DEBUG_UART_STATUS 70

#define IDLE 0 //game states
#define RESET 1
#define FLYING 2

#define LANDED 0
#define CRASHED 1
//flying is already defined as 2
#define READY 3 //rocket has been zeroed

_PIN *LEFT, *RIGHT, *THROTTLE;

uint8_t TXBUF1[1024], RXBUF1[1024];
uint8_t TXBUF2[1024], RXBUF2[1024];
char rx_msg[64], tx_msg[64];

typedef void (*STATE_HANDLER_T)(void);

void idle(void);
void reset(void);
void flying(void);
void win(void);
void lose(void);

STATE_HANDLER_T state, last_state;

//Throttle and Orientation will be encoded in Value.

uint8_t trials;
uint16_t rocket_state = READY;
uint16_t counter, coin;
uint16_t rocket_speed, rocket_tilt;
uint16_t throttle, tilt;

void uart_sendstr(uint8_t *str) {
    /*
    Sends a string on uart1. UNTESTED
    */
    printf("SENDING: %s\n\r", tx_msg);
    uart_puts(&uart1, str);
}

void uart_send(uint16_t value) {
    /*
    Formats and sends a value on uart1. Formats "value" as a hexadecimal string.
    */
    sprintf(tx_msg, "%x\r", value);
    printf("SENDING: %s\n\r", tx_msg);
    uart_puts(&uart1, tx_msg);
}

uint32_t uart_receive() {
    /*
    Non-blockingly receieves a string on uart2. Returns -1 if no data available
    on uart2. Else returns the string received on uart2 parsed as a hexadecimal
    uint32_t.
    */
    char *ptr;
    uint32_t decoded_msg;
    uart_gets(&uart2, rx_msg, 64);
    printf("REC: %s\n\r", rx_msg);
    if (rx_msg[0] == '\0') {  // If first char is null, then no data available
        decoded_msg = -1;  // Return -1
    } else {
        decoded_msg = strtol(rx_msg, &ptr, 16);  // Else return hex parsing
    }
    return decoded_msg;
}


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

    case DEBUG_UART_STATUS:
        temp.b[0] = bitread(uart1.UxSTA, 0);  // Receive buffer data available
        temp.b[1] = bitread(uart1.UxSTA, 1);  // Read overrun error bit
        BD[EP0IN].address[0] = temp.b[0];  // URXDA
        BD[EP0IN].address[1] = temp.b[1];  // OERR
        temp.b[0] = bitread(uart1.UxSTA, 2);  // Read framing error bit
        temp.b[1] = bitread(uart1.UxSTA, 3);  // Read parity error bit
        BD[EP0IN].address[2] = temp.b[0];  // FERR
        BD[EP0IN].address[3] = temp.b[1];  // PERR
        temp.b[0] = bitread(uart1.UxSTA, 4);  // Read receiver idle bit
        temp.b[1] = bitread(uart1.UxSTA, 5);  // Read address char. detect bit
        BD[EP0IN].address[4] = temp.b[0];  // RIDLE
        BD[EP0IN].address[5] = temp.b[1];  // ADDEN

        temp.b[0] = bitread(uart2.UxSTA, 0);  // Receive buffer data available
        temp.b[1] = bitread(uart2.UxSTA, 1);  // Read overrun error bit
        BD[EP0IN].address[6] = temp.b[0];  // URXDA
        BD[EP0IN].address[7] = temp.b[1];  // OERR
        temp.b[0] = bitread(uart2.UxSTA, 2);  // Read framing error bit
        temp.b[1] = bitread(uart2.UxSTA, 3);  // Read parity error bit
        BD[EP0IN].address[8] = temp.b[0];  // FERR
        BD[EP0IN].address[9] = temp.b[1];  // PERR
        temp.b[0] = bitread(uart2.UxSTA, 4);  // Read receiver idle bit
        temp.b[1] = bitread(uart2.UxSTA, 5);  // Read address char. detect bit
        BD[EP0IN].address[10] = temp.b[0];  // RIDLE
        BD[EP0IN].address[11] = temp.b[1];  // ADDEN

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

void idle(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        trials = 0;
        pin_clear(&D[8]);
        pin_clear(&D[9]);
    }

    coin = !pin_read(&D[2]);  // digital read of the coin acceptor.

    //Note: might be a good idea to add input conditioner later on.


    // Check for state transitions
    if (coin) {
        // A coin has been detected.
        state = reset;
        // Call uart_send(). If a coin has been detected, we must inform the
        // rocket PIC.
        uart_send(121);  // Send 121 to indicate coin inserted.
    }

    if (state != last_state) {
        led_off(&led1);  // if we are leaving the state, do clean up stuff
    }
}

void reset(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
    }
    uint32_t reset_msg;


    if (timer_flag(&timer2)) {
        timer_lower(&timer2);
        // Call uart_send(). We must inform the rocket PIC how many trials remain
        // master PIC.
        uart_send((trials << 8) | 0xaa);  // Send xx10101010, where xx is no. of trials.
        // Check for state transitions
        // Call uart_receive(). We are waiting for one of the following messages:
        //    * The rocket PIC has finished resetting to game-playable state
        //    * The rocket PIC has finished resetting to game-over state.
        reset_msg = uart_receive();
        if (reset_msg == -1) {
            // No UART data available
            state = idle;
        } else if (reset_msg == 4313) {  // Reset to game start complete
            state = flying;
        } else if (reset_msg == 4315) { // Reset to game over complete
            state = idle;
        } else {
            // Some other message receieved.
            // DANGER, why are we here?
        }
    }

    if (state != last_state) {
    }
}

void flying(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        rocket_state = FLYING;
        pin_clear(&D[8]);
        pin_set(&D[9]);
    }

    // Perform state tasks
    // Tilt check
    if (pin_read(LEFT)) {
        led_on(&led3);
        // Send command to tilt rocket to left
        tilt = 1;
    }
    else if (pin_read(RIGHT)) {
        led_on(&led3);
        // Send command to tilt rocket to right
        tilt = 2;
    }
    else {
        led_off(&led3);
        tilt = 0;
    }

    // Throttle check
    if (pin_read(THROTTLE)) {
        throttle = 1;
        led_on(&led1);
    }
    else {
        throttle = 0;
        led_off(&led1);
    }
    if (timer_flag(&timer2)) {
        timer_lower(&timer2);
        // Call uart_receive(). We are waiting for one of the following messages:
        //    * The rocket has crashed
        //    * The rocket has landed
        uint32_t landing_msg;
        landing_msg = uart_receive();
        if (landing_msg == -1) {
            // No UART data available
            state = flying;
        } else if (landing_msg == 911) {  // 911 indicates a crashed rocket.
            state = lose;
        } else if (landing_msg == 10000) { // 10000 indicates a landed rocket.
            state = win;
        } else {
            // Some other message receieved.
            // DANGER, why are we here?
        }
        // Call uart_send(). We must inform the master PIC of the commanded throttle and tilt.
        uart_send((tilt << 1) + throttle);  // Concatenate throttle+tilt into value
    }

    if (state != last_state) {  // if we are leaving the state, do clean up stuff
        // turn off tilt stick led before exiting state
        led_off(&led3);
    }
}

void lose(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        pin_set(&D[8]);
        pin_clear(&D[9]);
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
        pin_set(&D[8]);
        pin_set(&D[9]);
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

void setup_uart() {
    /*
    Configures UART for communications.
    Uses uart2 to receive messages from rocket PIC on ICD3 header (RX2, TX2).
    Uses uart1 to send messages to master PIC on ICD3 header (RTS2, CTS2).
    Automatically uses uart3 for stdout, stderr to PC via audio jack.
    */
    uart_open(&uart1, &TX2, &RX2, NULL, NULL, 115200., 'N', 1,
              0, TXBUF1, 1024, RXBUF1, 1024);
    // Enable UART ERR interrupt
    IFS4bits.U1ERIF = 0;
    IEC4bits.U1ERIE = 1;

    uart_open(&uart2, &RTS2, &CTS2, NULL, NULL, 115200., 'N', 1,
              0, TXBUF2, 1024, RXBUF2, 1024);
    // Enable UART ERR interrupt
    IFS4bits.U2ERIF = 0;
    IEC4bits.U2ERIE = 1;
}

void setup() {
    timer_setPeriod(&timer1, 1);  // Timer for LED operation/status blink
    timer_setPeriod(&timer2, 0.5);  // UART Timer
    timer_setPeriod(&timer3, 0.5);
    timer_setPeriod(&timer5, 0.1);

    timer_start(&timer1);
    timer_start(&timer2);
    timer_start(&timer3);
    timer_start(&timer5);

    setup_uart();
    rocket_tilt, rocket_speed = 0;
    pin_digitalOut(&D[8]);  // State pin 1
    pin_digitalOut(&D[9]);  // State pin 2

    pin_set(&D[8]);
    pin_clear(&D[9]);
    // Declare tilt digital I/O
    LEFT = &D[12];
    RIGHT = &D[13];
    THROTTLE = &D[7];
}

int16_t main(void) {
    // printf("Starting Master Controller...\r\n");
    init_clock();
    init_ui();
    init_timer();
    init_uart();
    init_pin();

    setup();
    pin_digitalIn(&D[2]);

    InitUSB();
    U1IE = 0xFF; //setting up ISR for USB requests
    U1EIE = 0xFF;
    IFS5bits.USB1IF = 0; //flag
    IEC5bits.USB1IE = 1; //enable

    // Initialize State Machine
    state = flying;
    last_state = (STATE_HANDLER_T)NULL;

    pin_digitalOut(&D[5]);  // Heartbeat pin
    while (1) {
        if (timer_flag(&timer5)) {
            timer_lower(&timer5);
            uint8_t state_num = -1;
            // if (state == idle) {
            //     printf("State: %d\n\r");
            // } else if (state == reset) {
            //     printf("State: RESET\n\r");
            // } else if (state == flying) {
            //     printf("State: FLYING\n\r");
            // } else if (state == win) {
            //     printf("State: WIN\n\r");
            // } else if (state == lose) {
            //     printf("State: LOSE\n\r");
            // } else {
            //     printf("State: UNKNOWN STATE\n\r");
            // }
        }
        state();
        pin_toggle(&D[5]);  // Heartbeat
    }
}

