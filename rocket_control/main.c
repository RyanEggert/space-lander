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
#include "main.h"


uint8_t MC_TXBUF[1024], MC_RXBUF[1024];

#define GET_ROCKET_VALS 0
#define SET_ROCKET_STATE 1
#define SEND_ROCKET_COMMANDS 2

#define IDLE 0
#define RESET 1
#define FLYING 2

#define SET_STATE    0   // Vendor request that receives 2 unsigned integer values
#define GET_VALS    1   // Vendor request that returns 2 unsigned integer values 
#define GET_ROCKET_INFO 2

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
        case GET_VALS:
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


void UARTrequests(){
    uart_gets(&uart1, rec_msg, 64);
    uint32_t decoded_msg = (uint32_t)strtol(rec_msg, NULL, 16);
    cmd = decoded_msg & 0x0f;
    value = (decoded_msg & 0xf0) >> 4;
    switch(cmd){
        case GET_ROCKET_VALS:
        //speed, orientation
            led_toggle(&led1);
            // rocket_speed = 0xFFFF;
            // rocket_tilt = 0xFFFF;
            // rocket_state = 0xFFFF;
            sprintf(rocketstuff, "%04x%04x%04x\r", rocket_speed, rocket_tilt, rocket_state);
            uart_puts(&uart1, rocketstuff);
            led_toggle(&led2);
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

//
void print_buffer(uint8_t *buffer, uint16_t size) {
    int i;
    uint8_t* buf_str = (uint8_t*) malloc (2*size + 1);
    uint8_t* buf_ptr = buf_str;
    for (i = 0; i < size; i++)
    {
        buf_ptr += sprintf(buf_ptr, "%02X", buffer[i]);
    }
    sprintf(buf_ptr,"\n");
    *(buf_ptr + 1) = '\0';
    printf("%s\n", buf_str);
}

char check_for_string(uint8_t *str) {
    /*
    If the input string buffer is "test", turn on led1 and return 1. Else 
    turn off led1 and return 0.
    */
    int rc = strcmp(str, "test");
    char out;
    if (rc == 0) {
        led_on(&led1);
        out = 1;
    } else {
        led_off(&led1);
        out = 0;
    }
    return out;
}

uint64_t convert_msg(uint8_t *str) {
    /* 
    Interprets string buffer which represents a hexadecimal number and stores 
    the encoded value as a uint64_t
    */

    // sprintf(a, "%08x%08x\r\n", 0xffff, 40);
    // printf("%d\r\n", a[0]);
    // printf("%s", a);
    uint64_t decoded_msg = (uint64_t)strtoll(str, NULL, 16);
    return decoded_msg;
}

char parse_addr(uint64_t *msg) {
    /*
    Parses the received message and returns a char representing whether this 
    information is addressed to this device.
    */
    // uint8_t msg_addr = (uint8_t)(*msg & ((uint64_t)(0xFF) << 56));
    uint8_t msg_addr = (uint8_t)(*msg & ((uint64_t)(0xFF)));
    char is_recipient = 0;
    if (msg_addr == LOCAL_ADDR) {
        is_recipient = 1;
    }
    return is_recipient;
}

void setup_uart() {
    // uart_open(&uart3, &AJTX, &AJRX, NULL, NULL, 115200., 'N', 1, 
    //           // 0, TXBUF, 1024, RXBUF, 1024);

    // uart_open(_UART *self, _PIN *TX, _PIN *RX, _PIN *RTS, _PIN *CTS, 
    //            float baudrate, int8_t parity, int16_t stopbits, 
    //            uint16_t TXthreshold, uint8_t *TXbuffer, uint16_t TXbufferlen, 
    //            uint8_t *RXbuffer, uint16_t RXbufferlen)

    // uart_open(&uart2, &D[0], &D[1], NULL, NULL, 115200., 'N', 1, 
    //           0, MC_TXBUF, 1024, MC_RXBUF, 1024);
    // uart_open(&uart3, &AJTX, &AJRX, NULL, NULL, 115200., 'N', 1, 
    //           0, TXBUF, 1024, RXBUF, 1024);

    // pin_init(&AJRX, (uint16_t *)&PORTG, (uint16_t *)&TRISG, 
    //          (uint16_t *)NULL, 6, -1, 8, 21, (uint16_t *)&RPOR10);
    // pin_init(&AJTX, (uint16_t *)&PORTG, (uint16_t *)&TRISG, 
    //          (uint16_t *)NULL, 7, -1, 0, 26, (uint16_t *)&RPOR13);

    uart_open(&uart1, &D[0], &D[1], NULL, NULL, 19200., 'N', 1, 
              0, MC_TXBUF, 1024, MC_RXBUF, 1024);
}

void setup() {
    timer_setPeriod(&timer1, 1);  // Timer for LED operation/status blink
    timer_setPeriod(&timer2, 0.5); 
    timer_start(&timer1);
    timer_start(&timer2);

    setup_uart();
    throttle, tilt = 0;
    val1,val2 = 8;
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
    led_off(&led2);
    led_off(&led1);

    InitUSB();
    U1IE = 0xFF; //setting up ISR for USB requests
    U1EIE = 0xFF;
    IFS5bits.USB1IF = 0; //flag
    IEC5bits.USB1IE = 1; //enable

    while (1) {
        // if (timer_flag(&timer1)) {
        //     // Blink green light to show normal operation.
        //     timer_lower(&timer1);
        //     led_toggle(&led2);
        // }

        if (timer_flag(&timer2)) {
            timer_lower(&timer2);
            // uart_gets(&uart1, rec_msg, 64);
            // led_toggle(&led2);
            UARTrequests();


            // if (throttle == 1){
            //     led_on(&led1);
            // }

            // else{
            //     led_off(&led1);
            // }

        // if(rocket_state == FLYING){
        //     led_on(&led1);
        // }

        // else{
        //     led_off(&led1);
        // }

        val1 = uart1.TXbuffer.head;
        val2 = uart1.TXbuffer.tail;

            
        }   
    }
}
