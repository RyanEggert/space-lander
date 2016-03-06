#include <p24FJ128GB206.h>
#include "config.h"
#include "common.h"
#include "pin.h"
#include "ui.h"
#include "timer.h"
#include "uart.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>

uint8_t MC_TXBUF[1024], MC_RXBUF[1024];

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
    pin_init(&AJRX, (uint16_t *)&PORTG, (uint16_t *)&TRISG, 
             (uint16_t *)NULL, 6, -1, 8, 21, (uint16_t *)&RPOR10);
    pin_init(&AJTX, (uint16_t *)&PORTG, (uint16_t *)&TRISG, 
             (uint16_t *)NULL, 7, -1, 0, 26, (uint16_t *)&RPOR13);
    uart_open(&uart1, &AJTX, &AJRX, NULL, NULL, 19200., 'N', 1, 
              0, MC_TXBUF, 1024, MC_RXBUF, 1024);
}

void setup() {
    timer_setPeriod(&timer1, 1);  // Timer for LED operation/status blink
    timer_setPeriod(&timer2, 0.5); 
    timer_start(&timer1);
    timer_start(&timer2);

    setup_uart();

}

int16_t main(void) {
    // printf("Starting Rocket Controller...\r\n");
    init_clock();
    init_ui();
    init_timer();
    init_uart();
    setup();
    uint16_t counter = 0;
    uint8_t rec_msg [64];
    uint64_t msg;
    char is_recip = 0;
    while (1) {
        if (timer_flag(&timer1)) {
            // Blink green light to show normal operation.
            timer_lower(&timer1);
            led_toggle(&led2);
        }
        if (timer_flag(&timer2)) {
            timer_lower(&timer2);
            uart_gets(&uart1, rec_msg, 64);
            msg = convert_msg(rec_msg);
            is_recip = parse_addr(&msg);
            if (is_recip == 1) {
                led_on(&led3);
            } else {
                led_off(&led3);
            }
        }   
    }
}
