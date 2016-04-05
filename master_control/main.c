#include <p24FJ128GB206.h>
#include "config.h"
#include "common.h"
#include "pin.h"
#include "ui.h"
#include "timer.h"
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>

_PIN *LEFT, *RIGHT;

uint8_t RC_TXBUF[1024], RC_RXBUF[1024];

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
              0, NULL, 0, NULL, 0);

}

void setup() {
    timer_setPeriod(&timer1, 1);  // Timer for LED operation/status blink
    timer_setPeriod(&timer2, 0.5); 
    timer_start(&timer1);
    timer_start(&timer2);

    setup_uart();
    // declare digital I/O
    LEFT = &D[12];
    RIGHT = &D[13];
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
    while (1) {
        if (timer_flag(&timer1)) {
            // Blink green light to show normal operation.
            timer_lower(&timer1);
            led_toggle(&led3);
        }
        if (timer_flag(&timer2)) {
            // Transmit UART data
            timer_lower(&timer2);
            led_on(&led1);
            counter ++;
            // sprintf(status_msg, "This is message #%d.\r", counter);
            uint8_t addr = counter % 5;
            uint8_t cmd = 28;
            sprintf(status_msg, "%02x%02x\r", cmd, addr);
            uart_puts(&uart1, status_msg);
            led_off(&led1);
        }   

    }
}
