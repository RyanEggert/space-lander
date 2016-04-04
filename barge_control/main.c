#include <p24FJ128GB206.h>
#include "config.h"
#include "common.h"
#include "pin.h"
#include "uart.h"
#include <stdio.h>

int16_t main(void) {
    init_clock();
    init_uart();
    init_pin();
    pin_digitalOut(&D[0]);

   while (1) {
        pin_toggle(&D[0]);
    }
}

