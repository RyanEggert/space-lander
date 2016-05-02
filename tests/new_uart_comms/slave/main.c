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
#include "stepper.h"
#include "main.h"
#include "usb.h"
#include "oc.h"
#include "dcm.h"
#include "msgs.h"

uint8_t RC_TXBUF[1024], RC_RXBUF[1024];
uint8_t RC3_TXBUF[1024], RC3_RXBUF[1024];

// Handle flying state
#define LANDED 0
#define CRASHED 1
//flying is already defined as 2
#define READY 3 //rocket has been zeroed
#define TESTING 4

// Tilt constants
#define TILT_ZERO 0
#define TILT_CCW 1
#define TILT_CW 2

#define SET_STATE    0   // Vendor request that receives 2 unsigned integer values
#define GET_VALS    1   // Vendor request that returns 2 unsigned integer values 
#define GET_ROCKET_INFO 2
#define DEBUG_UART_BUFFERS 3
#define GET_QUAD_INFO 4
#define COMMAND_DCMOTOR 5

#define DEBUG_SERVO_SET_POS 60
#define DEBUG_SERVO_SET_FREQ 61

#define DEBUG_UART_STATUS 70
#define DEBUG_OC_STATUS 72

#define GET_LIMIT_SW_INFO 75

#define X_AXIS_THRUST 0
#define Y_AXIS_THRUST 1

typedef void (*STATE_HANDLER_T)(void);

_OC *dcm_oc = &oc7;
_OC *st_oc = &oc5;




uint16_t counter;
uint8_t throttle, tilt; //commands
uint8_t rocketstuff[64], rec_msg[64];
uint8_t cmd, value;
uint16_t val1, val2;
char rx_msg[64], tx_msg[8];

void VendorRequests(void) {
    disable_interrupts();
    WORD temp;
    WORD temp2;
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

        BD[EP0IN].bytecount = 6;    // set EP0 IN byte count to 4
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        break;

    case DEBUG_OC_STATUS:
        temp.b[0] = bitread(dcm_oc->OCxCON1, 0);  // Receive buffer data available
        temp.b[1] = bitread(dcm_oc->OCxCON1, 1);  // Read overrun error bit
        BD[EP0IN].address[0] = temp.b[0];  // OCM0
        BD[EP0IN].address[1] = temp.b[1];  // OCM1
        temp.b[0] = bitread(dcm_oc->OCxCON1, 2);  // Read framing error bit
        temp.b[1] = bitread(dcm_oc->OCxCON1, 3);  // Read parity error bit
        BD[EP0IN].address[2] = temp.b[0];  // OCM2
        BD[EP0IN].address[3] = temp.b[1];  // OCTSEL
        temp.b[0] = bitread(dcm_oc->OCxCON1, 4);  // Read receiver idle bit
        BD[EP0IN].address[4] = temp.b[0];  // OCTFLT
        temp.b[0] = bitread(st_oc->OCxCON1, 0);  // Receive buffer data available
        temp.b[1] = bitread(st_oc->OCxCON1, 1);  // Read overrun error bit
        BD[EP0IN].address[5] = temp.b[0];  // OCM0
        BD[EP0IN].address[6] = temp.b[1];  // OCM1
        temp.b[0] = bitread(st_oc->OCxCON1, 2);  // Receive buffer data available
        temp.b[1] = bitread(st_oc->OCxCON1, 3);  // Read overrun error bit
        BD[EP0IN].address[7] = temp.b[0];  // OCM2
        BD[EP0IN].address[8] = temp.b[1];  // OCTFLT
        temp.b[0] = bitread(st_oc->OCxCON1, 4);  // Read receiver idle bit
        BD[EP0IN].address[9] = temp.b[0];  // OCTFLT

        BD[EP0IN].bytecount = 10;    // set EP0 IN byte count to 9
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit

    // case GET_ROCKET_INFO:
    //     temp.w = rocket_tilt;
    //     BD[EP0IN].address[0] = temp.b[0];
    //     BD[EP0IN].address[1] = temp.b[1];
    //     temp.w = rocket_speed;
    //     BD[EP0IN].address[2] = temp.b[0];
    //     BD[EP0IN].address[3] = temp.b[1];
    //     temp.w = throttle;
    //     BD[EP0IN].address[4] = temp.b[0];
    //     BD[EP0IN].address[5] = temp.b[1];
    //     temp.w = motor_speed;
    //     BD[EP0IN].address[6] = temp.b[0];
    //     BD[EP0IN].address[7] = temp.b[1];
    //     temp.w = motor_thrust;
    //     BD[EP0IN].address[8] = temp.b[0];
    //     BD[EP0IN].address[9] = temp.b[1];
    //     temp.w = tilt_ang;
    //     BD[EP0IN].address[10] = temp.b[0];
    //     BD[EP0IN].address[11] = temp.b[1];
    //     temp.w = tilt_dir;
    //     BD[EP0IN].address[12] = temp.b[0];
    //     BD[EP0IN].address[13] = temp.b[1];
    //     temp.w = (int16_t)(stepper_speed);
    //     BD[EP0IN].address[14] = temp.b[0];
    //     BD[EP0IN].address[15] = temp.b[1];
    //     temp.w = rocket_state;
    //     BD[EP0IN].address[16] = temp.b[0];
    //     BD[EP0IN].address[17] = temp.b[1];
    //     BD[EP0IN].bytecount = 18;    // set EP0 IN byte count to 14
    //     BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
    //     break;

    // case GET_QUAD_INFO:
    //     temp32.ul = quad1.counter;
    //     BD[EP0IN].address[0] = temp32.b[0];
    //     BD[EP0IN].address[1] = temp32.b[1];
    //     BD[EP0IN].address[2] = temp32.b[2];
    //     BD[EP0IN].address[3] = temp32.b[3];
    //     temp.w = quad1.overflow;
    //     BD[EP0IN].address[4] = temp.b[0];
    //     BD[EP0IN].address[5] = temp.b[1];
    //     BD[EP0IN].bytecount = 6;    // set EP0 IN byte count to 4
    //     BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
    //     break;

    case GET_LIMIT_SW_INFO:  // Python unimplemented
        temp.b[0] = es_y_bot.hit;
        temp.b[1] = es_y_top.hit;
        BD[EP0IN].address[0] = temp.b[0];
        BD[EP0IN].address[1] = temp.b[1];
        temp.b[0] = es_x_l.hit;
        temp.b[1] = es_x_r.hit;
        BD[EP0IN].address[2] = temp.b[0];
        BD[EP0IN].address[3] = temp.b[1];
        temp.b[0] = es_landing.hit;
        BD[EP0IN].address[4] = temp.b[0];
        BD[EP0IN].bytecount = 5;    // set EP0 IN byte count to 4
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        break;

    case COMMAND_DCMOTOR:
        dcm_velocity(&dcm1, USB_setup.wValue.w, USB_setup.wIndex.w);
        BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        break;

    // case DEBUG_SERVO_SET_POS:
    //     temp.w = USB_setup.wValue.w;   // Commanded position
    //     temp2.w = USB_setup.wIndex.w;  // Servo driver index
    //     servo_usb_set(&sd1, temp2.b[0], temp.w);
    //     BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
    //     BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
    //     break;

    // case DEBUG_SERVO_SET_FREQ:
    //     temp.w = USB_setup.wValue.w;
    //     servo_driver_configure(&sd1, ((float)(temp.w)) / 10);
    //     BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
    //     BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
    //     break;

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

// void UARTrequests() {
//     // received uart message length = 8
//     uart_gets(&uart1, rec_msg, 8);
//     uint32_t decoded_msg = (uint32_t)strtol(rec_msg, NULL, 16);
//     cmd = decoded_msg & 0x0f;
//     value = (decoded_msg & 0xf0) >> 4;
//     switch (cmd) {
//     case GET_ROCKET_VALS:
//         //speed, orientation
//         sprintf(rocketstuff, "%02x%02x%02x\r", rocket_speed, rocket_tilt, rocket_state);
//         uart_puts(&uart1, rocketstuff);
//         break;
//     case SET_ROCKET_STATE:
//         rocket_state = value;
//         break;
//     case SEND_ROCKET_COMMANDS:
//         throttle = value & 0b01;
//         tilt = (value & 0b110) >> 1;
//         break;
//     }
// }

void setup_uart() {
    /*
    Configures UART for communications.
    Uses uart1 for inter-PIC communications. Rx on D[0], Tx on D[1].
    Automatically uses uart3 for stdout, stderr to PC via audio jack.
    */
    uart_open(&uart1, &TX2, &RX2, NULL, NULL, 115200., 'N', 1,
              0, RC_TXBUF, 1024, RC_RXBUF, 1024);
    // Enable UART ERR interrupt
    IFS4bits.U1ERIF = 0;
    IEC4bits.U1ERIE = 1;

    uart_open(&uart2, &RTS2, &CTS2, NULL, NULL, 115200., 'N', 1,
              0, RC3_TXBUF, 1024, RC3_RXBUF, 1024);
    // Enable UART ERR interrupt
    IFS4bits.U2ERIF = 0;
    IEC4bits.U2ERIE = 1;
}

void read_limitsw(_TIMER *timer) { //debounce the things
    // Gantry X-Axis (stepper) endstops
    // stop_read(&es_x_l);
    // stop_read(&es_x_r);
    st_check_stops(&st_d);

    // Gantry Y-Axis (dc motor) endstops
    // stop_read(&es_y_top);
    // stop_read(&es_y_bot);
    dcm_check_stops(&dcm1);

    // Landing pad endstop
    stop_read(&es_landing);
}

void UART_send(uint8_t value) {
    sprintf(tx_msg, "%05x\r", value);
    printf("SENDING: %s\n\r", tx_msg);
    uart_puts(&uart2, tx_msg);
}

uint32_t UART_receive() {
    char *ptr;
    uint32_t decoded_msg;
    uart_gets(&uart1, rx_msg, 64);
    printf("REC: %s\n\r", rx_msg);
    if (rx_msg[0] == '\0') {
        led_on(&led2);
        decoded_msg = -1;
    } else {
        // printf("FIRST %s\n\r", rx_msg[0..10]);
        led_off(&led2);
        decoded_msg = strtol(rx_msg, &ptr, 16);
    }
    return decoded_msg;
}

void setup() {
    timer_setPeriod(&timer1, 1);  // Timer for LED operation/status blink
    timer_setPeriod(&timer2, 0.1);  // Timer for UART servicing
    timer_setPeriod(&timer3, 0.5);
    // timer_setPeriod(&timer4, 0.001);
    // timer_setPeriod(&timer5, 0.01);  // Timer for clocking stepper motor
    timer_start(&timer1);
    timer_start(&timer2);
    timer_start(&timer3);
    // timer_start(&timer4);
    // timer_start(&timer5);

    // DC MOTOR + QUAD ENCODER
    dcm_init(&dcm1, &D[10], &D[11], 1e3, 0, dcm_oc, &es_y_bot, &es_y_top);
    // quad_init(&quad1, &D[8], &D[9]); // quad1 uses pins D8 & D9
    // quad_every(&quad1, &timer5, 0.0000875); // quad1 will use timer5 interrupts

    // STEPPER
    // st_init(&st_d, &D[0], &D[1], &D[2], &D[3], st_oc, 0x7FFF, &es_x_l, &es_x_r);

    timer_every(&timer4, .001, read_limitsw);  // Start timed endstop reading
    // General use debugging output pin
    // pin_digitalOut(&D[2]);

    setup_uart();
    throttle, tilt = 0;
    val1, val2 = 8;

    // rocket_tilt = 500;
}

int16_t main(void) {
    // printf("Starting Rocket Controller...\r\n");
    init_clock();
    init_ui();
    init_oc();
    init_pin();
    init_timer();
    init_stops();
    init_st();      // if this is first, then D[1] - D[3] don't work as outputs
    init_quad();
    init_dcm();
    init_uart();    // if this is first, then D[0] doesn't output OC wfm
    setup();
    uint16_t counter = 0;
    uint64_t msg;
    char is_recip = 0;

    InitUSB();
    U1IE = 0xFF; //setting up ISR for USB requests
    U1EIE = 0xFF;
    IFS5bits.USB1IF = 0; //flag
    IEC5bits.USB1IE = 1; //enable

    pin_digitalOut(&D[5]);
    pin_digitalOut(&D[0]);
    pin_digitalOut(&D[1]);
    uint8_t val = 0;

    st_state(&st_d, 1);
    while (1) {
        if (timer_flag(&timer1)){
            timer_lower(&timer1);
            led_toggle(&led1);
            // pin_toggle(&D[0]);
            // pin_toggle(&D[1]);
            printf("BLINK\n\r");
        }

        if (timer_flag(&timer2)){  // UART Rx timer
            timer_lower(&timer2);
            uint32_t recd;
            recd = UART_receive();
            pin_toggle(&D[0]);
            printf("RECD %lu\n\r", recd);
            if (recd == 10) {
                led_on(&led3);
            } else {
                led_off(&led3);
            }
        }

        if (timer_flag(&timer3)){  // UART Tx timer
            timer_lower(&timer3);
            val = (val + 1) % 11;
            UART_send(val);
            pin_toggle(&D[1]);
        }
        pin_toggle(&D[5]);
    }
}

