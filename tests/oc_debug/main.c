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
// #include "usb.h"
#include "oc.h"
#include "dcm.h"
#include "msgs.h"

// _OC dcm_oc = oc7;
// _OC st_oc = oc5;

// stepper vars
uint16_t stepper_count = 0;
uint16_t stepper_dir_track = 0;
uint8_t stepper_state = 0;  // 0 = drive to left x endstop, 1 = drive to middle, 2 = stop *** might not need this?
float stepper_speed = 0;
float stepper_speed_limit = 1250;
uint16_t stepper_reset_lim = 1000;  // # of steps to move stepper during reset state
float stepper_deadband = 750;
float stepper_thrust;
float stepper_resist = 0.01;

// dc motor vars
uint16_t motor_state;
uint16_t motor_dir_track = 0;
uint16_t motor_speed = 0;
uint16_t motor_speed_limit = 0x7FFF;
uint16_t motor_deadband = 7000;  // will find once gantry is built
uint16_t motor_thrust;


// void VendorRequests(void) {
//     disable_interrupts();
//     WORD temp;
//     WORD temp2;
//     WORD32 temp32;
//     switch (USB_setup.bRequest) {
//     case SET_STATE:
//         // state = USB_setup.wValue.w;
//         BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
//         BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
//         break;

//     case DEBUG_UART_BUFFERS:
//         temp.w = uart1.TXbuffer.head;
//         BD[EP0IN].address[0] = temp.b[0];
//         BD[EP0IN].address[1] = temp.b[1];
//         temp.w = uart1.TXbuffer.tail;
//         BD[EP0IN].address[2] = temp.b[0];
//         BD[EP0IN].address[3] = temp.b[1];
//         temp.w = uart1.TXbuffer.count;
//         BD[EP0IN].address[4] = temp.b[0];
//         BD[EP0IN].address[5] = temp.b[1];

//         temp.w = uart1.RXbuffer.head;
//         BD[EP0IN].address[6] = temp.b[0];
//         BD[EP0IN].address[7] = temp.b[1];
//         temp.w = uart1.RXbuffer.tail;
//         BD[EP0IN].address[8] = temp.b[0];
//         BD[EP0IN].address[9] = temp.b[1];
//         temp.w = uart1.RXbuffer.count;
//         BD[EP0IN].address[10] = temp.b[0];
//         BD[EP0IN].address[11] = temp.b[1];
//         BD[EP0IN].bytecount = 12;    // set EP0 IN byte count to 4
//         BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
//         break;

//     case DEBUG_UART_STATUS:
//         temp.b[0] = bitread(uart1.UxSTA, 0);  // Receive buffer data available
//         temp.b[1] = bitread(uart1.UxSTA, 1);  // Read overrun error bit
//         BD[EP0IN].address[0] = temp.b[0];  // URXDA
//         BD[EP0IN].address[1] = temp.b[1];  // OERR
//         temp.b[0] = bitread(uart1.UxSTA, 2);  // Read framing error bit
//         temp.b[1] = bitread(uart1.UxSTA, 3);  // Read parity error bit
//         BD[EP0IN].address[2] = temp.b[0];  // FERR
//         BD[EP0IN].address[3] = temp.b[1];  // PERR
//         temp.b[0] = bitread(uart1.UxSTA, 4);  // Read receiver idle bit
//         temp.b[1] = bitread(uart1.UxSTA, 5);  // Read address char. detect bit
//         BD[EP0IN].address[4] = temp.b[0];  // RIDLE
//         BD[EP0IN].address[5] = temp.b[1];  // ADDEN

//         BD[EP0IN].bytecount = 6;    // set EP0 IN byte count to 4
//         BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
//         break;

//     case DEBUG_OC_STATUS:
//         temp.b[0] = bitread(dcm_oc.OCxCON, 0);  // Receive buffer data available
//         temp.b[1] = bitread(dcm_oc.OCxCON, 1);  // Read overrun error bit
//         BD[EP0IN].address[0] = temp.b[0];  // OCM0
//         BD[EP0IN].address[1] = temp.b[1];  // OCM1
//         temp.b[0] = bitread(dcm_oc.OCxCON, 2);  // Read framing error bit
//         temp.b[1] = bitread(dcm_oc.OCxCON, 3);  // Read parity error bit
//         BD[EP0IN].address[2] = temp.b[0];  // OCM2
//         BD[EP0IN].address[3] = temp.b[1];  // OCTSEL
//         temp.b[0] = bitread(dcm_oc.OCxCON, 4);  // Read receiver idle bit
//         BD[EP0IN].address[4] = temp.b[0];  // OCTFLT
//         temp.b[0] = bitread(st_oc.OCxCON, 0);  // Receive buffer data available
//         temp.b[1] = bitread(st_oc.OCxCON, 1);  // Read overrun error bit
//         BD[EP0IN].address[0] = temp.b[0];  // OCM0
//         BD[EP0IN].address[1] = temp.b[1];  // OCM1
//         temp.b[0] = bitread(st_oc.OCxCON, 2);  // Receive buffer data available
//         temp.b[1] = bitread(st_oc.OCxCON, 3);  // Read overrun error bit
//         BD[EP0IN].address[0] = temp.b[0];  // OCM2
//         BD[EP0IN].address[1] = temp.b[1];  // OCTFLT

//         BD[EP0IN].bytecount = 10;    // set EP0 IN byte count to 4
//         BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit

//     case GET_ROCKET_INFO:
//         temp.w = rocket_tilt;
//         BD[EP0IN].address[0] = temp.b[0];
//         BD[EP0IN].address[1] = temp.b[1];
//         temp.w = rocket_speed;
//         BD[EP0IN].address[2] = temp.b[0];
//         BD[EP0IN].address[3] = temp.b[1];
//         temp.w = throttle;
//         BD[EP0IN].address[4] = temp.b[0];
//         BD[EP0IN].address[5] = temp.b[1];
//         temp.w = motor_speed;
//         BD[EP0IN].address[6] = temp.b[0];
//         BD[EP0IN].address[7] = temp.b[1];
//         temp.w = motor_thrust;
//         BD[EP0IN].address[8] = temp.b[0];
//         BD[EP0IN].address[9] = temp.b[1];
//         temp.w = tilt_ang;
//         BD[EP0IN].address[10] = temp.b[0];
//         BD[EP0IN].address[11] = temp.b[1];
//         temp.w = tilt_dir;
//         BD[EP0IN].address[12] = temp.b[0];
//         BD[EP0IN].address[13] = temp.b[1];
//         temp.w = (int16_t)(stepper_speed);
//         BD[EP0IN].address[14] = temp.b[0];
//         BD[EP0IN].address[15] = temp.b[1];
//         temp.w = rocket_state;
//         BD[EP0IN].address[16] = temp.b[0];
//         BD[EP0IN].address[17] = temp.b[1];
//         BD[EP0IN].bytecount = 18;    // set EP0 IN byte count to 14
//         BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
//         break;

//     case GET_QUAD_INFO:
//         temp32.ul = quad1.counter;
//         BD[EP0IN].address[0] = temp32.b[0];
//         BD[EP0IN].address[1] = temp32.b[1];
//         BD[EP0IN].address[2] = temp32.b[2];
//         BD[EP0IN].address[3] = temp32.b[3];
//         temp.w = quad1.overflow;
//         BD[EP0IN].address[4] = temp.b[0];
//         BD[EP0IN].address[5] = temp.b[1];
//         BD[EP0IN].bytecount = 6;    // set EP0 IN byte count to 4
//         BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
//         break;

//     case GET_LIMIT_SW_INFO:  // Python unimplemented
//         temp.b[0] = es_y_bot.hit;
//         temp.b[1] = es_y_top.hit;
//         BD[EP0IN].address[0] = temp.b[0];
//         BD[EP0IN].address[1] = temp.b[1];
//         temp.b[0] = es_x_l.hit;
//         temp.b[1] = es_x_r.hit;
//         BD[EP0IN].address[2] = temp.b[0];
//         BD[EP0IN].address[3] = temp.b[1];
//         temp.b[0] = es_landing.hit;
//         BD[EP0IN].address[4] = temp.b[0];
//         BD[EP0IN].bytecount = 5;    // set EP0 IN byte count to 4
//         BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
//         break;

//     case COMMAND_DCMOTOR:
//         dcm_velocity(&dcm1, USB_setup.wValue.w, USB_setup.wIndex.w);
//         BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
//         BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
//         break;

//     case DEBUG_SERVO_SET_POS:
//         temp.w = USB_setup.wValue.w;   // Commanded position
//         temp2.w = USB_setup.wIndex.w;  // Servo driver index
//         servo_usb_set(&sd1, temp2.b[0], temp.w);
//         BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
//         BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
//         break;

//     case DEBUG_SERVO_SET_FREQ:
//         temp.w = USB_setup.wValue.w;
//         servo_driver_configure(&sd1, ((float)(temp.w)) / 10);
//         BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
//         BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
//         break;

//     default:
//         USB_error_flags |= 0x01;    // set Request Error Flag
//     }
//     enable_interrupts();
// }

// void VendorRequestsIn(void) {
//     switch (USB_request.setup.bRequest) {
//     default:
//         USB_error_flags |= 0x01;                    // set Request Error Flag
//     }
// }

// void VendorRequestsOut(void) {
//     switch (USB_request.setup.bRequest) {
//     default:
//         USB_error_flags |= 0x01;                    // set Request Error Flag
//     }
// }

void read_limitsw(_TIMER *timer) { //debounce the things
    // Gantry X-Axis (stepper) endstops
    // stop_read(&es_x_l);
    // stop_read(&es_x_r);
    st_check_stops(&st_d);
    st_d.stop_min->hit = 0;
    st_d.stop_max->hit = 0;

    // Gantry Y-Axis (dc motor) endstops
    // stop_read(&es_y_top);
    // stop_read(&es_y_bot);
    dcm_check_stops(&dcm1);
    dcm1.stop_min->hit = 0;
    dcm1.stop_max->hit = 0;


    // Landing pad endstop
    // stop_read(&es_landing);

}

void setup() {
    timer_setPeriod(&timer1, 0.001);  // Timer for LED operation/status blink
    timer_setPeriod(&timer2, 0.01);  // Timer for UART servicing
    timer_setPeriod(&timer3, 0.01);
    timer_setPeriod(&timer4, 0.001);
    // timer_setPeriod(&timer5, 0.01);  // Timer for clocking stepper motor
    timer_start(&timer1);
    timer_start(&timer2);
    timer_start(&timer3);
    timer_start(&timer4);
    // timer_start(&timer5);

    // // DC MOTOR + QUAD ENCODER
    // dcm_init(&dcm1, &D[10], &D[11], 1e3, 0, &oc7, &es_y_bot, &es_y_top);
    // // quad_init(&quad1, &D[8], &D[9]); // quad1 uses pins D8 & D9
    // // quad_every(&quad1, &timer5, 0.0000875); // quad1 will use timer5 interrupts

    // // STEPPER
    // st_init(&st_d, &D[0], &D[1], &D[2], &D[3], &oc5, 0x7FFF, &timer5, &es_x_l, &es_x_r);

    // timer_every(&timer4, .001, read_limitsw);  // Start timed endstop reading
    // General use debugging output pin
    // pin_digitalOut(&D[2]);

    // setup_uart();
    // rocket_tilt = 500;

}

uint16_t pwm_duty_pct_to_int(float percent) {
    return (uint16_t)(percent * 65535);
}

int16_t main(void) {
    // printf("Starting Rocket Controller...\r\n");
    init_clock();
    init_ui();
    init_oc();
    init_pin();
    init_timer();
    // init_stops();
    // init_st();      // if this is first, then D[1] - D[3] don't work as outputs
    // init_dcm();
    setup();

    // InitUSB();
    // U1IE = 0xFF; //setting up ISR for USB requests
    // U1EIE = 0xFF;
    // IFS5bits.USB1IF = 0; //flag
    // IEC5bits.USB1IE = 1; //enable

    // dcm_velocity(&dcm1, 64000, 1);
    pin_digitalOut(&D[5]);

    // st_state(&st_d, 1);
    // servo_set(&servo0, 150, 0);
    float stsp = .10;
    // dcm_speed(&dcm1, 32000);
    // void oc_pwm(_OC *self, _PIN *pin, _TIMER *timer, float freq, uint16_t duty) {

    oc_pwm(&oc5, &D[10], NULL, 10000, pwm_duty_pct_to_int(.50));
    oc_pwm(&oc6, &D[11], NULL, 15432, pwm_duty_pct_to_int(.50));

    while (1) {
        // ServiceUSB();
        pin_toggle(&D[5]);
        if (timer_flag(&timer1)) {
            timer_lower(&timer1);
            led_toggle(&led2);
            if (stsp == .10) {
                stsp = .20;
            } else {
                stsp = .10;
            }
            oc_free(&oc6);
            oc_pwm(&oc6, &D[11], NULL, 15432, pwm_duty_pct_to_int(stsp));
            // st_speed(&st_d, stsp);
        }
    }
}
