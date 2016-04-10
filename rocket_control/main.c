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

// Handle flying state
#define LANDED 0
#define CRASHED 1
//flying is already defined as 2
#define READY 3 //rocket has been zeroed

#define SET_STATE    0   // Vendor request that receives 2 unsigned integer values
#define GET_VALS    1   // Vendor request that returns 2 unsigned integer values 
#define GET_ROCKET_INFO 2
#define DEBUG_UART_BUFFERS 3
#define GET_QUAD_INFO 4
//
// endstop pins
_PIN *Y_END_TOP, *Y_END_BOT, *X_END_L, *X_END_R;

typedef void (*STATE_HANDLER_T)(void);

void idle(void);
void reset(void);
void flying(void);
void win(void);
void lose(void);

STATE_HANDLER_T state, last_state;

uint16_t rocket_state = FLYING;
uint16_t rocket_speed, rocket_tilt;
uint16_t counter;
uint8_t throttle, tilt; //commands
uint8_t rocketstuff[64], rec_msg[64];
uint8_t cmd, value;
uint16_t val1, val2;


volatile uint16_t sw_state = 0;
volatile uint16_t sw_state2 = 0;
volatile uint16_t sw_state3 = 0;
volatile uint16_t sw_state4 = 0;

volatile bool TOP_DSTOP, BOT_DSTOP, RT_DSTOP, LT_DSTOP;

// kinematic model vals
uint16_t thrust_val = 0x0010;
uint16_t grav_val = 0x0010;

// stepper vars
uint16_t stepper_count = 0;
uint16_t stepper_lim = 1000;
uint8_t stepper_state = 0;  // 0 = drive to X_END_L, 1 = drive to middle, 2 = stop
uint16_t stepper_speed = 0;

// dc motor vars
uint16_t motor_state;
uint16_t motor_dir_track = 0;
uint16_t motor_speed = 0;
uint16_t motor_speed_limit = 0x7FFF;
uint16_t motor_deadband = 0;  // will find once gantry is built

// tilt vals (maxes out at 4096; may use smaller range)
uint16_t tilt_max = 4095;
uint16_t tilt_min = 0;

void rocket_model() {
    // determines speed setpoint of rocket in x + y axes
    if (throttle) { // Thrust on
        // set y thrust val
        if (motor_dir_track == 0) {  // rocket falling
            if (motor_speed - motor_deadband > thrust_val) {  // nonzero velocity
                motor_speed = motor_speed - thrust_val;
            }
            else {  // zero velocity
                motor_dir_track = 1;
                motor_speed = motor_deadband;
            }
        }
        else { // rocket rising
            if (motor_speed < motor_speed_limit) {
                motor_speed = motor_speed + thrust_val;
            }
        }
        // led_on(&led2);
    }
    else { // no thrust
        // set y thrust val
        if (motor_dir_track == 0) { // rocket falling
            if (motor_speed < motor_speed_limit) {
                motor_speed = motor_speed + grav_val;
            }
        }
        else {  // rocket rising
            if (motor_speed - motor_deadband > thrust_val) {
                motor_speed = motor_speed - grav_val;
            }
            else {
                motor_dir_track = motor_deadband;
                motor_speed = grav_val;
            }
        }
        // led_off(&led2);
    }
    if (motor_dir_track) {
        // pin_set(MOTOR_DIR);
        dcm_velocity(&dcm1, motor_speed, 1);
    }
    else {
        dcm_velocity(&dcm1, motor_speed, 0);
    }
    // handle stepper motor

    rocket_speed = motor_speed + stepper_speed;
}


void VendorRequests(void) {
    disable_interrupts();
    WORD temp;
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
        temp.w = motor_speed;
        BD[EP0IN].address[6] = temp.b[0];
        BD[EP0IN].address[7] = temp.b[1];
        temp.w = stepper_speed;
        BD[EP0IN].address[8] = temp.b[0];
        BD[EP0IN].address[9] = temp.b[1];
        BD[EP0IN].bytecount = 6;    // set EP0 IN byte count to 4
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        break;
    case GET_QUAD_INFO:
        temp32.ul = quad1.counter;
        BD[EP0IN].address[0] = temp32.b[0];
        BD[EP0IN].address[1] = temp32.b[1];
        BD[EP0IN].address[2] = temp32.b[2];
        BD[EP0IN].address[3] = temp32.b[3];
        temp.w = quad1.overflow;
        BD[EP0IN].address[4] = temp.b[0];
        BD[EP0IN].address[5] = temp.b[1];
        BD[EP0IN].bytecount = 6;    // set EP0 IN byte count to 4
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        break;
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
            tilt = (value & 0b110) >> 1;
            break;
    }
}

void setup_uart() {
    /*
    Configures UART for communications.
    Uses uart1 for inter-PIC communications. Rx on D[1], Tx on D[0].
    Automatically uses uart2 for stdout, stderr to PC via audio jack.
    */
    uart_open(&uart1, &TX2, &RX2, NULL, NULL, 115200., 'N', 1,
              0, RC_TXBUF, 1024, RC_RXBUF, 1024);
}

void idle(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        // trials = 0; // let master handle # trials
        led_on(&led1);
    }

    if (state != last_state) {
        led_off(&led1);  // if we are leaving the state, do clean up stuff
    }
}

void reset(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        stepper_count = 0;
        stepper_state = 0;  // drive to X_END_L
        // led_on(&led2);
    }

    // Perform state tasks

    // Drive stepper left until it hits deadstop
    // *** Maybe switch this to STATE_HANDLER instead of switch-case?
    if (timer_flag(&timer3)) {
        timer_lower(&timer3);
        switch (stepper_state) {
            case 0:
                if (!pin_read(X_END_L)) {
                    st_direction(&st_d, 1);
                    st_speed(&st_d, 1000);
                }
                else {
                    stepper_state = 1;
                }
                break;
            case 1:
                if (stepper_count < stepper_lim) {
                   st_direction(&st_d, 0);
                    // will need to figure out exact timing/step count for this...
                    // will have to happen after gantry is set up
                   st_speed(&st_d, 1000);
                }
                else {
                    stepper_state = 2;
                }
                break;
            case 2:
                st_speed(&st_d, 0);
                break;
        }
        // Drive DC Motor up until top y-axis endstop hit
        switch (motor_state) {
            case 0:
                if (!pin_read(Y_END_TOP)) {
                    dcm_velocity(&dcm1, 64000, 1);
                }
                else {
                    motor_state = 1;
                }
                break;
            case 1:
                if (pin_read(Y_END_TOP)) {
                    dcm_velocity(&dcm1, 64000, 0);
                }
                else {
                    motor_state = 2;
                }
                break;
            case 2:
                dcm_speed(&dcm1, 0);
                break;
        }
        if (stepper_state == 2 && motor_state == 2) {
            // rocket zero'd, ready to fly
            rocket_state == READY;
        }
    }

    // Check for state transitions

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
        motor_speed = 0;
        stepper_speed = 0;
    }

    // Perform state tasks
    // Handle tilt
    if (tilt == 1) {
        // drive servo to CCW
        led_on(&led3);
        // Send command to tilt rocket to left
        if (rocket_tilt < tilt_max) {
            rocket_tilt += 1;    
        }
    }
    else if (tilt == 2) {
        // drive servo CW
        led_on(&led3);
        // Send command to tilt rocket to right; i2c to servo driver
        if (rocket_tilt > tilt_min) {
            rocket_tilt -= 1;    
        }
    }
    else {
        led_off(&led3);
    }

    // Handle throttle
    if (throttle) {
        led_on(&led1);
    }
    else {
        led_off(&led1);
    }

    // Handle DC motor/stepper motor
    if (timer_flag(&timer2)) {
        timer_lower(&timer2);
        rocket_model();
    }

    // Check for state transitions

    // crash condition:
    if (rocket_state == CRASHED) {
        state = lose;
    }
    // landing condition
    if (rocket_state == LANDED){
        state = win;
    }

    if (state != last_state) {  // if we are leaving the state, do clean up stuff
        // turn off tilt stick led before exiting state
        led_off(&led3);
    }
}

void lose(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        timer_start(&timer1);
        counter = 0;
        motor_speed = 0;
        stepper_speed = 0;
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
        // trials++;  // if we are leaving the state, do clean up stuff
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
        // trials++;  // if we are leaving the state, do clean up stuff
    }
}

void setup() {
    timer_setPeriod(&timer1, 1);  // Timer for LED operation/status blink
    timer_setPeriod(&timer2, 0.5);  // Timer for UART servicing
    timer_setPeriod(&timer3, 0.01);
    timer_start(&timer1);
    timer_start(&timer2);
    timer_start(&timer3);

    // DC MOTOR + QUAD ENCODER
    dcm_init(&dcm1, &D[10], &D[11], 1e3, 0, &oc7);
    quad_init(&quad1, &D[8], &D[9]); // quad1 uses pins D8 & D9
    quad_every(&quad1, &timer5, 0.0000875); // quad1 will use timer5 interrupts
    
    // General use debugging output pin
    // pin_digitalOut(&D[2]);

    // Init endstop switches
    Y_END_TOP = &D[4];
    Y_END_BOT = &D[5];
    X_END_L = &D[6];
    X_END_R = &D[7];

    pin_digitalIn(Y_END_TOP);
    pin_digitalIn(Y_END_BOT);
    pin_digitalIn(X_END_L);
    pin_digitalIn(X_END_R);

    setup_uart();
    throttle, tilt = 0;
    val1, val2 = 8;
    rocket_tilt = 15;

}

void read_limitsw(_TIMER *timer){ //debounce the things
    sw_state = (sw_state<<1) | pin_read(&D[4]) | 0xe000;
    if(sw_state==0xf000){
        TOP_DSTOP = 1;
    }

    if(sw_state==0xefff){
        TOP_DSTOP = 0;
    }

    sw_state2 = (sw_state2<<1) | pin_read(&D[8]) | 0xe000; //changed from d5
    if(sw_state2==0xf000){
        BOT_DSTOP = 1;
    }

    if(sw_state2==0xefff){
        BOT_DSTOP = 0;
    }

    sw_state3 = (sw_state3<<1) | pin_read(&D[6]) | 0xe000;
    if(sw_state3==0xf000){
        LT_DSTOP = 1;
    }

    if(sw_state3==0xefff){
        LT_DSTOP = 0;
    }

    sw_state4 = (sw_state4<<1) | pin_read(&D[7]) | 0xe000;
    if(sw_state4==0xf000){
        RT_DSTOP = 1;
    }

    if(sw_state4==0xefff){
        RT_DSTOP = 0;
    }
}


int16_t main(void) {
    // printf("Starting Rocket Controller...\r\n");
    init_clock();
    init_ui();
    init_pin();
    init_timer();
    init_uart();
    init_quad();
    init_oc();
    // init_stepper();
    init_dcm();
    setup();

    pin_digitalIn(&D[4]); //TOP
    pin_digitalIn(&D[8]); //changed from d5 BOTTOM
    pin_digitalIn(&D[6]); //LEFT
    pin_digitalIn(&D[7]); //RIGHT

    timer_every(&timer4, .001, read_limitsw);

    // oc_pwm(&oc1, &D[4], &timer4, 3000, 32000);
    uint16_t counter = 0;
    uint64_t msg;
    char is_recip = 0;

    InitUSB();
    U1IE = 0xFF; //setting up ISR for USB requests
    U1EIE = 0xFF;
    IFS5bits.USB1IF = 0; //flag
    IEC5bits.USB1IE = 1; //enable
    state = flying;
    last_state = (STATE_HANDLER_T)NULL;


    // dcm_velocity(&dcm1, 64000, 1);

    while (1) {
        ServiceUSB();
        UARTrequests();
        state();
    }

    // ***Test code for quad encoder***
    // dcm_velocity(&dcm1, 64000, 1);
    // while (1) {
    //     if (timer_flag(&timer1)) {
    //         // Blink green light to show normal operation.
    //         timer_lower(&timer1);
    //         led_toggle(&led2);
    //     }

    //     if (timer_flag(&timer2)) {
    //         timer_lower(&timer2);
    //         // UARTrequests();
    //         if (quad1.counter > 5000)
    //         {
    //             led_on(&led1);
    //         } else {
    //             led_off(&led1);
    //         }
    //     }
    // }


            // *** test code for limit switches***
        // if (TOP_DSTOP==1){
        //     led_on(&led3);
        // }
        // else{
        //     led_off(&led3);
        // }

        // if (BOT_DSTOP==1){
        //     led_on(&led2);
        // }
        // else{
        //     led_off(&led2);
        // }

        // if (LT_DSTOP==1){
        //     led_on(&led1);
        // }
        // else{
        //     led_off(&led1);
        // }

}
