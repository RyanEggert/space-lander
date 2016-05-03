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

uint8_t TXBUF1[1024], RXBUF1[1024];
uint8_t TXBUF2[1024], RXBUF2[1024];
char rx_msg[64], tx_msg[64];

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

void idle(void);
void reset(void);
void reset_from_origin(void);
void reset_to_game_over(void);
void flying(void);
void win(void);
void lose(void);

STATE_HANDLER_T state, last_state;

volatile uint16_t sw_state = 0;
volatile uint16_t sw_state2 = 0;
volatile uint16_t sw_state3 = 0;
volatile uint16_t sw_state4 = 0;

volatile bool TOP_DSTOP, BOT_DSTOP, RT_DSTOP, LT_DSTOP;  // Not needed. Can remove if endstop test code from below also removed.

// tilt vals (maxes out at 4096; may use smaller range)
#define tilt_max 450
#define tilt_min 120

// maximum value for fuel (0xFFFF)
#define fuel_max 65535

// rocket fuel vals
uint16_t fuel_val = 0xFFFF;
uint16_t fuel_burn_rate = 0.75;
float fuel_scale = (tilt_max - tilt_min) / fuel_max;
uint16_t fuel_offset = tilt_min;
uint16_t fuel_servo_set;


// kinematic model vals
float thrust_val = 5.0;
float grav_val = 4.0;
float stepper_thrust_val = 5;

// thrust angle LUT's; contain cos(theta) and sin(theta) vals
float angle_vals_LUT[10] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45};
float thrust_scale_y[10] = {1, 0.9961946981, 0.984807753, 0.9659258263, 0.9396926208, 0.906307787, 0.8660254038, 0.8191520443, 0.7660444431, 0.7071067812};
float thrust_scale_x[10] = {0, 0.08715574275, 0.1736481777, 0.2588190451, 0.3420201433, 0.4226182617, 0.5, 0.5735764364, 0.6427876097, 0.7071067812};
uint8_t LUT_ind;

// stepper vars
uint16_t stepper_count = 0;
uint16_t stepper_dir_track = 0;
uint8_t stepper_state = 0;  // 0 = drive to left x endstop, 1 = drive to middle, 2 = stop *** might not need this?
float stepper_speed = 0;
float stepper_speed_limit = 1500;
uint16_t stepper_reset_lim = 1000;  // # of steps to move stepper during reset state
float stepper_deadband = 250;
float stepper_thrust;
float stepper_resist = 0.1;

// dc motor vars
uint16_t motor_state;
uint16_t motor_dir_track = 0;
uint16_t motor_speed = 0;
// uint16_t motor_speed_limit = 0x7FFF;
#define motor_speed_limit_constant 49151;
uint16_t motor_speed_limit = motor_speed_limit_constant;
#define motor_deadband_constant 7000
uint16_t motor_deadband = motor_deadband_constant;  // will find once gantry is built
#define motor_speed_range 32767 - 7000
uint16_t motor_thrust;

// tilt vars
float tilt_zero = (tilt_max + tilt_min) / 2; //
uint16_t tilt_ang;  //
float tilt_scale = 120.0 / (tilt_max - tilt_min); // scale factor to convert digital tilt val [deg/div]
uint16_t tilt_dir = 0;
float tilt_incr = 0.01;

// scaling vars
uint16_t scale_ind;
float scale_val_x;
float scale_val_y;

uint16_t search_ind, array_length;

// state + high-level rocket vals
uint16_t rocket_state = FLYING;
uint16_t rocket_speed;
uint16_t rocket_speed_servo_set;
// uint16_t rocket_speed_range = motor_speed_limit_constant - motor_deadband_constant;
float rocket_speed_scale = (tilt_max - tilt_min) / motor_speed_range;
uint16_t rocket_speed_offset = tilt_min;
float rocket_tilt;
float rocket_tilt_last;
uint16_t counter;
uint8_t throttle, tilt; //commands
uint8_t rocketstuff[64], rec_msg[64];
uint8_t cmd, value;

void uart_sendstr(uint8_t *str) {
    /*
    Sends a string on uart2. UNTESTED
    */
    printf("SENDING: %s\n\r", tx_msg);
    uart_puts(&uart2, str);
}

void uart_send(uint16_t value) {
    /*
    Formats and sends a value on uart2. Formats "value" as a hexadecimal string.
    */
    sprintf(tx_msg, "%x\r", value);
    printf("SENDING: %s\n\r", tx_msg);
    uart_puts(&uart2, tx_msg);
}

uint32_t uart_receive() {
    /*
    Non-blockingly receieves a string on uart1. Returns -1 if no data available
    on uart1. Else returns the string received on uart1 parsed as a hexadecimal
    uint32_t.
    */
    char *ptr;
    uint32_t decoded_msg;
    uart_gets(&uart1, rx_msg, 64);
    // printf("REC: %s\n\r", rx_msg);
    if (rx_msg[0] == '\0') {  // If first char is null, then no data available
        decoded_msg = -1;  // Return -1
    } else {
        decoded_msg = strtol(rx_msg, &ptr, 16);  // Else return hex parsing
    }
    return decoded_msg;
}

uint16_t binary_search(uint16_t target_val, float target_array[], uint16_t min, uint16_t max) {
    // led_on(&led2);
    uint16_t curr_ind = (max + min ) / 2;
    uint16_t curr_val = target_array[curr_ind];
    uint16_t length = max - min;
    // base case
    if (length == 1 || length == 2) {
        // led_off(&led2);
        return curr_ind;
    }
    // recursive case
    else {
        if (curr_val < target_val) {
            // current value is too small, search larger half of array
            return binary_search(target_val, target_array, curr_ind, max);
        }
        else if (curr_val > target_val) {
            // current value is too large, search smaller half of array
            return binary_search(target_val, target_array, min, curr_ind);
        }
        else {
            // led_off(&led2);
            // current value ==
            return curr_ind;
        }

    }
};

uint16_t linear_search(uint16_t target_val, float target_array[], uint16_t target_array_size) {
    // assume sorted list
    search_ind = 0;
    while (target_array[search_ind] < target_val && search_ind < target_array_size - 1) {
        search_ind++;
    }
    return search_ind;
}

uint16_t get_thrust_scale_ind() {
    if (fuel_val > 0) {
        // fuel still greater than 0; continue decrementing
        fuel_val -= fuel_burn_rate;
    }
    else if (fuel_val == 0) {
        // fuel exhausted; round lost
        rocket_state = CRASHED;
    }
    // translate tilt val into angle, get direction of tilt
    if (rocket_tilt != rocket_tilt_last && timer_flag(&timer2)) {
        timer_lower(&timer2);
        if (rocket_tilt > tilt_zero) {
            // tilt in CW direction
            tilt_ang = (uint16_t)((rocket_tilt - tilt_zero) * tilt_scale);
            tilt_dir = TILT_CW;
        }
        else if (rocket_tilt < tilt_zero) {
            // tilt in CCW direction
            tilt_ang = (uint16_t)((tilt_zero - rocket_tilt) * tilt_scale);
            tilt_dir = TILT_CCW;
        }
        else {
            // tilt_ang = tilt_zero*tilt_scale;
            tilt_ang = 0;
            tilt_dir = TILT_ZERO;
        }
        // search LUT for index of corresponding angle
        array_length = sizeof(angle_vals_LUT) / sizeof(angle_vals_LUT[0]);
        scale_ind = linear_search(tilt_ang, angle_vals_LUT, array_length);
        // scale_ind = binary_search(tilt_ang, angle_vals_LUT, 0, 9);
        // scale_val_x = thrust_scale_x[scale_ind];
        // scale_val_y = thrust_scale_y[scale_ind];
        rocket_tilt_last = rocket_tilt;
    }
    return scale_ind;
}

void rocket_model() {
    // handle fuel value
    if (fuel_val > 0) {
        // fuel still greater than 0; continue decrementing
        fuel_val -= fuel_burn_rate;
    }
    else if (fuel_val == 0) {
        // fuel exhausted; round lost
        rocket_state = CRASHED;
    }
    // determines speed setpoint of rocket in x + y axes
    if (throttle) { // Thrust on
        led_on(&led1);
        // find thrust scale vals
        LUT_ind = get_thrust_scale_ind();
        scale_val_x = thrust_scale_x[LUT_ind];
        scale_val_y = thrust_scale_y[LUT_ind];
        // placeholder thrust scale vals
        // scale_val_x = thrust_scale_x[0];
        // scale_val_y = thrust_scale_y[0];
        // scale thrust in x+y axes
        stepper_thrust = (float)(stepper_thrust_val * scale_val_x);
        motor_thrust = (uint16_t)(thrust_val * scale_val_y); // uint16_t = float * float;
        if (motor_dir_track == 0) {  // rocket falling
            // led_on(&led2);
            if (motor_speed - motor_deadband > motor_thrust) {  // nonzero velocity
                motor_speed = motor_speed - motor_thrust;
            }
            else {  // zero velocity
                motor_dir_track = 1;
                motor_speed = motor_deadband;
            }
        }
        else if (motor_dir_track) { // rocket rising
            // led_off(&led2);
            if (motor_speed < motor_speed_limit) {
                motor_speed = motor_speed + motor_thrust;
            }
        }
        // ***set stepper thrust val***
        // direction of thrust is dependent on tilt direction
        // if (timer_flag(&timer3)) {
        if (stepper_dir_track) {
            /// stepper_dir_track == 1 denotes motion to left
            if (tilt_dir == TILT_CW) {
                // tilting to right
                if (stepper_speed - stepper_deadband > stepper_thrust) {
                    stepper_speed = stepper_speed - stepper_thrust;
                }
                else {
                    stepper_dir_track = 0;
                    stepper_speed = stepper_deadband + stepper_thrust;
                }
            }
            else if (tilt_dir == TILT_CCW) {
                // tilting to left
                if (stepper_speed < stepper_speed_limit) {
                    stepper_speed = stepper_speed + stepper_thrust;
                }
            }
            else if (tilt_dir == TILT_ZERO) {
                // no tilt
                if (stepper_speed > stepper_deadband + stepper_resist) {
                    stepper_speed = stepper_speed - stepper_resist;
                }
                else {
                    // wind resistance stops x-axis motion of rocket
                    stepper_speed = 0;
                }
            }
        }
        else if (!stepper_dir_track) {
            /// stepper_dir_track == 0 denotes motion to right
            if (tilt_dir == TILT_CW) {
                // tilting to left
                if (stepper_speed < stepper_speed_limit) {
                    stepper_speed = stepper_speed + stepper_thrust;
                }
            }
            else if (tilt_dir == TILT_CCW) {
                // tilting to right
                if (stepper_speed - stepper_deadband > stepper_thrust) {
                    stepper_speed = stepper_speed - stepper_thrust;
                }
                else {
                    stepper_dir_track = 1;
                    stepper_speed = stepper_deadband + stepper_thrust;
                }
            }
            else {
                // no tilt
                if (stepper_speed > stepper_deadband + stepper_resist) {
                    stepper_speed = stepper_speed - stepper_resist;
                }
                else {
                    // wind resistance stops x-axis motion of rocket
                    stepper_speed = 0;
                }
            }
        }
        //     timer_lower(&timer3);
        // }
        // led_on(&led2);
    }
    else { // no thrust
        led_off(&led1);
        // set y thrust val
        if (motor_dir_track == 0) { // rocket falling
            // led_on(&led2);
            if (motor_speed < motor_speed_limit) {
                motor_speed = motor_speed + grav_val;
            }
        }
        else if (motor_dir_track) {  // rocket rising
            // led_off(&led2);
            if (motor_speed - motor_deadband > thrust_val) {
                motor_speed = motor_speed - grav_val;
            }
            else {
                motor_dir_track = 0;
                motor_speed = motor_deadband;
            }
        }
        // set x thrust
        // if (timer_flag(&timer3)) {
        if (stepper_speed > stepper_deadband + stepper_resist) {
            stepper_speed = stepper_speed - stepper_resist;
        }
        else {
            stepper_speed = 0;
        }
        //     timer_lower(&timer3);
        // }
    }
    // drive DC motor
    if (motor_dir_track) {
        // pin_set(MOTOR_DIR);
        dcm_velocity(&dcm1, motor_speed, 0);
    }
    else {
        dcm_velocity(&dcm1, motor_speed, 1);
    }
    // drive stepper
    if (stepper_speed == 0) {
        st_speed(&st_d, 0);
    }
    else {
        if (stepper_dir_track) {
            st_direction(&st_d, 1);
            st_speed(&st_d, stepper_speed);
        }
        else {
            st_direction(&st_d, 0);
            st_speed(&st_d, stepper_speed);
        }
    }
    // Handle tilt
    if (tilt == TILT_CCW) {
        // drive servo to CCW
        st_direction(&st_d, 0);
        st_speed(&st_d, 150);
        led_on(&led3);
        if (rocket_tilt < tilt_max) {
            rocket_tilt += tilt_incr;
        }
    }
    else if (tilt == TILT_CW) {
        // drive servo CW
        st_direction(&st_d, 1);
        st_speed(&st_d, 150);
        led_on(&led3);
        if (rocket_tilt > tilt_min) {
            rocket_tilt -= tilt_incr;
        }
    }
    else if (tilt == TILT_ZERO) {
        stepper_speed = 0;
        st_speed(&st_d, 0);
        led_off(&led3);
    }

    if (timer_flag(&timer2)) {
        timer_lower(&timer2);
        // servo_set(&servo0, 350, 0);
        servo_set(&servo0, (uint16_t)(rocket_tilt), 0);
        fuel_servo_set = (uint16_t)((float)(fuel_val)*fuel_scale)+fuel_offset;
        printf("fuel_scale:%d\n\r", fuel_scale);
        printf("fuel_servo_set:%d\n\r", fuel_servo_set);
        servo_set(&servo4, fuel_servo_set, 0);
        // scale rocket_speed to servo drive value
        rocket_speed_servo_set = (uint16_t)((float)(motor_speed)*rocket_speed_scale)+rocket_speed_offset;
        printf("rocket_speed_scale:%d\n\r", rocket_speed_scale);
        printf("rocket_speed_servo_set:%d\n\r", rocket_speed_servo_set);
        servo_set(&servo5, rocket_speed_servo_set, 0);
    }

    rocket_speed = motor_speed;
}

void stepper_test() {
    if (tilt == TILT_CW) {
        stepper_thrust = 1;
        // increase velocity to right
        if (stepper_speed < stepper_speed_limit) {
            if (timer_flag(&timer3)) {
                timer_lower(&timer3);
                if (stepper_speed == 0) {
                    stepper_speed = stepper_deadband;
                }
                else {
                    stepper_speed = stepper_speed + stepper_thrust;
                }
            }
        }
        stepper_dir_track = 0;
        led_on(&led3);
    }
    else if (tilt == TILT_CCW) {
        if (stepper_speed < stepper_speed_limit) {
            if (timer_flag(&timer3)) {
                timer_lower(&timer3);
                if (stepper_speed == 0) {
                    stepper_speed = stepper_deadband;
                }
                else {
                    stepper_speed = stepper_speed + stepper_thrust;
                }
            }
        }
        stepper_dir_track = 1;
        led_on(&led3);
    }
    else if (tilt == TILT_ZERO) {
        stepper_speed = 0;
        led_off(&led3);
    }
    st_direction(&st_d, stepper_dir_track);
    st_speed(&st_d, stepper_speed);
}

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

    case DEBUG_OC_STATUS:
        temp.b[0] = bitread(oc7.OCxCON1, 0);  // Receive buffer data available
        temp.b[1] = bitread(oc7.OCxCON1, 1);  // Read overrun error bit
        BD[EP0IN].address[0] = temp.b[0];  // OCM0
        BD[EP0IN].address[1] = temp.b[1];  // OCM1
        temp.b[0] = bitread(oc7.OCxCON1, 2);  // Read framing error bit
        temp.b[1] = bitread(oc7.OCxCON1, 3);  // Read parity error bit
        BD[EP0IN].address[2] = temp.b[0];  // OCM2
        BD[EP0IN].address[3] = temp.b[1];  // OCTSEL
        temp.b[0] = bitread(oc7.OCxCON1, 4);  // Read receiver idle bit
        BD[EP0IN].address[4] = temp.b[0];  // OCTFLT
        temp.b[0] = bitread(oc5.OCxCON1, 0);  // Receive buffer data available
        temp.b[1] = bitread(oc5.OCxCON1, 1);  // Read overrun error bit
        BD[EP0IN].address[5] = temp.b[0];  // OCM0
        BD[EP0IN].address[6] = temp.b[1];  // OCM1
        temp.b[0] = bitread(oc5.OCxCON1, 2);  // Receive buffer data available
        temp.b[1] = bitread(oc5.OCxCON1, 3);  // Read overrun error bit
        BD[EP0IN].address[7] = temp.b[0];  // OCM2
        BD[EP0IN].address[8] = temp.b[1];  // OCTFLT
        temp.b[0] = bitread(oc5.OCxCON1, 4);  // Read receiver idle bit
        BD[EP0IN].address[9] = temp.b[0];  // OCTFLT

        BD[EP0IN].bytecount = 10;    // set EP0 IN byte count to 9
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit

    case GET_ROCKET_INFO:
        temp.w = rocket_tilt;
        BD[EP0IN].address[0] = temp.b[0];
        BD[EP0IN].address[1] = temp.b[1];
        temp.w = rocket_speed;
        BD[EP0IN].address[2] = temp.b[0];
        BD[EP0IN].address[3] = temp.b[1];
        temp.w = throttle;
        BD[EP0IN].address[4] = temp.b[0];
        BD[EP0IN].address[5] = temp.b[1];
        temp.w = motor_speed;
        BD[EP0IN].address[6] = temp.b[0];
        BD[EP0IN].address[7] = temp.b[1];
        temp.w = motor_thrust;
        BD[EP0IN].address[8] = temp.b[0];
        BD[EP0IN].address[9] = temp.b[1];
        temp.w = tilt_ang;
        BD[EP0IN].address[10] = temp.b[0];
        BD[EP0IN].address[11] = temp.b[1];
        temp.w = tilt_dir;
        BD[EP0IN].address[12] = temp.b[0];
        BD[EP0IN].address[13] = temp.b[1];
        temp.w = (int16_t)(stepper_speed);
        BD[EP0IN].address[14] = temp.b[0];
        BD[EP0IN].address[15] = temp.b[1];
        temp.w = fuel_servo_set;
        BD[EP0IN].address[16] = temp.b[0];
        BD[EP0IN].address[17] = temp.b[1];
        BD[EP0IN].bytecount = 18;    // set EP0 IN byte count to 14
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

    case DEBUG_SERVO_SET_POS:
        temp.w = USB_setup.wValue.w;   // Commanded position
        temp2.w = USB_setup.wIndex.w;  // Servo driver index
        servo_usb_set(&sd1, temp2.b[0], temp.w);
        BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
        BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        break;

    case DEBUG_SERVO_SET_FREQ:
        temp.w = USB_setup.wValue.w;
        servo_driver_configure(&sd1, ((float)(temp.w)) / 10);
        BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
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


void idle(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
    }

    if (timer_flag(&timer2)) {
        timer_lower(&timer2);
        uint32_t coin_msg;
        // Call uart_receive(). We are waiting for one of the following messages:
        //    * A coin has been inserted
        coin_msg = uart_receive();

        if (coin_msg == -1) {
            // No UART data available
            state = idle;
        } else if (coin_msg == 121) {
            // Coin message received
            state = reset;  // If coin, then proceed to reset state.
        } else {
            // Some other message receieved.
            // DANGER, why are we here?
        }
    }

    if (state != last_state) {  // if we are leaving the state, do clean up stuff

    }
}

void reset_from_origin(void) {
    /*
    Moves rocket from origin position to reset position (top middle). Moves
    on to flying state once reset position reached.
    */
    static bool dc_reset;
    static bool stepper_reset;
    static uint16_t reset_steps;

    if (state != last_state) {
        // State initialization
        last_state = state;
        led_on(&led3);


        // Set up stepper positioning system
        st_direction(&st_d, 1);  // Drive stepper right
        st_manual_init(&st_d, 30);
        stepper_reset = false;
        // float pulley_rad = 6.35;  // Radius of pulley in mm
        // float dist_const = (0.0279253 * pulley_rad) / (8); // (1.6 degrees -> radians) * belt pulley radius (mm) / (8th steps)
        // uint16_t reset_dist = 220;  // Distance from origin to reset position (mm)
        // reset_steps = (uint16_t)(reset_dist / dist_const); // No. steps from origin to reset position
        reset_steps = 9925;  // Move to center
        printf("RESET_STEPS: %d\n\r", reset_steps);
        // zero quad encoder

        // Set up DC motor positioning system
        dcm_velocity(&dcm1, 20000, 1);  // Drive motor downwards

        dc_reset = false;
    }

    dcm_velocity(&dcm1, 20000, 1);  // Drive motor downwards

    if (st_d.manual_count >= reset_steps ) {
        // Stepper has reached reset location.
        stepper_reset = true;
    } else {
        // Stepper has not reached reset location.
        st_manual_toggle(&st_d);
    }

    if (!(dcm1.stop_max->hit)) {
        // DC motor has cleared its top endstop.
        dcm_stop(&dcm1);
        dc_reset = true;
    }

    if (dc_reset == true && stepper_reset == true) {
        // Both axes have reached their reset positions.
        // Move to next state
        // if()
        state = flying;
    }

    if (state != last_state) {
        // State exit
        led_off(&led3);

        st_manual_exit(&st_d); // Leave manual toggling mode
        st_stop(&st_d); // Make sure stepper is stationary.

        // Call uart_send(). When we are done resetting, we must inform the
        // master PIC.
        uart_send(4313);  // Send 4313 to indicate reset to game start complete.
    }
}

void reset_to_game_over(void) {
    /*
    Moves rocket from origin position to game-over position (below top left). Moves
    on to idle state once reset position reached.
    */
    static bool dc_reset;
    static bool stepper_reset;
    static uint16_t reset_steps;

    if (state != last_state) {
        // State initialization
        last_state = state;
        led_on(&led3);

        // Set up stepper positioning system
        st_direction(&st_d, 1);  // Drive stepper right
        st_manual_init(&st_d, 30);
        stepper_reset = false;
        // float pulley_rad = 6.35;  // Radius of pulley in mm
        // float dist_const = (0.0279253 * pulley_rad) / (8); // (1.6 degrees -> radians) * belt pulley radius (mm) / (8th steps)
        // uint16_t reset_dist = 15;  // Distance from origin to reset position (mm)
        // reset_steps = (uint16_t)(reset_dist / dist_const); // No. steps from origin to reset position
        reset_steps = 1800;  // Move to center
        printf("RESET STEPS: %d\n\r", reset_steps);
        // zero quad encoder

        // Set up DC motor positioning system
        dcm_velocity(&dcm1, 20000, 1);  // Drive motor downwards

        dc_reset = false;
    }

    dcm_velocity(&dcm1, 20000, 1);  // Drive motor downwards


    if (st_d.manual_count >= reset_steps ) {
        // Stepper has reached reset location.
        stepper_reset = true;
    } else {
        // Stepper has not reached reset location.
        stepper_reset = false;
        st_manual_toggle(&st_d);
    }

    if (!(dcm1.stop_max->hit)) {
        // DC motor has cleared its top endstop.
        dcm_stop(&dcm1);
        dc_reset = true;
    }

    if (dc_reset && stepper_reset) {
        // Both axes have reached their reset positions.
        // Move to next state
        // if()
        state = idle;
    }

    if (state != last_state) {
        // State exit
        st_manual_exit(&st_d); // Leave manual toggling mode
        st_stop(&st_d); // Make sure stepper is stationary.

        // Call uart_send(). When we are done resetting, we must inform the
        // master PIC.
        uart_send(4315);  // Send 4315 to indicate reset to game over complete.
    }
}

void reset(void) {
    /*
    This state resets the XY gantry to its origin (top left).
    Once the origin is reached, the FSM moves on to the `reset_from origin`
    state.
    */
    uint32_t trials_msg;
    uint16_t rxd_trials;
    static uint8_t trials;
    static bool rxd_trials_flag;

    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        stepper_count = 0;
        stepper_state = 0;  // drive to X_END_L
        // Move motors towards reset position.
        dcm_velocity(&dcm1, 40000, 0);  // Drive upwards at 40000.

        st_direction(&st_d, 0);  // Drive stepper left.
        st_speed(&st_d, 1000);  // Drive stepper left.

        rxd_trials_flag = false;  // No new trials received.
        // led_on(&led2);
    }
    dcm_velocity(&dcm1, 40000, 0);  // Drive upwards at 40000.
    st_direction(&st_d, 0);  // Drive stepper left.
    st_speed(&st_d, 1000);  // Drive stepper left.

    if (timer_flag(&timer2)) {
        timer_lower(&timer2);
        trials_msg = uart_receive();
        rxd_trials = trials_msg >> 8;
        if (trials_msg == -1) {
            // No UART data available
            state = idle;
        } else if (rxd_trials <= 3) {  // Where 3 is max trials
            // New trials value received
            trials = rxd_trials;
            rxd_trials_flag = true;
        } else {
            // Some other message receieved.
            // DANGER, why are we here?
        }
    }
    // Perform state tasks
    // Perform state tasks
        if ((st_d.stop_min->hit) && (dcm1.stop_max->hit) && rxd_trials_flag) {
            // We are in top left corner and have been told how many trials are left.
            servo_set(&servo0, tilt_zero, 0);

            if (rxd_trials == 3) {
                // if no trials remain, reset to game over position.
                state = reset_to_game_over;
            } else {
                // If 1, 2, or 3 trials, reset to start game position.
                state = reset_from_origin;
            }
        } else {
            state = reset;
        }

    if (state != last_state) {
        dcm_stop(&dcm1);
        st_stop(&st_d);
    }
}

void flying(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        motor_speed = motor_deadband;
        rocket_tilt = tilt_zero;
        stepper_speed = 0;
        rocket_state = FLYING;
        fuel_val = 0xFFFF;
        printf("ENTER FLYING STATE");
    }

    // Call uart_receive(). We are waiting for one of the following messages:
    //    * Status message with tilt and throttle commands
    uint32_t command_msg;
    command_msg = uart_receive();
    if (command_msg == -1) {
        // No UART data available
    } else if (command_msg <= 7) {  // If we have received a valid command_msg,
        // then update throttle and tilt accordingly.
        throttle = command_msg & 0b01;
        tilt = (command_msg & 0b110) >> 1;
    } else {
        // Some other message receieved.
        // DANGER, why are we here?
    }

    if (timer_flag(&timer2)) {
        timer_lower(&timer2);
    }

    // *** rocket model handles thrust scaling for x+y axes, drives DCM, stepper, servo ***

    // if (timer_flag(&timer3)) {
    //     timer_lower(&timer3);
    rocket_model();
    // }
    // *** use to determine stepper deadband over vendor requests ***
    // stepper_test();

    // Check for landings, intentional and non-intentional
    if (es_x_l.hit | es_x_r.hit | es_y_top.hit | es_y_bot.hit) {
        // If gantry endstops pressed, rocket crashes, player loses.
        rocket_state = CRASHED;
        // state = lose;
    }

    if (es_landing.hit == 1) {
        // Rocket has landed on the barge.
        // led_on(&led2);
        if (abs(rocket_speed) <= 15000 && abs(tilt_ang) <= 30) {
            // If rocket has landed within a range of feasible parameter values,
            rocket_state = LANDED;  // then the landing is successful.
        } else {
            rocket_state = CRASHED;  // else, the rocket crashes.
        }
    }


    // crash condition:
    if (rocket_state == CRASHED) {
        state = lose;
        uart_send(911);  // Notify master that rocket has crashed.
    }
    // landing condition
    if (rocket_state == LANDED) {
        state = win;
        uart_send(10000);  // Notify master that rocket has landed.
    }

    if (state != last_state) {  // if we are leaving the state, do clean up stuff
        // turn off tilt stick led before exiting state
        led_off(&led3);
        st_stop(&st_d);  // Stop motor
        dcm_stop(&dcm1);  // Stop motor
    }
}

void lose(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        timer_start(&timer1);
        counter = 0;
        motor_speed = 0;
        stepper_speed = 0;
        st_stop(&st_d);  // Be absolutely sure we've stopped motor
        dcm_stop(&dcm1);  // Be absolutely sure we've stopped motor
    }
    // TODO: Have UART signal from master prompt switch to reset state.
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
        printf("EXIT FLYING STATE");

    }
}

void win(void) {
    if (state != last_state) {  // if we are entering the state, do initialization stuff
        last_state = state;
        timer_start(&timer1);
        st_stop(&st_d);  // Be absolutely sure we've stopped motor
        dcm_stop(&dcm1);  // Be absolutely sure we've stopped motor
        counter = 0;
    }
    // TODO: Have UART signal from master prompt switch to reset state.
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

void setup_uart() {
    /*
    Configures UART for communications.
    Uses uart1 to receive messages from master PIC on ICD3 header (RX2, TX2).
    Uses uart2 to send messages to master PIC on ICD3 header (RTS2, CTS2).
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
    // TIMERS
    timer_setPeriod(&timer1, 1);  // Timer for LED operation/status blink
    timer_setPeriod(&timer2, 0.01);  // Motor clocking timer
    timer_setPeriod(&timer3, 0.5);  // General use timer (state-by-state basis)
    timer_setPeriod(&timer5, 0.2);  // Timer for debug state printf
    timer_start(&timer1);
    timer_start(&timer2);
    timer_start(&timer3);
    timer_start(&timer5);

    // DC MOTOR + QUAD ENCODER
    dcm_init(&dcm1, &D[10], &D[11], 1e3, 0, &oc7, &es_y_bot, &es_y_top);
    // quad_init(&quad1, &D[8], &D[9]); // quad1 uses pins D8 & D9
    // quad_every(&quad1, &timer5, 0.0000875); // quad1 will use timer5 interrupts

    // STEPPER
    st_init(&st_d, &D[0], &D[1], &D[2], &D[3], &oc5, 0x7FFF, &es_x_l, &es_x_r);
    st_state(&st_d, 1);

    // LIMIT SWITCHES
    timer_every(&timer4, .001, read_limitsw);  // Start timed endstop reading

    // UART
    setup_uart();

    throttle, tilt = 0;
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
    init_i2c();
    init_servo_driver(&sd1, &i2c3, 16000., 0x0);
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

    // Initialize State Machine
    state = idle;
    last_state = (STATE_HANDLER_T)NULL;

    pin_digitalOut(&D[5]);  // Heartbeat pin
    while (1) {
        if (timer_flag(&timer5)) {
            timer_lower(&timer5);
            uint8_t state_num = -1;
            if (state == idle) {
                printf("State: IDLE\n\r");
            } else if (state == reset) {
                printf("State: RESET\n\r");
            } else if (state == reset_from_origin) {
                printf("State: RESET_FROM_ORIGIN\n\r");
            } else if (state == reset_to_game_over) {
                printf("State: RESET_TO_GAME_OVER\n\r");
            } else if (state == flying) {
                printf("State: FLYING\n\r");
            } else if (state == win) {
                printf("State: WIN\n\r");
            } else if (state == lose) {
                printf("State: LOSE\n\r");
            } else {
                printf("State: UNKNOWN STATE\n\r");
            }
        }
        state();
        pin_toggle(&D[5]);  // Heartbeat
    }
}
