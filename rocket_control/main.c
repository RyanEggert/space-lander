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

#define X_AXIS_THRUST 0
#define Y_AXIS_THRUST 1

// endstop pins
_PIN *Y_END_TOP, *Y_END_BOT, *X_END_L, *X_END_R;

typedef void (*STATE_HANDLER_T)(void);

void idle(void);
void reset(void);
void flying(void);
void win(void);
void lose(void);

STATE_HANDLER_T state, last_state;


volatile uint16_t sw_state = 0;
volatile uint16_t sw_state2 = 0;
volatile uint16_t sw_state3 = 0;
volatile uint16_t sw_state4 = 0;

volatile bool TOP_DSTOP, BOT_DSTOP, RT_DSTOP, LT_DSTOP;

// kinematic model vals
float thrust_val = 50.0;
float grav_val = 5.0;

// thrust angle LUT's; contain cos(theta) and sin(theta) vals
float angle_vals_LUT[10] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45};
float thrust_scale_y[10] = {1, 0.9961946981, 0.984807753, 0.9659258263, 0.9396926208, 0.906307787, 0.8660254038, 0.8191520443, 0.7660444431, 0.7071067812};
float thrust_scale_x[10] = {0, 0.08715574275, 0.1736481777, 0.2588190451, 0.3420201433, 0.4226182617, 0.5, 0.5735764364, 0.6427876097, 0.7071067812};
uint8_t LUT_ind;

// stepper vars
uint16_t stepper_count = 0;
uint16_t stepper_dir_track = 0;
uint8_t stepper_state = 0;  // 0 = drive to X_END_L, 1 = drive to middle, 2 = stop *** might not need this?
float stepper_speed = 0;
uint16_t stepper_speed_limit = 750;
uint16_t stepper_reset_lim = 1000;  // # of steps to move stepper during reset state
uint16_t stepper_deadband = 1;
uint16_t stepper_thrust;
uint16_t stepper_resist = 0x00F0;

// dc motor vars
uint16_t motor_state;
uint16_t motor_dir_track = 0;
uint16_t motor_speed = 0;
uint16_t motor_speed_limit = 0x7FFF;
uint16_t motor_deadband = 19000;  // will find once gantry is built
uint16_t motor_thrust;

// tilt vals (maxes out at 4096; may use smaller range)
#define tilt_max 450
#define tilt_min 120

float tilt_zero = (tilt_max + tilt_min)/2; //  
uint16_t tilt_ang;  //
float tilt_scale = 120.0/(tilt_max-tilt_min);  // scale factor to convert digital tilt val [deg/div]
uint16_t tilt_dir = 0;

// scaling vars
uint16_t scale_ind;
float scale_val_x;
float scale_val_y;

uint16_t search_ind, array_length;

// state + high-level rocket vals
uint16_t rocket_state = FLYING;
uint16_t rocket_speed;
float rocket_tilt;
float rocket_tilt_last;
uint16_t counter;
uint8_t throttle, tilt; //commands
uint8_t rocketstuff[64], rec_msg[64];
uint8_t cmd, value;
uint16_t val1, val2;

uint16_t binary_search(uint16_t target_val, float target_array[], uint16_t min, uint16_t max) {
    led_on(&led2);
    uint16_t curr_ind = (max + min )/2;
    uint16_t curr_val = target_array[curr_ind];
    uint16_t length = max-min;
    // base case
    if (length == 1 || length == 2) {
        led_off(&led2);
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
            led_off(&led2);
            // current value == 
            return curr_ind;
        }

    }
};

uint16_t linear_search(uint16_t target_val, float target_array[], uint16_t target_array_size) {
    // assume sorted list
    search_ind = 0;
    while (target_array[search_ind] < target_val && search_ind < target_array_size-1) {
        search_ind++;
    }
    return search_ind;
}

uint16_t get_thrust_scale_ind() {
    // translate tilt val into angle, get direction of tilt
    if (rocket_tilt != rocket_tilt_last && timer_flag(&timer3)) {
        timer_lower(&timer3);
        if (rocket_tilt > tilt_zero) {
            // tilt in CW direction
            tilt_ang = (uint16_t)((rocket_tilt - tilt_zero)*tilt_scale);
            tilt_dir = TILT_CW;
        }
        else if (rocket_tilt < tilt_zero) {
            // tilt in CCW direction
            tilt_ang = (uint16_t)((tilt_zero - rocket_tilt)*tilt_scale);
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
        stepper_thrust = (uint16_t)(thrust_val*scale_val_x);
        motor_thrust = (uint16_t)(thrust_val*scale_val_y);   // uint16_t = float * float;
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
                    stepper_dir_track = 0;
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
        if (stepper_speed > stepper_deadband + stepper_resist) {
            stepper_speed = stepper_speed - stepper_resist;
        }
        else {
            stepper_speed = 0;
        }
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
        // st_direction(&st_d, 0);
        // st_speed(&st_d, 150);
        // led_on(&led3);
        if (rocket_tilt < tilt_max) {
            rocket_tilt += 0.1;    
        }
    }
    else if (tilt == TILT_CW) {
        // drive servo CW
        // st_direction(&st_d, 1);
        // st_speed(&st_d, 150);   
        // led_on(&led3);
        if (rocket_tilt > tilt_min) {
            rocket_tilt -= 0.1;    
        }
    }
    else if (tilt == TILT_ZERO) {
        // stepper_speed = 0;
        // st_speed(&st_d, 0);
        // led_off(&led3);
    }

    if (timer_flag(&timer2)) {
        timer_lower(&timer2);
        // servo_set(&servo0, 350, 0);
        servo_set(&servo0, (uint16_t)(rocket_tilt), 0);
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
    else if (tilt== TILT_CCW) {
        if (stepper_speed < stepper_speed_limit) {
            if (timer_flag(&timer3)) {
                timer_lower(&timer3);
                if (stepper_speed == 0) {
                    stepper_speed = stepper_deadband;
                }
                else {
                    stepper_speed = stepper_speed + stepper_thrust;
                }            }
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
        BD[EP0IN].bytecount = 12;    // set EP0 IN byte count to 12
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
                if (stepper_count < stepper_reset_lim) {
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
        motor_speed = motor_deadband;
        rocket_tilt = tilt_zero;
        stepper_speed = 0;
    }

    // Perform state tasks

    // Write tilt to servo
    // servo_set(&orientation_servo, rocket_tilt, 0);

    // *** rocket model handles thrust scaling for x+y axes, drives DCM and stepper ***
    rocket_model();
    // *** use to determine stepper deadband over vendor requests ***
    // stepper_test();

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

void read_limitsw(_TIMER *timer){ //debounce the things
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

void setup() {
    timer_setPeriod(&timer1, 1);  // Timer for LED operation/status blink
    timer_setPeriod(&timer2, 0.01);  // Timer for UART servicing
    timer_setPeriod(&timer3, 0.1);
    timer_start(&timer1);
    timer_start(&timer2);
    timer_start(&timer3);

    // DC MOTOR + QUAD ENCODER
    dcm_init(&dcm1, &D[10], &D[11], 1e3, 0, &oc7, &es_y_bot, &es_y_top);
    quad_init(&quad1, &D[8], &D[9]); // quad1 uses pins D8 & D9
    quad_every(&quad1, &timer5, 0.0000875); // quad1 will use timer5 interrupts

    timer_every(&timer4, .001, read_limitsw); // Start timed endstop reading
    // General use debugging output pin
    // pin_digitalOut(&D[2]);

    // // Init endstop switches
    // Y_END_TOP = &D[4];
    // Y_END_BOT = &D[12];
    // X_END_L = &D[6];
    // X_END_R = &D[7];

    // pin_digitalIn(Y_END_TOP);
    // pin_digitalIn(Y_END_BOT);
    // pin_digitalIn(X_END_L);
    // pin_digitalIn(X_END_R);

    setup_uart();
    throttle, tilt = 0;
    val1, val2 = 8;
    rocket_tilt = 500;

}

int16_t main(void) {
    // printf("Starting Rocket Controller...\r\n");
    init_clock();
    init_ui();
    init_oc();
    init_pin();
    init_timer();
    init_st();      // if this is first, then D[1] - D[3] don't work as outputs
    init_quad();
    init_dcm();
    init_i2c();
    init_servo_driver(&sd1, &i2c3, 16000., 0x0);
    init_uart();    // if this is first, then D[0] doesn't output OC wfm
    setup();
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

    st_state(&st_d, 1);
    // servo_set(&servo0, 150, 0);
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
