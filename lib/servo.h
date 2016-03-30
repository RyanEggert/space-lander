/*
** I2C SERVO LIBRARY
** Adapted from Adafruit Arduino servo driver library
** (https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)
**
*/

#ifndef LIB_SERVO_H_
#define LIB_SERVO_H_

#include <stdbool.h>
#include "common.h"
#include "i2c.h"

#define I2C_WRITE 0
#define I2C_READ 1

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define DEV0_ON_L 0x6
#define DEV0_ON_H 0x7
#define DEV0_OFF_L 0x8
#define DEV0_OFF_H 0x9

#define ALLDEV_ON_L 0xFA
#define ALLDEV_ON_H 0xFB
#define ALLDEV_OFF_L 0xFC
#define ALLDEV_OFF_H 0xFD

// Servo Driver
typedef struct {
    _I2C *bus;
    uint8_t hardware_addr;
    uint8_t i2c_addr;
    float i2c_freq;
    uint8_t mode1;  // Normal, awake mode of operation [MODE1 register]
} _SERVODRIVER;

// Servo Driver
typedef struct {
    _SERVODRIVER *driver;
    uint8_t num;
} _SERVO;

extern _SERVODRIVER sd1;
extern _SERVO orientation_servo, speed_ind_servo, fuel_ind_servo, servo0, servo1, servo2, servo3, servo4;

void init_servo_driver(_SERVODRIVER *self, _I2C *bus, float i2c_freq, uint8_t hardware_addr);

void close_servo_driver_i2c(_SERVODRIVER *self);

void init_servo(_SERVO *self, _SERVODRIVER *sd, uint8_t number);

void servo_driver_sleep(_SERVODRIVER *self);

void servo_driver_wake(_SERVODRIVER *self);

void servo_driver_reset(_SERVODRIVER *self);

void servo_driver_set_pwm_freq(_SERVODRIVER *self, float new_freq);

void servo_set_pwm(_SERVO *self, uint16_t on, uint16_t off);

void servo_set(_SERVO *self, uint16_t val, bool invert);

void servo_driver_begin_transmission(_SERVODRIVER *dest, uint8_t rw);

void servo_driver_write_register(_SERVODRIVER *dest, uint8_t reg, uint8_t val);

uint8_t servo_driver_read_register(_SERVODRIVER *dest, uint8_t reg);

void servo_driver_end_transmission(_SERVODRIVER *dest);

#endif  // LIB_SERVO_H_
