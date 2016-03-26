#include <p24FJ128GB206.h>
#include <libpic30.h>
#include "common.h"
#include "pin.h"
#include "i2c.h"
#include "servo.h"

void init_servo_driver(_SERVODRIVER *self, _I2C *bus, uint8_t hardware_addr) {

}

void init_servo(_SERVODRIVER *sd, uint8_t number) {

}


void servo_driver_sleep(_SERVODRIVER *self) {
    /*
    Puts servo driver into low-power sleep mode (to wake, use wake()) and
    stores mode1 settings to self -> mode1 for later reinstatement on wake.
    */
    servo_driver_begin_transmission(self, I2C_READ);
    self -> mode1 = servo_driver_read_register(self, PCA9685_MODE1);
    uint8_t sleep_mode = (self -> mode1 & 0x7F) | 0x10;
    servo_driver_end_transmission(self);
    // Put servo driver to sleep
    servo_driver_begin_transmission(self, I2C_WRITE);
    servo_driver_write_register(self, PCA9685_MODE1, sleep_mode);
    servo_driver_end_transmission(self)
}

void servo_driver_wake(_SERVODRIVER *self) {
    /*
    Wakes servo driver from sleep (to sleep, use servo_driver_sleep()) and
    reinstates mode1 settings from pre-sleep period (uses self -> mode1).
    */

    servo_driver_begin_transmission(self, I2C_WRITE);
    servo_driver_write_register(self, PCA9685_MODE1, self -> mode1);
    servo_driver_end_transmission(self)
    __delay_us(500); // Oscillator takes max 500us to restart from sleep
    servo_driver_begin_transmission(self, I2C_WRITE);
    servo_driver_write_register(PCA9685_MODE1, self -> mode1 | 0x81);
    servo_driver_end_transmission(self);
    // Servo driver is awake and functioning again!
}

void servo_driver_reset(_SERVODRIVER *self) {
    servo_driver_begin_transmission(self, I2C_WRITE);
    servo_driver_write_register(self, PCA9685_MODE1, 0x0);
    servo_driver_end_transmission(self);
}

void servo_driver_set_pwm_freq(_SERVODRIVER *self, float new_freq) {
    freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11).
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    uint8_t prescale = floor(prescaleval + 0.5);
    // PWM frequency can only be updated when servo driver is asleep.
    servo_driver_sleep(self);
    servo_driver_begin_transmission(self, I2C_WRITE);
    servo_driver_write_register(PCA9685_PRESCALE, prescale);
    servo_driver_end_transmission(self);
    servo_driver_wake(self);
}


void servo_driver_begin_transmission(_SERVODRIVER *dest, uint8_t rw) {
    /*
    Addresses the specified servo driver over I2C. This should be followed by 
    the writing of registers/other commands to this servo driver. rw specifies
    whether the following operations will be "read" (rw=1) or "write" (rw=0)
    operations.
    */
    i2c_start(dest -> bus);
    uint8_t assembled_addr = (dest -> i2c_addr << 1) + rw;
    i2c_putc(dest -> bus, assembled_addr);
}

void servo_driver_write_register(_SERVODRIVER *dest, uint8_t reg, uint8_t val) {
    i2c_putc(dest -> bus, reg);
    i2c_putc(dest -> bus, val);
}

uint8_t servo_driver_read_register(_SERVODRIVER *dest, uint8_t reg) {
    i2c_putc(dest -> bus, reg);
    return i2c_getc(dest -> bus);
}

void servo_driver_end_transmission (_SERVODRIVER *dest) {
    i2c_stop(dest -> bus);
}