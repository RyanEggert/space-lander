#include "common.h"
#include "pin.h"
#include "i2c.h"
#include "servo.h"

void init_servo_driver(_SERVODRIVER *self, _I2C *bus, uint8_t hardware_addr) {

}

void init_servo(_SERVODRIVER *sd, uint8_t number) {

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
    if (ENABLE_DEBUG_OUTPUT) {
      //Serial.print("Estimated pre-scale: "); Serial.println(prescaleval);
    }
    uint8_t prescale = floor(prescaleval + 0.5);
    if (ENABLE_DEBUG_OUTPUT) {
      //Serial.print("Final pre-scale: "); Serial.println(prescale);
    }
    
    uint8_t oldmode = read8(PCA9685_MODE1);
    uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
    write8(PCA9685_MODE1, newmode); // go to sleep
    write8(PCA9685_PRESCALE, prescale); // set the prescaler
    write8(PCA9685_MODE1, oldmode);
    delay(5);
    write8(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
                                            // This is why the beginTransmission below was not working.
    //  Serial.print("Mode now 0x"); Serial.println(read8(PCA9685_MODE1), HEX);
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

void servo_driver_end_transmission (_SERVODRIVER *dest) {
    i2c_stop(dest -> bus);
}