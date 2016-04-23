/*
** I2C SERVO LIBRARY
** Adapted from Adafruit Arduino servo driver library
** (https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)
**
*/

#include <p24FJ128GB206.h>
#include "common.h"
#include <libpic30.h>
#include <math.h>
#include <stdbool.h>
// #include "ui.h"
#include "pin.h"
#include "i2c.h"
#include "servo.h"

#define delay_ms_time 1  // delay between i2c transactions in servo_drive_configure function

_SERVODRIVER sd1;
_SERVO servo0, servo1, servo2, servo3, servo4, servo5, servo6, servo7, servo8, servo9, servo10, servo11, servo12, servo13, servo14, servo15;

void init_servo_driver(_SERVODRIVER *self, _I2C *bus, float i2c_freq, uint8_t hardware_addr) {
    /*
    Initializes an I2C servo driver device given an initialized _I2C object
    and its 6-bit slave hardware address, as configured on the device.
    */
    self -> bus = bus;
    self -> hardware_addr = 0x3F & (hardware_addr);
    self -> i2c_addr = 0x40 | self -> hardware_addr; // Add preceding 1 to addr
    self -> mode1 = 0b10100001;  // Suggested mode1 configuration [0xa1]
    self -> mode2 = 0b00000100;  // Suggested mode2 configuration
    self -> i2c_freq = i2c_freq;
    i2c_open(self -> bus, i2c_freq); // Open I2C bus
    servo_driver_configure(self, 50);  // 50 Hz. default PWM frequency
    init_servo(&servo0, self, 0);
    init_servo(&servo1, self, 1);
    init_servo(&servo2, self, 2);
    init_servo(&servo3, self, 3);
    init_servo(&servo4, self, 4);
    init_servo(&servo5, self, 5);
    init_servo(&servo5, self, 5);
    init_servo(&servo7, self, 7);
    init_servo(&servo8, self, 8);
    init_servo(&servo9, self, 9);
    init_servo(&servo10, self, 10);
    init_servo(&servo11, self, 11);
    init_servo(&servo12, self, 12);
    init_servo(&servo13, self, 13);
    init_servo(&servo14, self, 14);
    init_servo(&servo15, self, 15);
}

void close_servo_driver_i2c(_SERVODRIVER *self) {
    /*
    Closes I2C communication object for the specified servo driver
    */
    i2c_close(self -> bus);
}

void init_servo(_SERVO *self, _SERVODRIVER *sd, uint8_t number) {
    /*
    Initializes a servo object connected to an I2C servo driver. Requires an
    object representing the controlling I2C servo driver and the servo's
    numerical address (0-16, incl.) on its controlling I2C servo driver.
    */
    self -> driver = sd;
    self -> num = number;
    sd -> servos[number] = self;
}

void servo_driver_configure(_SERVODRIVER *self, float pwm_freq) {
    /*
    Sets the PWM frequency (bounds 24Hz - 1526Hz). This involves sleeping and
    restarting the servo driver. Also sets MODE1 and MODE2 registers to
    configured (see init_servo_driver()) values.
    */

    // pwm_freq *= 0.90;  // Not needed: [Correct for overshoot in the frequency setting]
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= pwm_freq;
    prescaleval -= 1;
    uint8_t prescale = floor(prescaleval + 0.5);

    uint8_t address = self->i2c_addr << 1;  // 0b10000000
    // disable_interrupts();
    i2c_start(self->bus);                   // Start 
    i2c_putc(self->bus, address);           // Slave address 
    i2c_putc(self->bus, PCA9685_MODE1);     // Mode 1 address 
    i2c_putc(self->bus, 0b00110001);        // Setting mode to sleep so we can change the default PWM frequency 
    i2c_stop(self->bus);                    // Stop 
    __delay_ms(delay_ms_time);                          // Required 50 us delay 
    i2c_start(self->bus);                   // Start 
    i2c_putc(self->bus, address);           // Slave address 
    i2c_putc(self->bus, PCA9685_PRESCALE);  // PWM frequency PRE_SCALE address 
    i2c_putc(self->bus, prescale);          // osc_clk/(4096*update_rate) // 25000000/(4096*40)= 4.069 ~4 
    i2c_stop(self->bus);                    // Stop 
    __delay_ms(delay_ms_time);                          // delay at least 500 us 
    i2c_start(self->bus);                   // Start 
    i2c_putc(self->bus, address);           // Slave address 
    i2c_putc(self->bus, PCA9685_MODE1);     // Mode 1 register address 
    i2c_putc(self->bus, self->mode1);       // Set to our prefered mode1 
    i2c_stop(self->bus);                    // Stop 
    __delay_ms(delay_ms_time);                          // delay at least 500 us 
    i2c_start(self->bus);                   // Start 
    i2c_putc(self->bus, address);           // Slave Address 
    i2c_putc(self->bus, PCA9685_MODE2);     // Mode2 register address 
    i2c_putc(self->bus, 0b00000100);        // Set to our prefered mode2 
    i2c_stop(self->bus);
    // enable_interrupts();

}

void servo_set_pwm(_SERVO *self, uint16_t on, uint16_t off) {
    /*
    Sets a PWM signal as specified by the datasheet by writing 16-bit on and
    "off" values into a series of four registers. This requires that auto-
    increment be enabled in the MODE1 register.
    */
    // disable_interrupts();
    servo_driver_begin_transmission(self -> driver, I2C_WRITE);
    i2c_putc(self -> driver -> bus, DEV0_ON_L + 4 * self -> num);
    i2c_putc(self -> driver -> bus, on);
    i2c_putc(self -> driver -> bus, on >> 8);
    i2c_putc(self -> driver -> bus, off);
    i2c_putc(self -> driver -> bus, off >> 8);
    servo_driver_end_transmission(self -> driver);
    // enable_interrupts();
}

void servo_set(_SERVO *self, uint16_t val, bool invert) {
    /*
    Sets pin without having to deal with on/off tick placement and properly
    handles a zero value as completely off.  Optional invert parameter supports
    inverting the pulse for sinking to ground.
    Val should be a value from 0 to 4095 inclusive.
    */
    // Clamp value between 0 and 4095 inclusive.
    // led_on(&led2);
    if ( val > 4095) {
        val = 4095;
    }
    if (invert) {
        if (val == 0) {
            // Special value for signal fully on.
            servo_set_pwm(self, 4096, 0);
        }
        else if (val == 4095) {
            // Special value for signal fully off.
            servo_set_pwm(self, 0, 4096);
        }
        else {
            servo_set_pwm(self, 0, 4095 - val);
        }
    }
    else {
        if (val == 4095) {
            // Special value for signal fully on.
            servo_set_pwm(self, 4096, 0);
        }
        else if (val == 0) {
            // Special value for signal fully off.
            servo_set_pwm(self, 0, 4096);
        }
        else {
            servo_set_pwm(self, 0, val);
        }
        // led_off(&led2);
    }
}

void servo_usb_set(_SERVODRIVER *self, uint8_t index, uint16_t val) {
    /*
    Sets a servo to val based on its index on the specified servo driver
    */
    servo_set(self->servos[index], val, 0);
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

void servo_driver_end_transmission(_SERVODRIVER *dest) {
    i2c_stop(dest -> bus);
}