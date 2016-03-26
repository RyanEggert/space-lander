#ifndef LIB_SERVO_H_
#define LIB_SERVO_H_

#define I2C_WRITE 0
#define I2C_READ 1

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD



// Servo Driver
typedef struct {
    _I2C bus;
    uint8_t hardware_addr
    uint8_t i2c_addr;
    float i2c_freq;
    uint8_t mode1; // Normal, awake mode of operation [MODE1 register]

} _SERVODRIVER;

// Servo Driver
typedef struct {
    _SERVODRIVER driver;
    uint8_t num

} _SERVO;


#endif  // LIB_SERVO_H_
