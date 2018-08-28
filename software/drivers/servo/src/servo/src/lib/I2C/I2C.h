#ifndef SERVO_I2C_H
#define SERVO_I2C_H

#include <inttypes.h>

#define BUFFER_SIZE 0x01

class I2C {
public:
    I2C(int bus, int address);
    ~I2C();
    uint8_t dataBuffer[BUFFER_SIZE];
    uint8_t read_byte(uint8_t address);
    uint8_t write_byte(uint8_t address, uint8_t data);

private:
    int _i2caddr;
    int _i2cbus;
    void openfd();
    char busfile[64];
    int fd;
};

#endif //SERVO_I2C_H
