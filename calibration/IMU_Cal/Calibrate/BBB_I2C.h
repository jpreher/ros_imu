/*=====================================================================================================
// I2C Tools for the BeagleBone Black - HEADER
//=====================================================================================================
//
// Set of tools for to utilize the I2C interface of BBB.
//
// Date			Author			Notes
// 02/10/2014 	Jake Reher		Initial Release
//
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#ifndef BBB_I2C_H
#define BBB_I2C_H

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stropts.h>
#include <stdio.h>
#include <iostream>
#include <math.h>


class BBBI2C {
public:
    BBBI2C();

    static int8_t readBit(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t bitNum, uint8_t *data);
    static int8_t readBits(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t bitStart, uint8_t length, uint8_t *data);
    static int8_t readByte(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t *data);
    static int8_t readBytes(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t length, uint8_t *data);

    static bool writeBit(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t bitNum, uint8_t data);
    static bool writeBits(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t bitStart, uint8_t length, uint8_t data);
    static bool writeByte(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t data);
};


#endif // BBB_I2C_H
