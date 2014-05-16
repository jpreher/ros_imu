/*=====================================================================================================
// I2C Tools for the BeagleBone Black
//=====================================================================================================
//
// Set of tools for to utilize the I2C interface of BBB.
//
// Date			Author			Notes
// 02/10/2014 	Jake Reher		Initial Release
//
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#include "BBB_I2C.h"

/* CONSTRUCTOR
 */
BBBI2C::BBBI2C() {
}

/* FUNCTION readBit(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t bitNum, uint8_t *data)
 * Read a single bit from an I2C device.
 * @PARAM bus - bus of the device to be read.
 * @PARAM devAddress - I2C device address.
 * @PARAM bitNum - bit index number to be read.
 * @PARAM data - container for single bit data.
 */
int8_t BBBI2C::readBit(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    uint8_t count = readByte(bus, devAddress, regAddress, &b);
    *data = b & (1 << bitNum);
    return count;
}

/* FUNCTION readBits(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t bitStart, uint8_t length, uint8_t *data)
 * Read multiple bits from an I2C device.
 * @PARAM bus - bus of the device to be read.
 * @PARAM devAddress - I2C device address.
 * @PARAM bitStart - bit index number to start read from.
 * @PARAM length - number of bits to be read.
 * @PARAM data - container for multiple bits of data.
 */
int8_t BBBI2C::readBits(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t bitStart, uint8_t length, uint8_t *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(bus, devAddress, regAddress, &b)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/* FUNCTION readByte(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t *data)
 * Read a single byte from an I2C device.
 * @PARAM bus - bus of the device to be read.
 * @PARAM devAddress - I2C device address.
 * @PARAM regAddress - Register address to read byte from.
 * @PARAM data - container for single byte data.
 */
int8_t BBBI2C::readByte(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t *data) {
    return readBytes(bus, devAddress, regAddress, 1, data);
}

/* FUNCTION readBytes(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t length, uint8_t *data)
 * Read multiple bytes from an I2C device.
 * @PARAM bus - bus of the device to be read.
 * @PARAM devAddress - I2C device address.
 * @PARAM regAddress - Register address to begin read from.
 * @PARAM length - number of bytes to be read.
 * @PARAM data - container for multiple bytes of data.
 */
int8_t BBBI2C::readBytes(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t length, uint8_t *data) {
    char namebuf[64];
    snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", bus);
    int file;
    if ((file = open(namebuf, O_RDWR)) < 0){
            std::cout << "Failed to open MPU6050 Sensor on " << namebuf << " I2C Bus" << std::endl;
            return(1);
    }
    if (ioctl(file, I2C_SLAVE, devAddress) < 0){
            std::cout << "I2C_SLAVE address " << devAddress << " failed..." << std::endl;
            return(2);
    }

    // You need to send the first address
    // in write mode and then a stop/start condition is issued. Data bytes are
    // transferred with automatic address increment.
    char buf[1] = { regAddress };
    if(write(file, buf, 1) !=1){
        std::cout << "Failed to Reset Address in readFullSensor() " << std::endl;
    }

    int numberBytes = length;
    int bytesRead = read(file, data, numberBytes);
    if (bytesRead == -1){
        std::cout << "Failure to read Byte Stream in readFullSensor()" << std::endl;
    }
    close(file);
    return bytesRead;
}

/* FUNCTION writeBit(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t bitNum, uint8_t data)
 * Write a single bit of data to an I2C device.
 * @PARAM bus - bus of the device to be read.
 * @PARAM devAddress - I2C device address.
 * @PARAM regAddress - Register address to begin write to.
 * @PARAM bitNum - Bit index to be written to.
 * @PARAM data - Bit to be written.
 */
bool BBBI2C::writeBit(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(bus, devAddress, regAddress, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(bus, devAddress, regAddress, b);
}

/* FUNCTION writeBits(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t bitStart, uint8_t length, uint8_t data)
 * Write a multiple bits of data to an I2C device.
 * @PARAM bus - bus of the device to be read.
 * @PARAM devAddress - I2C device address.
 * @PARAM regAddress - Register address to begin write to.
 * @PARAM bitNum - Bit index to begin write from.
 * @PARAM data - Bits to be written.
 */
bool BBBI2C::writeBits(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(bus, devAddress, regAddress, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(bus, devAddress, regAddress, b);
    } else {
        return false;
    }
}

/* FUNCTION writeByte(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t data)
 * Write a single byte of data to an I2C device.
 * @PARAM bus - bus of the device to be read.
 * @PARAM devAddress - I2C device address.
 * @PARAM regAddress - Register address to begin write to.
 * @PARAM data - Byte to be written.
 */
bool BBBI2C::writeByte(uint8_t bus, uint8_t devAddress, uint8_t regAddress, uint8_t data) {
    int file;
    char namebuf[64];
    snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", bus);

    //Opens the path to device
    if ((file = open(namebuf, O_RDWR)) < 0){
            printf("%s did not open at address %d.\n", namebuf, devAddress);
            exit(1);
    }

    //Joins to the device.
    if (ioctl(file, I2C_SLAVE, devAddress) < 0){
            printf("Can't join the I2C Bus at address %d.\n", devAddress);
            exit(1);
    }

    char buffer[2];
    buffer[0] = regAddress;
    buffer[1] = data;
    if (write(file, buffer, 2) != 2) {
        printf("Failed to write to I2C Device at address %d.\n", devAddress);
        exit(1);
    }

    return true;
}





