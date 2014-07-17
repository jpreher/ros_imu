#ifndef PCA9547_H
#define PCA9547_H

#include "BBB_I2C.h"

#define PCA9547_BBB_BUS		1
#define PCA9547_DEV_ADDR 	0x70
#define PCA9547_CONTROL_REG 0x04
#define PCA9547_CURRENT_REG	0x00
#define PCA9547_DESL		0x00

class pca9547 {
public:
	pca9547();

	static int probe_mux();
	static int select_chan(int chan);
	static int deselect_mux();

    static inline uint8_t readBit(uint8_t bus, uint8_t chan, uint8_t devAddress, uint8_t regAddress, uint8_t bitNum, uint8_t *data) {
        select_chan(chan);
        return BBBI2C::readBit(bus, devAddress, regAddress, bitNum, data);
    }

    static inline int8_t readBits(uint8_t bus, uint8_t chan, uint8_t devAddress, uint8_t regAddress, uint8_t bitStart, uint8_t length, uint8_t *data) {
        select_chan(chan);
        return BBBI2C::readBits(bus, devAddress, regAddress, bitStart, length, data);
    }

    static inline int8_t readByte(uint8_t bus, uint8_t chan, uint8_t devAddress, uint8_t regAddress, uint8_t *data) {
        select_chan(chan);
        return BBBI2C::readByte(bus, devAddress, regAddress, data);
    }

    static inline int8_t readBytes(uint8_t bus, uint8_t chan, uint8_t devAddress, uint8_t regAddress, uint8_t length, uint8_t *data) {
        select_chan(chan);
        return BBBI2C::readBytes(bus, devAddress, regAddress, length, data);
    }

    static inline bool writeBit(uint8_t bus, uint8_t chan, uint8_t devAddress, uint8_t regAddress, uint8_t bitNum, uint8_t data) {
        select_chan(chan);
        return BBBI2C::writeBit(bus, devAddress, regAddress, bitNum, data);
    }

    static inline bool writeBits(uint8_t bus, uint8_t chan, uint8_t devAddress, uint8_t regAddress, uint8_t bitStart, uint8_t length, uint8_t data) {
        select_chan(chan);
        return BBBI2C::writeBits(bus, devAddress, regAddress, bitStart, length, data);
    }

    static inline bool writeByte(uint8_t bus, uint8_t chan, uint8_t devAddress, uint8_t regAddress, uint8_t data) {
        select_chan(chan);
        return BBBI2C::writeByte(bus, devAddress, regAddress, data);
    }



};
#endif // PCA9547_H
