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
};
#endif // PCA9547_H