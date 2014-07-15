#include "pca9547.h"

pca9547::pca9547() {
}

/* FUNCTION: probe_mux()
 * Probes mux for data, returns an integer corresponding to current selected mux. Return -1 if invalid.
 * RETURN: Integer identifier of current mux selection.
 */
int pca9547::probe_mux() {
	uint8_t buffer[1];
	int temp;
	if( BBBI2C::readByte(PCA9547_BBB_BUS, PCA9547_DEV_ADDR, PCA9547_CURRENT_REG, buffer ) < 0)
		return -1; // Return -1 if no byte is read

	temp = (int)buffer[0] - 8;	// Addressing starts at 0x08 = muxed bus 0
	return temp;
}


int pca9547::select_chan(int chan) {
	chan += 8;
	if ( !BBBI2C::writeByte( PCA9547_BBB_BUS, PCA9547_DEV_ADDR, PCA9547_CONTROL_REG, (uint8_t)chan ) )
		return -1;

	return chan;
}

int pca9547::deselect_mux() {
	return select_chan(PCA9547_DESL);
}