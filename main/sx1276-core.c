/*
 * sx1276-core.c
 *
 *  Created on: Oct 18, 2023
 *      Author: dev
 */

#include "sx1276-core.h"
#include "sx1276-spi.h"



SX1276OnDataHandler *sx1276_on_data = { NULL };

void sx1276_init(SX1276OnDataHandler *onDataHandler) {

	sx1276_on_data = onDataHandler;

//	sx1276_dev_t *dev = &sx1276_device;

	Sx1276SpiInit();
}

