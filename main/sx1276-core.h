/*
 * sx1276-core.h
 *
 *  Created on: Oct 18, 2023
 *      Author: dev
 */

#ifndef MAIN_SX1276_CORE_H_
#define MAIN_SX1276_CORE_H_

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>

typedef void(SX1276OnDataHandler)(uint8_t *data, uint8_t len);

void sx1276_init(SX1276OnDataHandler *onDataHandler);

#endif /* MAIN_SX1276_CORE_H_ */
