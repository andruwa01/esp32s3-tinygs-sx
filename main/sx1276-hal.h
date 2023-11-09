/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Matthieu Verdy and Benjamin Boulet
*/
#ifndef __SX1276_HAL_H__
#define __SX1276_HAL_H__

#include "stdint.h"
#include "sx1276.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"


//todo

#define RADIO_DIO0_Port		 M_DIO0_GPIO_Port
#define RADIO_DIO0_Pin		 M_DIO0_Pin

#define RADIO_DIO1_Port		 M_DIO1_GPIO_Port
#define RADIO_DIO1_Pin		 M_DIO1_Pin

#define RADIO_DIO2_Port 	 M_DIO2_GPIO_Port
#define RADIO_DIO2_Pin		 M_DIO2_Pin

#define RADIO_DIO3_Port 	 M_DIO3_GPIO_Port
#define RADIO_DIO3_Pin		 M_DIO3_Pin

#define RADIO_DIO4_Port 	 M_DIO4_GPIO_Port
#define RADIO_DIO4_Pin		 M_DIO4_Pin


#define RADIO_nRESET_PORT	 M_NRESET_GPIO_Port
#define RADIO_nRESET_PIN	 M_NRESET_Pin

#define RADIO_NSS_PORT		 M_NSS_GPIO_Port
#define RADIO_NSS_PIN		 M_NSS_Pin

/*!
 * * \brief Define which DIOs are connected 
*/
#define RADIO_DIO0_ENABLE	1
#define RADIO_DIO1_ENABLE	1
#define RADIO_DIO2_ENABLE	1
#define RADIO_DIO3_ENABLE	0
#define RADIO_DIO4_ENABLE	0


void SX1276HalInit( DioIrqHandler **irqHandlers );
void SX1276HalIoInit( void );

/*!
 * \brief Soft resets the radio
 */
void SX1276HalReset( void );

/*!
 * \brief Write data to the radio memory
 *
 * \param [in]  address       The address of the first byte to write in the radio
 * \param [in]  buffer        The data to be written in radio's memory
 * \param [in]  size          The number of bytes to write in radio's memory
 */
//void SX1276HalWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size );

/*!
 * \brief Write a single byte of data to the radio memory
 *
 * \param [in]  address       The address of the first byte to write in the radio
 * \param [in]  value         The data to be written in radio's memory
 */
void SX1276HalWriteRegister( uint8_t address, uint8_t value );

/*!
 * \brief Read data from the radio memory
 *
 * \param [in]  address       The address of the first byte to read from the radio
 * \param [out] buffer        The buffer that holds data read from radio
 * \param [in]  size          The number of bytes to read from radio's memory
 */
//void SX1276HalReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size );

/*!
 * \brief Read a single byte of data from the radio memory
 *
 * \param [in]  address       The address of the first byte to write in the
     *                            radio
 *
 * \retval      value         The value of the byte at the given address in
     *                            radio's memory
 */
uint8_t SX1276HalReadRegister( uint8_t address );

/*!
 * \brief Write data to the buffer holding the payload in the radio
 *
 * \param [in]  offset        The offset to start writing the payload
 * \param [in]  buffer        The data to be written (the payload)
 * \param [in]  size          The number of byte to be written
 */
void SX1276HalWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size );


/*!
 * \brief Read data from the buffer holding the payload in the radio
 *
 * \param [in]  offset        The offset to start writing the payload
 * \param [out] buffer        A pointer to a buffer holding the data from the radio
 * \param [in]  size          The number of byte to be read
 */
void SX1276HalReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size );


/*!
 * \brief Returns the status of DIOs pins
 *
 * \retval      dioStatus     A byte where each bit represents a DIO state:
 *                            [ DIOx | BUSY ]
 */
uint8_t SX1276HalGetDioStatus( void );

void SX1276HalIoIrqInit( DioIrqHandler **irqHandlers );

#endif // __SX1276_HAL_H__
