/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Matthieu Verdy and Benjamin Boulet
*/
//#include "hw.h"
#include "sx1276-hal.h"
#include "sx1276-spi.h"
#include "radio.h"
#include "hw-gpio.h"
#include <string.h>

/*!
 * \brief Define the size of tx and rx hal buffers
 *
 * The Tx and Rx hal buffers are used for SPI communication to
 * store data to be sent/receive to/from the chip.
 *
 * \warning The application must ensure the maximal useful size to be much lower
 *          than the MAX_HAL_BUFFER_SIZE
 */
#define MAX_HAL_BUFFER_SIZE   0xFFF

#define IRQ_HIGH_PRIORITY  0

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
		SX1276Init,
		SX1276HalReset,
		SX1276GetStatus,
	    SX1276SetModem,
	    SX1276SetChannel,
	    SX1276IsChannelFree,
	    SX1276Random,
	    SX1276SetRxConfig,
	    SX1276SetTxConfig,
	    SX1276CheckRfFrequency,
	    SX1276GetTimeOnAir,
	    SX1276Send,
	    SX1276SetSleep,
		SX1276SetStby,
	    SX1276SetRx,
	    SX1276StartCad,
	    SX1276SetTxContinuousWave,
		SX1276ReadRssi,
	    SX1276HalWriteRegister,
	    SX1276HalReadRegister,
	    SX1276HalWriteBuffer,
	    SX1276HalReadBuffer,
	    SX1276SetMaxPayloadLength,
	    SX1276SetPublicNetwork,
	    SX1276GetWakeupTime,
	    NULL,
		NULL,
		NULL,
};

void SX1276HalInit( DioIrqHandler **irqHandlers )
{
    SX1276HalReset( );
    SX1276HalIoIrqInit(irqHandlers);
}

void SX1276HalIoIrqInit( DioIrqHandler **irqHandlers )
{
#if( RADIO_DIO0_ENABLE )
    gpio_set_irq( RADIO_DIO0_Port, RADIO_DIO0_Pin, 5, irqHandlers[0], NULL);
#endif
#if( RADIO_DIO1_ENABLE )
    gpio_set_irq( RADIO_DIO1_Port, RADIO_DIO1_Pin, 5, irqHandlers[1], NULL);
#endif
#if( RADIO_DIO2_ENABLE )
    gpio_set_irq( RADIO_DIO2_Port, RADIO_DIO2_Pin, 5, irqHandlers[2], NULL);
#endif
#if( RADIO_DIO3_ENABLE )
    gpio_set_irq( RADIO_DIO3_Port, RADIO_DIO3_Pin, 5, irqHandlers[3], NULL);
#endif
#if( RADIO_DIO4_ENABLE )
    gpio_set_irq( RADIO_DIO4_Port, RADIO_DIO4_Pin, 5, irqHandlers[4], NULL);
#endif
#if(!RADIO_DIO0_ENABLE  && !RADIO_DIO1_ENABLE && !RADIO_DIO2_ENABLE && !RADIO_DIO3_ENABLE && !RADIO_DIO4_ENABLE)
#error "Please define a DIO" 
#endif
}

void SX1276HalReset( void )
{
    gpio_reset(RADIO_nRESET_PORT, RADIO_nRESET_PIN);
    HAL_Delay( 1 );
    gpio_set( RADIO_nRESET_PORT, RADIO_nRESET_PIN);
    HAL_Delay( 6 );
}


void SX1276HalWriteRegister( uint8_t address, uint8_t value )
{
    SX1276HalWriteBuffer( address, &value, 1 );
}

uint8_t SX1276HalReadRegister( uint8_t address )
{
    uint8_t value;
    SX1276HalReadBuffer( address, &value, 1 );
    return value;
}


void SX1276HalWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    //uint8_t i;
    //NSS = 0;
    gpio_reset( RADIO_NSS_PORT, RADIO_NSS_PIN);
    Sx1276SpiInOut(offset | 0x80 );
    Sx1276SpiTransmit(buffer, size);
//    for( i = 0; i < size; i++ )
//    {
//    	Sx1276SpiInOut(buffer[i]);
//    }
    //NSS = 1;
    gpio_set( RADIO_NSS_PORT, RADIO_NSS_PIN);
}


void SX1276HalReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    //uint8_t i;
    //NSS = 0;
    gpio_reset( RADIO_NSS_PORT, RADIO_NSS_PIN);

    Sx1276SpiInOut(offset & 0x7F);

    Sx1276SpiReceive(buffer, size);
//    for( i = 0; i < size; i++ )
//    {
//        buffer[i] = Sx1276SpiInOut(0);
//    }

    //NSS = 1;
    gpio_set( RADIO_NSS_PORT, RADIO_NSS_PIN);
}


uint8_t SX1276HalGetDioStatus( void )
{
	uint8_t Status = 0;
#if( RADIO_DIO0_ENABLE )
	Status |= (gpio_read( RADIO_DIO0_Port, RADIO_DIO0_Pin ) << 1);
#endif
#if( RADIO_DIO1_ENABLE )
	Status |= (gpio_read( RADIO_DIO1_Port, RADIO_DIO1_Pin ) << 1);
#endif
#if( RADIO_DIO2_ENABLE )
	Status |= (gpio_read( RADIO_DIO2_Port, RADIO_DIO2_Pin ) << 2);
#endif
#if( RADIO_DIO3_ENABLE )
	Status |= (gpio_read( RADIO_DIO3_Port, RADIO_DIO3_Pin ) << 3);
#endif
#if( RADIO_DIO4_ENABLE )
	Status |= (gpio_read( RADIO_DIO4_Port, RADIO_DIO4_Pin ) << 3);
#endif
	#if( !RADIO_DIO0_ENABLE && !RADIO_DIO1_ENABLE && !RADIO_DIO2_ENABLE && !RADIO_DIO3_ENABLE && !RADIO_DIO4_ENABLE)
#error "Please define a DIO" 
#endif
	
	return Status;
}
