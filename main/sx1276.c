/*!
 * \file      sx1276.c
 *
 * \brief     SX1276 driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Wael Guibene ( Semtech )
 */
#include <math.h>
#include <string.h>
#include "cmsis_os.h"
#include "radio.h"
#include "sx1276.h"
#include "sx1276-hal.h"
#include "sx1276-spi.h"


#include "hw-gpio.h"
#define RF_MID_BAND_THRESH                          525000000


/*!
 * \brief Internal frequency of the radio
 */
#define SX1276_XTAL_FREQ                            32000000UL

/*!
 * \brief Scaling factor used to perform fixed-point operations
 */
#define SX1276_PLL_STEP_SHIFT_AMOUNT                ( 8 )

/*!
 * \brief PLL step - scaled with SX1276_PLL_STEP_SHIFT_AMOUNT
 */
#define SX1276_PLL_STEP_SCALED                      ( SX1276_XTAL_FREQ >> ( 19 - SX1276_PLL_STEP_SHIFT_AMOUNT ) )

/*!
 * \brief Radio buffer size
 */
#define RX_TX_BUFFER_SIZE                           256

/*
 * Local types definition
 */

/*!
 * Radio registers definition
 */
typedef struct
{
    RadioModems_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
}RadioRegisters_t;

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;



/*
 * Private functions prototypes
 */

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration( void );

/*!
 * \brief Sets the SX1276 in transmission mode for the given time
 * \param [IN] timeout Transmission timeout [ms] [0: continuous, others timeout]
 */
static void SX1276SetTx( uint32_t timeout );

/*!
 * \brief Writes the buffer contents to the SX1276 FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
static void SX1276WriteFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads the contents of the SX1276 FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
static void SX1276ReadFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the SX1276 operating mode
 *
 * \param [IN] opMode New operating mode
 */
static void SX1276SetOpMode( uint8_t opMode );

/*!
 * \brief Get frequency in Hertz for a given number of PLL steps
 *
 * \param [in] pllSteps Number of PLL steps
 *
 * \returns Frequency in Hertz
 */
static uint32_t SX1276ConvertPllStepToFreqInHz( uint32_t pllSteps );

/*!
 * \brief Get the number of PLL steps for a given frequency in Hertz
 *
 * \param [in] freqInHz Frequency in Hertz
 *
 * \returns Number of PLL steps
 */
static uint32_t SX1276ConvertFreqInHzToPllStep( uint32_t freqInHz );

/*!
 * \brief Get the parameter corresponding to a FSK Rx bandwith immediately above the minimum requested one.
 *
 * \param [in] bw Minimum required bandwith in Hz
 *
 * \returns parameter
 */
static uint8_t GetFskBandwidthRegValue( uint32_t bw );

/*!
 * \brief Get the actual value in Hertz of a given LoRa bandwidth
 *
 * \param [in] bw LoRa bandwidth parameter
 *
 * \returns Actual LoRa bandwidth in Hertz
 */
static uint32_t SX1276GetLoRaBandwidthInHz( uint32_t bw );

/*!
 * Compute the numerator for GFSK time-on-air computation.
 *
 * \remark To get the actual time-on-air in second, this value has to be divided by the GFSK bitrate in bits per
 * second.
 *
 * \param [in] preambleLen 
 * \param [in] fixLen 
 * \param [in] payloadLen 
 * \param [in] crcOn 
 *
 * \returns GFSK time-on-air numerator
 */
static uint32_t SX1276GetGfskTimeOnAirNumerator( uint16_t preambleLen, bool fixLen,
                                                 uint8_t payloadLen, bool crcOn );

/*!
 * Compute the numerator for LoRa time-on-air computation.
 *
 * \remark To get the actual time-on-air in second, this value has to be divided by the LoRa bandwidth in Hertz.
 *
 * \param [in] bandwidth 
 * \param [in] datarate 
 * \param [in] coderate 
 * \param [in] preambleLen 
 * \param [in] fixLen 
 * \param [in] payloadLen 
 * \param [in] crcOn 
 *
 * \returns LoRa time-on-air numerator
 */
static uint32_t SX1276GetLoRaTimeOnAirNumerator( uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn );

/*
 * SX1276 DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
static void SX1276OnDio0Irq();

/*!
 * \brief DIO 1 IRQ callback
 */
static void SX1276OnDio1Irq();

/*!
 * \brief DIO 2 IRQ callback
 */
static void SX1276OnDio2Irq();

/*!
 * \brief DIO 3 IRQ callback
 */
static void SX1276OnDio3Irq();

/*!
 * \brief DIO 4 IRQ callback
 */
static void SX1276OnDio4Irq();

/*!
 * \brief Tx & Rx timeout timer callback
 */
static void SX1276OnTimeoutIrq();

/*
 * Private global constants
 */

/*!
 * Radio hardware registers initialization
 *
 * \remark RADIO_INIT_REGISTERS_VALUE is defined in sx1276-board.h file
 */
//todo
const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{
    { 2600  , 0x17 },
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Bandwidth
};

/*
 * Private global variables
 */

/*!
 * Radio callbacks variable
 */
static RadioEvents_t *RadioEvents;

/*!
 * Reception buffer
 */
static uint8_t RxTxBuffer[RX_TX_BUFFER_SIZE];

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
//SX1276_t SX1276;
RadioSettings_t sx1276_settings;

/*!
 * Hardware DIO IRQ callback initialization
 */
DioIrqHandler *DioIrq[] = { SX1276OnDio0Irq, SX1276OnDio1Irq,
                            SX1276OnDio2Irq, SX1276OnDio3Irq,
                            SX1276OnDio4Irq, NULL };

/*!
 * Tx and Rx timers
 */

static osThreadId_t sx1276TimerTaskHandle;
static const osThreadAttr_t sx1276TimerTask_attributes = {
  .name = "sx1276TimerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

typedef struct {
	volatile bool IsStarted;
	volatile uint32_t StartTick;
	uint32_t TimeoutTick;
	void (*Callback)();
} timer_t;

timer_t TxTimeoutTimer = {false, 0, 0, SX1276OnTimeoutIrq};
timer_t RxTimeoutTimer = {false, 0, 0, SX1276OnTimeoutIrq};
timer_t RxTimeoutSyncWordTimer = {false, 0, 0, SX1276OnTimeoutIrq};


void SX1276TimerStart(timer_t *t, uint32_t timeout) {
	if (t == NULL)
		return;
	t->StartTick = HAL_GetTick();
	t->TimeoutTick = timeout;
	t->IsStarted = true;
}

void SX1276TimerStop(timer_t * t) {
	if (t == NULL)
		return;
	t->IsStarted = false;
	t->StartTick = 0;
	t->TimeoutTick = 0;
}

void SX1276TimerProcess(timer_t * t) {
	if (t->IsStarted && HAL_GetTick()-t->StartTick >= t->TimeoutTick)
	{
		t->IsStarted = false;
		if (t->Callback != NULL)
			t->Callback();
	}
}


void SX1276TimerTaskHandler(void *arg) {
	for (;;) {
		SX1276TimerProcess(&TxTimeoutTimer);
		SX1276TimerProcess(&RxTimeoutTimer);
		SX1276TimerProcess(&RxTimeoutSyncWordTimer);
		osDelay(10);
	}
	osThreadExit();
}



/*
 * Radio driver functions implementation
 */

void SX1276Init( RadioEvents_t *events )
{

    RadioEvents = events;
    SX1276HalInit( DioIrq );

    SX1276HalReset();

    RxChainCalibration( );

    SX1276SetOpMode( RF_OPMODE_SLEEP );


    for(int i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1276SetModem( RadioRegsInit[i].Modem );
        SX1276HalWriteRegister( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }

    SX1276SetModem( MODEM_FSK );

    sx1276_settings.State = RF_IDLE;

    sx1276TimerTaskHandle = osThreadNew(SX1276TimerTaskHandler, NULL, &sx1276TimerTask_attributes);
}

RadioState_t SX1276GetStatus( void )
{
    return sx1276_settings.State;
}

void SX1276SetChannel( uint32_t freq )
{
    uint32_t freqInPllSteps = SX1276ConvertFreqInHzToPllStep( freq );

    sx1276_settings.Channel = freq;

    SX1276HalWriteRegister( REG_FRFMSB, ( uint8_t )( ( freqInPllSteps >> 16 ) & 0xFF ) );
    SX1276HalWriteRegister( REG_FRFMID, ( uint8_t )( ( freqInPllSteps >> 8 ) & 0xFF ) );
    SX1276HalWriteRegister( REG_FRFLSB, ( uint8_t )( freqInPllSteps & 0xFF ) );
}

bool SX1276IsChannelFree( uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
    bool status = true;
    int16_t rssi = 0;
    uint32_t carrierSenseTime = 0;

    SX1276SetSleep( );

    SX1276SetModem( MODEM_FSK );

    SX1276SetChannel( freq );

    SX1276HalWriteRegister( REG_RXBW, GetFskBandwidthRegValue( rxBandwidth ) );
    SX1276HalWriteRegister( REG_AFCBW, GetFskBandwidthRegValue( rxBandwidth ) );

    SX1276SetOpMode( RF_OPMODE_RECEIVER );

    HAL_Delay( 1 );

    carrierSenseTime = HAL_GetTick( );
    // Perform carrier sense for maxCarrierSenseTime
    while( HAL_GetTick()-carrierSenseTime < maxCarrierSenseTime / portTICK_PERIOD_MS)

    {
        rssi = SX1276ReadRssi( MODEM_FSK );

        if( rssi > rssiThresh )
        {
            status = false;
            break;
        }
    }
    SX1276SetSleep( );
    return status;
}

uint32_t SX1276Random( void )
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    SX1276SetModem( MODEM_LORA );

    // Disable LoRa modem interrupts
    SX1276HalWriteRegister( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
												 RFLR_IRQFLAGS_RXDONE |
												 RFLR_IRQFLAGS_PAYLOADCRCERROR |
												 RFLR_IRQFLAGS_VALIDHEADER |
												 RFLR_IRQFLAGS_TXDONE |
												 RFLR_IRQFLAGS_CADDONE |
												 RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
												 RFLR_IRQFLAGS_CADDETECTED );

    // Set radio in continuous reception
    SX1276SetOpMode( RF_OPMODE_RECEIVER );

    for( i = 0; i < 32; i++ )
    {
        HAL_Delay( 1 );
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ( ( uint32_t )SX1276HalReadRegister( REG_LR_RSSIWIDEBAND ) & 0x01 ) << i;
    }

    SX1276SetSleep( );

    return rnd;
}

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration( void )
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = SX1276HalReadRegister( REG_PACONFIG );

    initialFreq = SX1276ConvertPllStepToFreqInHz( ( ( ( uint32_t )SX1276HalReadRegister( REG_FRFMSB ) << 16 ) |
                                                    ( ( uint32_t )SX1276HalReadRegister( REG_FRFMID ) << 8 ) |
                                                    ( ( uint32_t )SX1276HalReadRegister( REG_FRFLSB ) ) ) );

    // Cut the PA just in case, RFO output, power = -1 dBm
    SX1276HalWriteRegister( REG_PACONFIG, 0x00 );

    // Launch Rx chain calibration for LF band
    SX1276HalWriteRegister( REG_IMAGECAL, ( SX1276HalReadRegister( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX1276HalReadRegister( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Sets a Frequency in HF band
    SX1276SetChannel( 868000000 );

    // Launch Rx chain calibration for HF band
    SX1276HalWriteRegister( REG_IMAGECAL, ( SX1276HalReadRegister( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX1276HalReadRegister( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Restore context
    SX1276HalWriteRegister( REG_PACONFIG, regPaConfigInitVal );
    SX1276SetChannel( initialFreq );
}

void SX1276SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    SX1276SetModem( modem );

    SX1276SetStby( );

    switch( modem )
    {
    case MODEM_FSK:
        {
        	sx1276_settings.Fsk.Bandwidth = bandwidth;
        	sx1276_settings.Fsk.Datarate = datarate;
        	sx1276_settings.Fsk.BandwidthAfc = bandwidthAfc;
        	sx1276_settings.Fsk.FixLen = fixLen;
        	sx1276_settings.Fsk.PayloadLen = payloadLen;
        	sx1276_settings.Fsk.CrcOn = crcOn;
            sx1276_settings.Fsk.IqInverted = iqInverted;
            sx1276_settings.Fsk.RxContinuous = rxContinuous;
            sx1276_settings.Fsk.PreambleLen = preambleLen;
            sx1276_settings.Fsk.RxSingleTimeout = ( uint32_t )symbTimeout * 8000UL / datarate;

            uint32_t bitRate = ( uint32_t )( SX1276_XTAL_FREQ / datarate );
            SX1276HalWriteRegister( REG_BITRATEMSB, ( uint8_t )( bitRate >> 8 ) );
            SX1276HalWriteRegister( REG_BITRATELSB, ( uint8_t )( bitRate & 0xFF ) );

            SX1276HalWriteRegister( REG_RXBW, GetFskBandwidthRegValue( bandwidth ) );
            SX1276HalWriteRegister( REG_AFCBW, GetFskBandwidthRegValue( bandwidthAfc ) );

            SX1276HalWriteRegister( REG_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            SX1276HalWriteRegister( REG_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                SX1276HalWriteRegister( REG_PAYLOADLENGTH, payloadLen );
            }
            else
            {
                SX1276HalWriteRegister( REG_PAYLOADLENGTH, 0xFF ); // Set payload length to the maximum
            }

            SX1276HalWriteRegister( REG_PACKETCONFIG1,( SX1276HalReadRegister( REG_PACKETCONFIG1 ) &
                           	   	   	   	   	   	   	    RF_PACKETCONFIG1_CRC_MASK &
														RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           	   	   	   	   	   	   	   ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
													   	( crcOn << 4 ) );
            SX1276HalWriteRegister( REG_PACKETCONFIG2, ( SX1276HalReadRegister( REG_PACKETCONFIG2 ) | RF_PACKETCONFIG2_DATAMODE_PACKET ) );
        }
        break;
    case MODEM_LORA:
        {
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            sx1276_settings.LoRa.Bandwidth = bandwidth;
            sx1276_settings.LoRa.Datarate = datarate;
            sx1276_settings.LoRa.Coderate = coderate;
            sx1276_settings.LoRa.PreambleLen = preambleLen;
            sx1276_settings.LoRa.FixLen = fixLen;
            sx1276_settings.LoRa.PayloadLen = payloadLen;
            sx1276_settings.LoRa.CrcOn = crcOn;
            sx1276_settings.LoRa.FreqHopOn = freqHopOn;
            sx1276_settings.LoRa.HopPeriod = hopPeriod;
            sx1276_settings.LoRa.IqInverted = iqInverted;
            sx1276_settings.LoRa.RxContinuous = rxContinuous;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }

            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                sx1276_settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                sx1276_settings.LoRa.LowDatarateOptimize = 0x00;
            }

            SX1276HalWriteRegister( REG_LR_MODEMCONFIG1,
                         ( SX1276HalReadRegister( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

            SX1276HalWriteRegister( REG_LR_MODEMCONFIG2,
                         ( SX1276HalReadRegister( REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) |
                           ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

            SX1276HalWriteRegister( REG_LR_MODEMCONFIG3,
                         ( SX1276HalReadRegister( REG_LR_MODEMCONFIG3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( sx1276_settings.LoRa.LowDatarateOptimize << 3 ) );

            SX1276HalWriteRegister( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );

            SX1276HalWriteRegister( REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            SX1276HalWriteRegister( REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                SX1276HalWriteRegister( REG_LR_PAYLOADLENGTH, payloadLen );
            }

            if( sx1276_settings.LoRa.FreqHopOn == true )
            {
                SX1276HalWriteRegister( REG_LR_PLLHOP, ( SX1276HalReadRegister( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX1276HalWriteRegister( REG_LR_HOPPERIOD, sx1276_settings.LoRa.HopPeriod );
            }

            if( ( bandwidth == 9 ) && ( sx1276_settings.Channel > RF_MID_BAND_THRESH ) )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276HalWriteRegister( REG_LR_HIGHBWOPTIMIZE1, 0x02 );
                SX1276HalWriteRegister( REG_LR_HIGHBWOPTIMIZE2, 0x64 );
            }
            else if( bandwidth == 9 )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276HalWriteRegister( REG_LR_HIGHBWOPTIMIZE1, 0x02 );
                SX1276HalWriteRegister( REG_LR_HIGHBWOPTIMIZE2, 0x7F );
            }
            else
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276HalWriteRegister( REG_LR_HIGHBWOPTIMIZE1, 0x03 );
            }

            if( datarate == 6 )
            {
                SX1276HalWriteRegister( REG_LR_DETECTOPTIMIZE,
                             ( SX1276HalReadRegister( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX1276HalWriteRegister( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                SX1276HalWriteRegister( REG_LR_DETECTOPTIMIZE,
                             ( SX1276HalReadRegister( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX1276HalWriteRegister( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

void SX1276SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    SX1276SetModem( modem );

    SX1276SetStby( );

    SX1276SetRfTxPower( power );

    switch( modem )
    {
    case MODEM_FSK:
        {
            sx1276_settings.Fsk.Power = power;
            sx1276_settings.Fsk.Fdev = fdev;
            sx1276_settings.Fsk.Bandwidth = bandwidth;
            sx1276_settings.Fsk.Datarate = datarate;
            sx1276_settings.Fsk.PreambleLen = preambleLen;
            sx1276_settings.Fsk.FixLen = fixLen;
            sx1276_settings.Fsk.CrcOn = crcOn;
            sx1276_settings.Fsk.IqInverted = iqInverted;
            sx1276_settings.Fsk.TxTimeout = timeout;

            uint32_t fdevInPllSteps = SX1276ConvertFreqInHzToPllStep( fdev );
            SX1276HalWriteRegister( REG_FDEVMSB, ( uint8_t )( fdevInPllSteps >> 8 ) );
            SX1276HalWriteRegister( REG_FDEVLSB, ( uint8_t )( fdevInPllSteps & 0xFF ) );

            uint32_t bitRate = ( uint32_t )( SX1276_XTAL_FREQ / datarate );
            SX1276HalWriteRegister( REG_BITRATEMSB, ( uint8_t )( bitRate >> 8 ) );
            SX1276HalWriteRegister( REG_BITRATELSB, ( uint8_t )( bitRate & 0xFF ) );

            SX1276HalWriteRegister( REG_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX1276HalWriteRegister( REG_PREAMBLELSB, preambleLen & 0xFF );

            SX1276HalWriteRegister( REG_PACKETCONFIG1,
                         ( SX1276HalReadRegister( REG_PACKETCONFIG1 ) &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );
            SX1276HalWriteRegister( REG_PACKETCONFIG2, ( SX1276HalReadRegister( REG_PACKETCONFIG2 ) | RF_PACKETCONFIG2_DATAMODE_PACKET ) );
        }
        break;
    case MODEM_LORA:
        {
            sx1276_settings.LoRa.Power = power;
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            sx1276_settings.LoRa.Bandwidth = bandwidth;
            sx1276_settings.LoRa.Datarate = datarate;
            sx1276_settings.LoRa.Coderate = coderate;
            sx1276_settings.LoRa.PreambleLen = preambleLen;
            sx1276_settings.LoRa.FixLen = fixLen;
            sx1276_settings.LoRa.FreqHopOn = freqHopOn;
            sx1276_settings.LoRa.HopPeriod = hopPeriod;
            sx1276_settings.LoRa.CrcOn = crcOn;
            sx1276_settings.LoRa.IqInverted = iqInverted;
            sx1276_settings.LoRa.TxTimeout = timeout;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }
            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                sx1276_settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                sx1276_settings.LoRa.LowDatarateOptimize = 0x00;
            }

            if( sx1276_settings.LoRa.FreqHopOn == true )
            {
                SX1276HalWriteRegister( REG_LR_PLLHOP, ( SX1276HalReadRegister( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX1276HalWriteRegister( REG_LR_HOPPERIOD, sx1276_settings.LoRa.HopPeriod );
            }

            SX1276HalWriteRegister( REG_LR_MODEMCONFIG1,
                         ( SX1276HalReadRegister( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

            SX1276HalWriteRegister( REG_LR_MODEMCONFIG2,
                         ( SX1276HalReadRegister( REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) );

            SX1276HalWriteRegister( REG_LR_MODEMCONFIG3,
                         ( SX1276HalReadRegister( REG_LR_MODEMCONFIG3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( sx1276_settings.LoRa.LowDatarateOptimize << 3 ) );

            SX1276HalWriteRegister( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX1276HalWriteRegister( REG_LR_PREAMBLELSB, preambleLen & 0xFF );

            if( datarate == 6 )
            {
                SX1276HalWriteRegister( REG_LR_DETECTOPTIMIZE,
                             ( SX1276HalReadRegister( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX1276HalWriteRegister( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                SX1276HalWriteRegister( REG_LR_DETECTOPTIMIZE,
                             ( SX1276HalReadRegister( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX1276HalWriteRegister( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

uint32_t SX1276GetTimeOnAir( RadioModems_t modem, uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn )
{
    uint32_t numerator = 0;
    uint32_t denominator = 1;

    switch( modem )
    {
    case MODEM_FSK:
        {
            numerator   = 1000U * SX1276GetGfskTimeOnAirNumerator( preambleLen, fixLen, payloadLen, crcOn );
            denominator = datarate;
        }
        break;
    case MODEM_LORA:
        {
            numerator   = 1000U * SX1276GetLoRaTimeOnAirNumerator( bandwidth, datarate, coderate, preambleLen, fixLen,
                                                                   payloadLen, crcOn );
            denominator = SX1276GetLoRaBandwidthInHz( bandwidth );
        }
        break;
    }
    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

void SX1276Send( uint8_t *buffer, uint8_t size )
{
    uint32_t txTimeout = 0;

    switch( sx1276_settings.Modem )
    {
    case MODEM_FSK:
        {
            sx1276_settings.FskPacketHandler.NbBytes = 0;
            sx1276_settings.FskPacketHandler.Size = size;

            if( sx1276_settings.Fsk.FixLen == false )
            {
                SX1276WriteFifo( ( uint8_t* )&size, 1 );
            }
            else
            {
                SX1276HalWriteRegister( REG_PAYLOADLENGTH, size );
            }

            if( ( size > 0 ) && ( size <= 64 ) )
            {
                sx1276_settings.FskPacketHandler.ChunkSize = size;
            }
            else
            {
                memcpy( RxTxBuffer, buffer, size );
                sx1276_settings.FskPacketHandler.ChunkSize = 32;
            }

            // Write payload buffer
            SX1276WriteFifo( buffer, sx1276_settings.FskPacketHandler.ChunkSize );
            sx1276_settings.FskPacketHandler.NbBytes += sx1276_settings.FskPacketHandler.ChunkSize;
            txTimeout = sx1276_settings.Fsk.TxTimeout;
        }
        break;
    case MODEM_LORA:
        {
            if( sx1276_settings.LoRa.IqInverted == true )
            {
                SX1276HalWriteRegister( REG_LR_INVERTIQ, ( ( SX1276HalReadRegister( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
                SX1276HalWriteRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                SX1276HalWriteRegister( REG_LR_INVERTIQ, ( ( SX1276HalReadRegister( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX1276HalWriteRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            sx1276_settings.LoRaPacketHandler.Size = size;

            // Initializes the payload size
            SX1276HalWriteRegister( REG_LR_PAYLOADLENGTH, size );

            // Full buffer used for Tx
            SX1276HalWriteRegister( REG_LR_FIFOTXBASEADDR, 0 );
            SX1276HalWriteRegister( REG_LR_FIFOADDRPTR, 0 );

            // FIFO operations can not take place in Sleep mode
            if( ( SX1276HalReadRegister( REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
            {
                SX1276SetStby( );
                HAL_Delay( 1 );
            }
            // Write payload buffer
            SX1276WriteFifo( buffer, size );
            txTimeout = sx1276_settings.LoRa.TxTimeout;
        }
        break;
    }

    SX1276SetTx( txTimeout );
}

void SX1276SetSleep( void )
{
	SX1276TimerStop(&RxTimeoutTimer);
	SX1276TimerStop(&TxTimeoutTimer);
	SX1276TimerStop(&RxTimeoutSyncWordTimer);

    SX1276SetOpMode( RF_OPMODE_SLEEP );

    // Disable TCXO radio is in SLEEP mode
    SX1276SetBoardTcxo( false );

    sx1276_settings.State = RF_IDLE;
}

void SX1276SetStby( void )
{
	SX1276TimerStop(&RxTimeoutTimer);
	SX1276TimerStop(&TxTimeoutTimer);
	SX1276TimerStop(&RxTimeoutSyncWordTimer);

    SX1276SetOpMode( RF_OPMODE_STANDBY );
    sx1276_settings.State = RF_IDLE;
}

void SX1276SetRx( uint32_t timeout )
{
    bool rxContinuous = false;
    //osTimerStop(TxTimeoutTimer);
    SX1276TimerStop(&TxTimeoutTimer);

    switch( sx1276_settings.Modem )
    {
    case MODEM_FSK:
        {
            rxContinuous = sx1276_settings.Fsk.RxContinuous;

            // DIO0=PayloadReady
            // DIO1=FifoLevel
            // DIO2=SyncAddr
            // DIO3=FifoEmpty
            // DIO4=Preamble
            // DIO5=ModeReady
            SX1276HalWriteRegister( REG_DIOMAPPING1, ( SX1276HalReadRegister( REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK & RF_DIOMAPPING1_DIO1_MASK & RF_DIOMAPPING1_DIO2_MASK ) |
                                                     RF_DIOMAPPING1_DIO0_00 |
                                                     RF_DIOMAPPING1_DIO1_00 |
                                                     RF_DIOMAPPING1_DIO2_11 );

            SX1276HalWriteRegister( REG_DIOMAPPING2, ( SX1276HalReadRegister( REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK & RF_DIOMAPPING2_MAP_MASK ) |
            		                                  RF_DIOMAPPING2_DIO4_11 |
													  RF_DIOMAPPING2_MAP_PREAMBLEDETECT );

            sx1276_settings.FskPacketHandler.FifoThresh = SX1276HalReadRegister( REG_FIFOTHRESH ) & 0x3F;

            SX1276HalWriteRegister( REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT );

            sx1276_settings.FskPacketHandler.PreambleDetected = false;
            sx1276_settings.FskPacketHandler.SyncWordDetected = false;
            sx1276_settings.FskPacketHandler.NbBytes = 0;
            sx1276_settings.FskPacketHandler.Size = 0;
        }
        break;
    case MODEM_LORA:
        {
            if( sx1276_settings.LoRa.IqInverted == true )
            {
                SX1276HalWriteRegister( REG_LR_INVERTIQ, ( ( SX1276HalReadRegister( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
                SX1276HalWriteRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                SX1276HalWriteRegister( REG_LR_INVERTIQ, ( ( SX1276HalReadRegister( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX1276HalWriteRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
            if( sx1276_settings.LoRa.Bandwidth < 9 )
            {
                SX1276HalWriteRegister( REG_LR_DETECTOPTIMIZE, SX1276HalReadRegister( REG_LR_DETECTOPTIMIZE ) & 0x7F );
                SX1276HalWriteRegister( REG_LR_IFFREQ2, 0x00 );
                switch( sx1276_settings.LoRa.Bandwidth )
                {
                case 0: // 7.8 kHz
                    SX1276HalWriteRegister( REG_LR_IFFREQ1, 0x48 );
                    SX1276SetChannel(sx1276_settings.Channel + 7810 );
                    break;
                case 1: // 10.4 kHz
                    SX1276HalWriteRegister( REG_LR_IFFREQ1, 0x44 );
                    SX1276SetChannel(sx1276_settings.Channel + 10420 );
                    break;
                case 2: // 15.6 kHz
                    SX1276HalWriteRegister( REG_LR_IFFREQ1, 0x44 );
                    SX1276SetChannel(sx1276_settings.Channel + 15620 );
                    break;
                case 3: // 20.8 kHz
                    SX1276HalWriteRegister( REG_LR_IFFREQ1, 0x44 );
                    SX1276SetChannel(sx1276_settings.Channel + 20830 );
                    break;
                case 4: // 31.2 kHz
                    SX1276HalWriteRegister( REG_LR_IFFREQ1, 0x44 );
                    SX1276SetChannel(sx1276_settings.Channel + 31250 );
                    break;
                case 5: // 41.4 kHz
                    SX1276HalWriteRegister( REG_LR_IFFREQ1, 0x44 );
                    SX1276SetChannel(sx1276_settings.Channel + 41670 );
                    break;
                case 6: // 62.5 kHz
                    SX1276HalWriteRegister( REG_LR_IFFREQ1, 0x40 );
                    break;
                case 7: // 125 kHz
                    SX1276HalWriteRegister( REG_LR_IFFREQ1, 0x40 );
                    break;
                case 8: // 250 kHz
                    SX1276HalWriteRegister( REG_LR_IFFREQ1, 0x40 );
                    break;
                }
            }
            else
            {
                SX1276HalWriteRegister( REG_LR_DETECTOPTIMIZE, SX1276HalReadRegister( REG_LR_DETECTOPTIMIZE ) | 0x80 );
            }

            rxContinuous = sx1276_settings.LoRa.RxContinuous;

            if( sx1276_settings.LoRa.FreqHopOn == true )
            {
                SX1276HalWriteRegister( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  	  	  	 //RFLR_IRQFLAGS_RXDONE |
                                                  	  	  	 //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  	  	  	 RFLR_IRQFLAGS_VALIDHEADER |
															 RFLR_IRQFLAGS_TXDONE |
															 RFLR_IRQFLAGS_CADDONE |
															 //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
															 RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone, DIO2=FhssChangeChannel
                SX1276HalWriteRegister( REG_DIOMAPPING1, ( SX1276HalReadRegister( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX1276HalWriteRegister( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  	  	  	 //RFLR_IRQFLAGS_RXDONE |
                                                  	  	  	 //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  	  	  	 RFLR_IRQFLAGS_VALIDHEADER |
															 RFLR_IRQFLAGS_TXDONE |
															 RFLR_IRQFLAGS_CADDONE |
															 RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
															 RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone
                SX1276HalWriteRegister( REG_DIOMAPPING1, ( SX1276HalReadRegister( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
            }
            SX1276HalWriteRegister( REG_LR_FIFORXBASEADDR, 0 );
            SX1276HalWriteRegister( REG_LR_FIFOADDRPTR, 0 );
        }
        break;
    }

    sx1276_settings.State = RF_RX_RUNNING;
    if( timeout != 0 )
    {
        SX1276TimerStart(&RxTimeoutTimer, timeout / portTICK_PERIOD_MS );
    }

    if( sx1276_settings.Modem == MODEM_FSK )
    {
        SX1276SetOpMode( RF_OPMODE_RECEIVER );

        if( rxContinuous == false )
        {
            if (sx1276_settings.Fsk.RxSingleTimeout > 0)
            	SX1276TimerStart(&RxTimeoutSyncWordTimer, sx1276_settings.Fsk.RxSingleTimeout / portTICK_PERIOD_MS );
        }
    }
    else
    {
        if( rxContinuous == true )
        {
            SX1276SetOpMode( RFLR_OPMODE_RECEIVER );
        }
        else
        {
            SX1276SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
    }
}

static void SX1276SetTx( uint32_t timeout )
{
	//osTimerStop( RxTimeoutTimer );
	SX1276TimerStop( &RxTimeoutTimer );

    switch( sx1276_settings.Modem )
    {
    case MODEM_FSK:
        {
            // DIO0=PacketSent
            // DIO1=FifoLevel
            // DIO2=FifoFull
            // DIO3=FifoEmpty
            // DIO4=LowBat
            // DIO5=ModeReady
            SX1276HalWriteRegister( REG_DIOMAPPING1, ( SX1276HalReadRegister( REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK & RF_DIOMAPPING1_DIO1_MASK & RF_DIOMAPPING1_DIO2_MASK ) );
            SX1276HalWriteRegister( REG_DIOMAPPING2, ( SX1276HalReadRegister( REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK & RF_DIOMAPPING2_MAP_MASK ) );
            sx1276_settings.FskPacketHandler.FifoThresh = SX1276HalReadRegister( REG_FIFOTHRESH ) & 0x3F;
        }
        break;
    case MODEM_LORA:
        {
            if( sx1276_settings.LoRa.FreqHopOn == true )
            {
                SX1276HalWriteRegister( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
															 RFLR_IRQFLAGS_RXDONE |
															 RFLR_IRQFLAGS_PAYLOADCRCERROR |
															 RFLR_IRQFLAGS_VALIDHEADER |
															 //RFLR_IRQFLAGS_TXDONE |
															 RFLR_IRQFLAGS_CADDONE |
															 //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
															 RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone, DIO2=FhssChangeChannel
                SX1276HalWriteRegister( REG_DIOMAPPING1, ( SX1276HalReadRegister( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) |
                										RFLR_DIOMAPPING1_DIO0_01 |
														RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX1276HalWriteRegister( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  	  	  	 RFLR_IRQFLAGS_RXDONE |
															 RFLR_IRQFLAGS_PAYLOADCRCERROR |
															 RFLR_IRQFLAGS_VALIDHEADER |
															 //RFLR_IRQFLAGS_TXDONE |
															 RFLR_IRQFLAGS_CADDONE |
															 RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
															 RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone
                SX1276HalWriteRegister( REG_DIOMAPPING1, ( SX1276HalReadRegister( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
            }
        }
        break;
    }
    sx1276_settings.State = RF_TX_RUNNING;
    if (timeout > 0)
    	SX1276TimerStart( &TxTimeoutTimer, timeout  / portTICK_PERIOD_MS );
    SX1276SetOpMode( RF_OPMODE_TRANSMITTER );
}

void SX1276StartCad( void )
{
    switch( sx1276_settings.Modem )
    {
    case MODEM_FSK:
        {

        }
        break;
    case MODEM_LORA:
        {
            SX1276HalWriteRegister( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
														 RFLR_IRQFLAGS_RXDONE |
														 RFLR_IRQFLAGS_PAYLOADCRCERROR |
														 RFLR_IRQFLAGS_VALIDHEADER |
														 RFLR_IRQFLAGS_TXDONE |
														 //RFLR_IRQFLAGS_CADDONE |
														 RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
														 //RFLR_IRQFLAGS_CADDETECTED
														 );

            // DIO3=CADDone
            SX1276HalWriteRegister( REG_DIOMAPPING1, ( SX1276HalReadRegister( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO3_MASK ) | RFLR_DIOMAPPING1_DIO3_00 );

            sx1276_settings.State = RF_CAD;
            SX1276SetOpMode( RFLR_OPMODE_CAD );
        }
        break;
    default:
        break;
    }
}

void SX1276SetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time )
{
	if (time == 0)
		return;
	uint32_t timeout = ( uint32_t )time * 1000;

    SX1276SetChannel( freq );

    SX1276SetTxConfig( MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, timeout );

    SX1276HalWriteRegister( REG_PACKETCONFIG2, ( SX1276HalReadRegister( REG_PACKETCONFIG2 ) & RF_PACKETCONFIG2_DATAMODE_MASK ) );
    // Disable radio interrupts
    SX1276HalWriteRegister( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11 );
    SX1276HalWriteRegister( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10 );

    sx1276_settings.State = RF_TX_RUNNING;
    SX1276TimerStart( &TxTimeoutTimer , timeout  / portTICK_PERIOD_MS );
    SX1276SetOpMode( RF_OPMODE_TRANSMITTER );
}

int16_t SX1276ReadRssi( RadioModems_t modem )
{
    int16_t rssi = 0;

    switch( modem )
    {
    case MODEM_FSK:
        rssi = -( SX1276HalReadRegister( REG_RSSIVALUE ) >> 1 );
        break;
    case MODEM_LORA:
        if( sx1276_settings.Channel > RF_MID_BAND_THRESH )
        {
            rssi = RSSI_OFFSET_HF + SX1276HalReadRegister( REG_LR_RSSIVALUE );
        }
        else
        {
            rssi = RSSI_OFFSET_LF + SX1276HalReadRegister( REG_LR_RSSIVALUE );
        }
        break;
    default:
        rssi = -1;
        break;
    }
    return rssi;
}

static void SX1276SetOpMode( uint8_t opMode )
{
#if defined( USE_RADIO_DEBUG )
    switch( opMode )
    {
        case RF_OPMODE_TRANSMITTER:
            SX1276DbgPinTxWrite( 1 );
            SX1276DbgPinRxWrite( 0 );
            break;
        case RF_OPMODE_RECEIVER:
        case RFLR_OPMODE_RECEIVER_SINGLE:
            SX1276DbgPinTxWrite( 0 );
            SX1276DbgPinRxWrite( 1 );
            break;
        default:
            SX1276DbgPinTxWrite( 0 );
            SX1276DbgPinRxWrite( 0 );
            break;
    }
#endif
    if( opMode == RF_OPMODE_SLEEP )
    {
        SX1276SetAntSwLowPower( true );
    }
    else
    {
        // Enable TCXO if operating mode different from SLEEP.
        SX1276SetBoardTcxo( true );
        SX1276SetAntSwLowPower( false );
        SX1276SetAntSw( opMode );
    }
    SX1276HalWriteRegister( REG_OPMODE, ( SX1276HalReadRegister( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
}

void SX1276SetModem( RadioModems_t modem )
{
    if( ( SX1276HalReadRegister( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_ON ) != 0 )
    {
        sx1276_settings.Modem = MODEM_LORA;
    }
    else
    {
        sx1276_settings.Modem = MODEM_FSK;
    }

    if( sx1276_settings.Modem == modem )
    {
        return;
    }

    sx1276_settings.Modem = modem;
    switch( sx1276_settings.Modem )
    {
    	default:
    	case MODEM_FSK:
    		SX1276SetOpMode( RF_OPMODE_SLEEP );
    		SX1276HalWriteRegister( REG_OPMODE, ( SX1276HalReadRegister( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );

    		SX1276HalWriteRegister( REG_DIOMAPPING1, 0x00 );
    		SX1276HalWriteRegister( REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady
    	break;
    	case MODEM_LORA:
    		SX1276SetOpMode( RF_OPMODE_SLEEP );
    		SX1276HalWriteRegister( REG_OPMODE, ( SX1276HalReadRegister( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );

    		SX1276HalWriteRegister( REG_DIOMAPPING1, 0x00 );
    		SX1276HalWriteRegister( REG_DIOMAPPING2, 0x00 );
        break;
    }
}


static void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276HalWriteBuffer( 0, buffer, size );
}

static void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276HalReadBuffer( 0, buffer, size );
}

void SX1276SetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    SX1276SetModem( modem );

    switch( modem )
    {
    case MODEM_FSK:
        if( sx1276_settings.Fsk.FixLen == false )
        {
            SX1276HalWriteRegister( REG_PAYLOADLENGTH, max );
        }
        break;
    case MODEM_LORA:
        SX1276HalWriteRegister( REG_LR_PAYLOADMAXLENGTH, max );
        break;
    }
}

void SX1276SetPublicNetwork( bool enable )
{
    SX1276SetModem( MODEM_LORA );
    sx1276_settings.LoRa.PublicNetwork = enable;
    if( enable == true )
    {
        // Change LoRa modem SyncWord
        SX1276HalWriteRegister( REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD );
    }
    else
    {
        // Change LoRa modem SyncWord
        SX1276HalWriteRegister( REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD );
    }
}

uint32_t SX1276GetWakeupTime( void )
{
    return SX1276GetBoardTcxoWakeupTime( ) + RADIO_WAKEUP_TIME;
}

static uint32_t SX1276ConvertPllStepToFreqInHz( uint32_t pllSteps )
{
    uint32_t freqInHzInt;
    uint32_t freqInHzFrac;
    
    // freqInHz = pllSteps * ( SX1276_XTAL_FREQ / 2^19 )
    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    freqInHzInt = pllSteps >> SX1276_PLL_STEP_SHIFT_AMOUNT;
    freqInHzFrac = pllSteps - ( freqInHzInt << SX1276_PLL_STEP_SHIFT_AMOUNT );
    
    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return freqInHzInt * SX1276_PLL_STEP_SCALED + 
           ( ( freqInHzFrac * SX1276_PLL_STEP_SCALED + ( 128 ) ) >> SX1276_PLL_STEP_SHIFT_AMOUNT );
}

static uint32_t SX1276ConvertFreqInHzToPllStep( uint32_t freqInHz )
{
    uint32_t stepsInt;
    uint32_t stepsFrac;

    // pllSteps = freqInHz / (SX1276_XTAL_FREQ / 2^19 )
    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    stepsInt = freqInHz / SX1276_PLL_STEP_SCALED;
    stepsFrac = freqInHz - ( stepsInt * SX1276_PLL_STEP_SCALED );
    
    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return ( stepsInt << SX1276_PLL_STEP_SHIFT_AMOUNT ) + 
           ( ( ( stepsFrac << SX1276_PLL_STEP_SHIFT_AMOUNT ) + ( SX1276_PLL_STEP_SCALED >> 1 ) ) /
             SX1276_PLL_STEP_SCALED );
}

static uint8_t GetFskBandwidthRegValue( uint32_t bw )
{
    uint8_t i;

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bw >= FskBandwidths[i].bandwidth ) && ( bw < FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 );
}

static uint32_t SX1276GetLoRaBandwidthInHz( uint32_t bw )
{
    uint32_t bandwidthInHz = 0;

    switch( bw )
    {
    case 0: // 125 kHz
        bandwidthInHz = 125000UL;
        break;
    case 1: // 250 kHz
        bandwidthInHz = 250000UL;
        break;
    case 2: // 500 kHz
        bandwidthInHz = 500000UL;
        break;
    }

    return bandwidthInHz;
}

static uint32_t SX1276GetGfskTimeOnAirNumerator( uint16_t preambleLen, bool fixLen,
                                                 uint8_t payloadLen, bool crcOn )
{
    const uint8_t syncWordLength = 3;

    return ( preambleLen << 3 ) +
           ( ( fixLen == false ) ? 8 : 0 ) +
             ( syncWordLength << 3 ) +
             ( ( payloadLen +
               ( 0 ) + // Address filter size
               ( ( crcOn == true ) ? 2 : 0 ) 
               ) << 3 
             );
}

static uint32_t SX1276GetLoRaTimeOnAirNumerator( uint32_t bandwidth, uint32_t datarate, uint8_t coderate,
                              	  	  	  	  	 uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
												 bool crcOn )
{
    int32_t crDenom = coderate + 4;
    bool lowDatareOptimize = false;

    // Ensure that the preamble length is at least 12 symbols when using SF5 or
    // SF6
    if( ( datarate == 5 ) || ( datarate == 6 ) )
    {
        if( preambleLen < 12 )
        {
            preambleLen = 12;
        }
    }

    if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
        ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
    {
        lowDatareOptimize = true;
    }

    int32_t ceilDenominator;
    int32_t ceilNumerator = ( payloadLen << 3 ) +
                            ( crcOn ? 16 : 0 ) -
                            ( 4 * datarate ) +
                            ( fixLen ? 0 : 20 );

    if( datarate <= 6 )
    {
        ceilDenominator = 4 * datarate;
    }
    else
    {
        ceilNumerator += 8;

        if( lowDatareOptimize == true )
        {
            ceilDenominator = 4 * ( datarate - 2 );
        }
        else
        {
            ceilDenominator = 4 * datarate;
        }
    }

    if( ceilNumerator < 0 )
    {
        ceilNumerator = 0;
    }

    // Perform integral ceil()
    int32_t intermediate =
        ( ( ceilNumerator + ceilDenominator - 1 ) / ceilDenominator ) * crDenom + preambleLen + 12;

    if( datarate <= 6 )
    {
        intermediate += 2;
    }

    return ( uint32_t )( ( 4 * intermediate + 1 ) * ( 1 << ( datarate - 2 ) ) );
}

RadioModems_t GetModemType(void){ //xx
  return sx1276_settings.Modem;
}

bool GetModemFixLen(void){//xx
  if(sx1276_settings.Modem == MODEM_LORA){
     return  sx1276_settings.LoRa.FixLen;
  }
  else{
     return  sx1276_settings.Fsk.FixLen;
  }
}

void SX1276StopRX_RUNNING(void){ //xx
  if( sx1276_settings.State == RF_RX_RUNNING){
    if( sx1276_settings.Modem == MODEM_FSK )
        {
            sx1276_settings.FskPacketHandler.PreambleDetected = false;
            sx1276_settings.FskPacketHandler.SyncWordDetected = false;
            sx1276_settings.FskPacketHandler.NbBytes = 0;
            sx1276_settings.FskPacketHandler.Size = 0;

            // Clear Irqs
            SX1276HalWriteRegister( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                        RF_IRQFLAGS1_PREAMBLEDETECT |
                                        RF_IRQFLAGS1_SYNCADDRESSMATCH );
            SX1276HalWriteRegister( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

            if( sx1276_settings.Fsk.RxContinuous == true )
            {
                // Continuous mode restart Rx chain
                SX1276HalWriteRegister( REG_RXCONFIG, SX1276HalReadRegister( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
            }
            else
            {
                //SX1276SetOpMode( RF_OPMODE_SLEEP );
                SX1276SetOpMode( RF_OPMODE_STANDBY );
            	sx1276_settings.State = RF_IDLE;
                SX1276TimerStop( &RxTimeoutSyncWordTimer );
            }
        }
    }
}

static void SX1276OnTimeoutIrq( void* context )
{
    switch( sx1276_settings.State )
    {
    case RF_RX_RUNNING:
        if( sx1276_settings.Modem == MODEM_FSK )
        {
            sx1276_settings.FskPacketHandler.PreambleDetected = false;
            sx1276_settings.FskPacketHandler.SyncWordDetected = false;
            sx1276_settings.FskPacketHandler.NbBytes = 0;
            sx1276_settings.FskPacketHandler.Size = 0;

            // Clear Irqs
            SX1276HalWriteRegister( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                        RF_IRQFLAGS1_PREAMBLEDETECT |
                                        RF_IRQFLAGS1_SYNCADDRESSMATCH );
            SX1276HalWriteRegister( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

            if( sx1276_settings.Fsk.RxContinuous == true )
            {
                // Continuous mode restart Rx chain
                SX1276HalWriteRegister( REG_RXCONFIG, SX1276HalReadRegister( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
            }
            else
            {
                //SX1276SetOpMode( RF_OPMODE_SLEEP );
                SX1276SetOpMode( RF_OPMODE_STANDBY );
            	sx1276_settings.State = RF_IDLE;
                SX1276TimerStop( &RxTimeoutSyncWordTimer );
            }
        }
//        else if (sx1276_settings.Modem == MODEM_LORA) {
//            SX1276SetOpMode( RF_OPMODE_STANDBY );
//        	sx1276_settings.State = RF_IDLE;
//
//        }

        if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
        {
            RadioEvents->RxTimeout( );
        }
        break;
    case RF_TX_RUNNING:
        // Tx timeout shouldn't happen.
        // Reported issue of SPI data corruption resulting in TX TIMEOUT 
        // is NOT related to a bug in radio transceiver.
        // It is mainly caused by improper PCB routing of SPI lines and/or
        // violation of SPI specifications.
        // To mitigate redesign, Semtech offers a workaround which resets
        // the radio transceiver and putting it into a known state.

        // BEGIN WORKAROUND
//xx
        /*// Reset the radio
        SX1276HalReset();

        // Calibrate Rx chain
        RxChainCalibration( );

        // Initialize radio default values
        SX1276SetOpMode( RF_OPMODE_SLEEP );

        for( uint8_t i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
        {
            SX1276SetModem( RadioRegsInit[i].Modem );
            SX1276HalWriteRegister( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
        }

        SX1276SetModem( MODEM_FSK );

        // Restore previous network type setting.
        SX1276SetPublicNetwork( sx1276_settings.LoRa.PublicNetwork );*/

        // END WORKAROUND
    	SX1276SetOpMode( RF_OPMODE_SLEEP );
        sx1276_settings.State = RF_IDLE;
        if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
        {
            RadioEvents->TxTimeout( );
        }
        break;
    default:
        break;
    }
}

static void SX1276OnDio0Irq( )
{
    volatile uint8_t irqFlags = 0;

    switch( sx1276_settings.State )
    {
        case RF_RX_RUNNING:
            //TimerStop( &RxTimeoutTimer );
            // RxDone interrupt
            switch( sx1276_settings.Modem )
            {
            case MODEM_FSK:
                if( sx1276_settings.Fsk.CrcOn == true )
                {
                    irqFlags = SX1276HalReadRegister( REG_IRQFLAGS2 );
                    if( ( irqFlags & RF_IRQFLAGS2_CRCOK ) != RF_IRQFLAGS2_CRCOK )
                    {
                        // Clear Irqs
                        SX1276HalWriteRegister( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                                    RF_IRQFLAGS1_PREAMBLEDETECT |
                                                    RF_IRQFLAGS1_SYNCADDRESSMATCH );
                        SX1276HalWriteRegister( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );


                        SX1276TimerStop( &RxTimeoutTimer  );

                        if( sx1276_settings.Fsk.RxContinuous == false )
                        {
                        	SX1276TimerStop( &RxTimeoutSyncWordTimer);
                        	SX1276SetOpMode( RF_OPMODE_STANDBY );
                            //SX1276SetOpMode( RF_OPMODE_SLEEP );
                        	sx1276_settings.State = RF_IDLE;

                        }
                        else
                        {
                            // Continuous mode restart Rx chain
                            SX1276HalWriteRegister( REG_RXCONFIG, SX1276HalReadRegister( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                        }

                        if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                        {
                            RadioEvents->RxError( );
                        }
                        sx1276_settings.FskPacketHandler.PreambleDetected = false;
                        sx1276_settings.FskPacketHandler.SyncWordDetected = false;
                        sx1276_settings.FskPacketHandler.NbBytes = 0;
                        sx1276_settings.FskPacketHandler.Size = 0;
                        break;
                    }
                }

                // Read received packet size
                if( ( sx1276_settings.FskPacketHandler.Size == 0 ) && ( sx1276_settings.FskPacketHandler.NbBytes == 0 ) )
                {
                    if( sx1276_settings.Fsk.FixLen == false )
                    {
                        SX1276ReadFifo( ( uint8_t* )&sx1276_settings.FskPacketHandler.Size, 1 );
                    }
                    else
                    {
                        sx1276_settings.FskPacketHandler.Size = SX1276HalReadRegister( REG_PAYLOADLENGTH );
                    }
                    SX1276ReadFifo( RxTxBuffer + sx1276_settings.FskPacketHandler.NbBytes, sx1276_settings.FskPacketHandler.Size - sx1276_settings.FskPacketHandler.NbBytes );
                    sx1276_settings.FskPacketHandler.NbBytes += ( sx1276_settings.FskPacketHandler.Size - sx1276_settings.FskPacketHandler.NbBytes );
                }
                else
                {
                    SX1276ReadFifo( RxTxBuffer + sx1276_settings.FskPacketHandler.NbBytes, sx1276_settings.FskPacketHandler.Size - sx1276_settings.FskPacketHandler.NbBytes );
                    sx1276_settings.FskPacketHandler.NbBytes += ( sx1276_settings.FskPacketHandler.Size - sx1276_settings.FskPacketHandler.NbBytes );
                }

                SX1276TimerStop( &RxTimeoutTimer );

                if( sx1276_settings.Fsk.RxContinuous == false )
                {
                	SX1276SetOpMode( RF_OPMODE_STANDBY );
                    sx1276_settings.State = RF_IDLE;
                    //SX1276SetOpMode( RF_OPMODE_SLEEP );
                    SX1276TimerStop( &RxTimeoutSyncWordTimer);
                }
                else
                {
                    // Continuous mode restart Rx chain
                    SX1276HalWriteRegister( REG_RXCONFIG, SX1276HalReadRegister( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                }

                if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                {
                    RadioEvents->RxDone( RxTxBuffer, sx1276_settings.FskPacketHandler.Size, sx1276_settings.FskPacketHandler.RssiValue, 0 );
                }
                sx1276_settings.FskPacketHandler.PreambleDetected = false;
                sx1276_settings.FskPacketHandler.SyncWordDetected = false;
                sx1276_settings.FskPacketHandler.NbBytes = 0;
                sx1276_settings.FskPacketHandler.Size = 0;
                break;
            case MODEM_LORA:
                {
                    // Clear Irq
                    SX1276HalWriteRegister( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );

                    irqFlags = SX1276HalReadRegister( REG_LR_IRQFLAGS );
                    if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
                    {
                        // Clear Irq
                        SX1276HalWriteRegister( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );

                        if( sx1276_settings.LoRa.RxContinuous == false )
                        {
                        	SX1276SetOpMode( RF_OPMODE_STANDBY );
                            sx1276_settings.State = RF_IDLE;
                            //SX1276SetOpMode( RF_OPMODE_SLEEP );
                        }
                        //osTimerStop( RxTimeoutTimer );
                        SX1276TimerStop( &RxTimeoutTimer );
                        if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                        {
                            RadioEvents->RxError( );
                        }
                        break;
                    }

                    // Returns SNR value [dB] rounded to the nearest integer value
                    sx1276_settings.LoRaPacketHandler.SnrValue = ( ( ( int8_t )SX1276HalReadRegister( REG_LR_PKTSNRVALUE ) ) + 2 ) >> 2;

                    int16_t rssi = SX1276HalReadRegister( REG_LR_PKTRSSIVALUE );
                    if( sx1276_settings.LoRaPacketHandler.SnrValue < 0 )
                    {
                        if( sx1276_settings.Channel > RF_MID_BAND_THRESH )
                        {
                            sx1276_settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +
                                                                          sx1276_settings.LoRaPacketHandler.SnrValue;
                        }
                        else
                        {
                            sx1276_settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) +
                                                                          sx1276_settings.LoRaPacketHandler.SnrValue;
                        }
                    }
                    else
                    {
                        if( sx1276_settings.Channel > RF_MID_BAND_THRESH )
                        {
                            sx1276_settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
                        }
                        else
                        {
                            sx1276_settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
                        }
                    }

                    sx1276_settings.LoRaPacketHandler.Size = SX1276HalReadRegister( REG_LR_RXNBBYTES );
                    SX1276HalWriteRegister( REG_LR_FIFOADDRPTR, SX1276HalReadRegister( REG_LR_FIFORXCURRENTADDR ) );
                    SX1276ReadFifo( RxTxBuffer, sx1276_settings.LoRaPacketHandler.Size );

                    if( sx1276_settings.LoRa.RxContinuous == false )
                    {
                    	SX1276SetOpMode( RF_OPMODE_STANDBY );
                        sx1276_settings.State = RF_IDLE;
                        //SX1276SetOpMode( RF_OPMODE_SLEEP );
                    }
                    SX1276TimerStop( &RxTimeoutTimer );

                    if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                    {
                        RadioEvents->RxDone( RxTxBuffer, sx1276_settings.LoRaPacketHandler.Size, sx1276_settings.LoRaPacketHandler.RssiValue, sx1276_settings.LoRaPacketHandler.SnrValue );
                    }
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            SX1276TimerStop( &TxTimeoutTimer );
            // TxDone interrupt
            switch( sx1276_settings.Modem )
            {
            case MODEM_LORA:
                // Clear Irq
                SX1276HalWriteRegister( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
                // Intentional fall through
            case MODEM_FSK:
            default:
                SX1276SetOpMode( RF_OPMODE_STANDBY );
            	sx1276_settings.State = RF_IDLE;

            	//SX1276SetOpMode( RF_OPMODE_SLEEP );
                if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
                {
                    RadioEvents->TxDone( );
                }
                break;
            }
            break;
        default:
            break;
    }
}

static void SX1276OnDio1Irq()
{
    switch( sx1276_settings.State )
    {
        case RF_RX_RUNNING:
            switch( sx1276_settings.Modem )
            {
            case MODEM_FSK:
                // Check FIFO level DIO1 pin state
                //
                // As DIO1 interrupt is triggered when a rising or a falling edge is detected the IRQ handler must
                // verify DIO1 pin state in order to decide if something has to be done.
                // When radio is operating in FSK reception mode a rising edge must be detected in order to handle the
                // IRQ.
            	//if( SX1276GetDio1PinState( ) == 0 )
                if (gpio_read(RADIO_DIO1_Port, RADIO_DIO1_Pin) == 0)
                {
                    break;
                }
                // Stop timer
                SX1276TimerStop( &RxTimeoutSyncWordTimer );

                // FifoLevel interrupt
                // Read received packet size
                if( ( sx1276_settings.FskPacketHandler.Size == 0 ) && ( sx1276_settings.FskPacketHandler.NbBytes == 0 ) )
                {
                    if( sx1276_settings.Fsk.FixLen == false )
                    {
                        SX1276ReadFifo( ( uint8_t* )&sx1276_settings.FskPacketHandler.Size, 1 );
                    }
                    else
                    {
                        sx1276_settings.FskPacketHandler.Size = SX1276HalReadRegister( REG_PAYLOADLENGTH );
                    }
                }

                // ERRATA 3.1 - PayloadReady Set for 31.25ns if FIFO is Empty
                //
                //              When FifoLevel interrupt is used to offload the
                //              FIFO, the microcontroller should  monitor  both
                //              PayloadReady  and FifoLevel interrupts, and
                //              read only (FifoThreshold-1) bytes off the FIFO
                //              when FifoLevel fires
                if( ( sx1276_settings.FskPacketHandler.Size - sx1276_settings.FskPacketHandler.NbBytes ) >= sx1276_settings.FskPacketHandler.FifoThresh )
                {
                    SX1276ReadFifo( ( RxTxBuffer + sx1276_settings.FskPacketHandler.NbBytes ), sx1276_settings.FskPacketHandler.FifoThresh - 1 );
                    sx1276_settings.FskPacketHandler.NbBytes += sx1276_settings.FskPacketHandler.FifoThresh - 1;
                }
                else
                {
                    SX1276ReadFifo( ( RxTxBuffer + sx1276_settings.FskPacketHandler.NbBytes ), sx1276_settings.FskPacketHandler.Size - sx1276_settings.FskPacketHandler.NbBytes );
                    sx1276_settings.FskPacketHandler.NbBytes += ( sx1276_settings.FskPacketHandler.Size - sx1276_settings.FskPacketHandler.NbBytes );
                }
                break;
            case MODEM_LORA:
                // Check RxTimeout DIO1 pin state
                //
                // DIO1 irq is setup to be triggered on rsing and falling edges
                // As DIO1 interrupt is triggered when a rising or a falling edge is detected the IRQ handler must
                // verify DIO1 pin state in order to decide if something has to be done.
                // When radio is operating in LoRa reception mode a rising edge must be detected in order to handle the
                // IRQ.
            	//if( SX1276GetDio1PinState( ) == 0 )
                if (gpio_read(RADIO_DIO1_Port, RADIO_DIO1_Pin) == 0)
                {
                    break;
                }
//              // Sync time out
                SX1276TimerStop( &RxTimeoutTimer);

                // Clear Irq
                SX1276HalWriteRegister( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT );

                SX1276SetOpMode( RF_OPMODE_STANDBY );
                //SX1276SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
                sx1276_settings.State = RF_IDLE;


                if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
                {
                    RadioEvents->RxTimeout( );
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( sx1276_settings.Modem )
            {
            case MODEM_FSK:
                // Check FIFO level DIO1 pin state
                //
                // As DIO1 interrupt is triggered when a rising or a falling edge is detected the IRQ handler must
                // verify DIO1 pin state in order to decide if something has to be done.
                // When radio is operating in FSK transmission mode a falling edge must be detected in order to handle
                // the IRQ.
            	//if( SX1276GetDio1PinState( ) == 1 )
                if (gpio_read(RADIO_DIO1_Port, RADIO_DIO1_Pin) == 1)//xx if (gpio_read(RADIO_DIO1_Port, RADIO_DIO1_Pin) == 0)
                {
                    break;
                }

                // FifoLevel interrupt
                if( ( sx1276_settings.FskPacketHandler.Size - sx1276_settings.FskPacketHandler.NbBytes ) > sx1276_settings.FskPacketHandler.ChunkSize )
                {
                    SX1276WriteFifo( ( RxTxBuffer + sx1276_settings.FskPacketHandler.NbBytes ), sx1276_settings.FskPacketHandler.ChunkSize );
                    sx1276_settings.FskPacketHandler.NbBytes += sx1276_settings.FskPacketHandler.ChunkSize;
                }
                else
                {
                    // Write the last chunk of data
                    SX1276WriteFifo( RxTxBuffer + sx1276_settings.FskPacketHandler.NbBytes, sx1276_settings.FskPacketHandler.Size - sx1276_settings.FskPacketHandler.NbBytes );
                    sx1276_settings.FskPacketHandler.NbBytes += sx1276_settings.FskPacketHandler.Size - sx1276_settings.FskPacketHandler.NbBytes;
                }
                break;
            case MODEM_LORA:
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
}

static void SX1276OnDio2Irq()
{
    switch( sx1276_settings.State )
    {
        case RF_RX_RUNNING:
            switch( sx1276_settings.Modem )
            {
            case MODEM_FSK:
                // Checks if DIO4 is connected. If it is not PreambleDetected is set to true.
#if RADIO_DIO4_ENABLE
                sx1276_settings.FskPacketHandler.PreambleDetected = true;
#endif

                if( ( sx1276_settings.FskPacketHandler.PreambleDetected != 0 ) && ( sx1276_settings.FskPacketHandler.SyncWordDetected == 0 ) )
                {
                	SX1276TimerStop( &RxTimeoutSyncWordTimer);

                    sx1276_settings.FskPacketHandler.SyncWordDetected = true;

                    sx1276_settings.FskPacketHandler.RssiValue = -( SX1276HalReadRegister( REG_RSSIVALUE ) >> 1 );

                    sx1276_settings.FskPacketHandler.AfcValue = ( int32_t )SX1276ConvertPllStepToFreqInHz( ( ( uint16_t )SX1276HalReadRegister( REG_AFCMSB ) << 8 ) |
                                                                                                           ( uint16_t )SX1276HalReadRegister( REG_AFCLSB ) );
                    sx1276_settings.FskPacketHandler.RxGain = ( SX1276HalReadRegister( REG_LNA ) >> 5 ) & 0x07;
                }
                break;
            case MODEM_LORA:
                if( sx1276_settings.LoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    SX1276HalWriteRegister( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

                    if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        RadioEvents->FhssChangeChannel( ( SX1276HalReadRegister( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( sx1276_settings.Modem )
            {
            case MODEM_FSK:
                break;
            case MODEM_LORA:
                if( sx1276_settings.LoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    SX1276HalWriteRegister( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

                    if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        RadioEvents->FhssChangeChannel( ( SX1276HalReadRegister( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
                }
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
}

static void SX1276OnDio3Irq()
{
    switch( sx1276_settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        if( ( SX1276HalReadRegister( REG_LR_IRQFLAGS ) & RFLR_IRQFLAGS_CADDETECTED ) == RFLR_IRQFLAGS_CADDETECTED )
        {
            // Clear Irq
            SX1276HalWriteRegister( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE );
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( true );
            }
        }
        else
        {
            // Clear Irq
            SX1276HalWriteRegister( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE );
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( false );
            }
        }
        break;
    default:
        break;
    }
}

static void SX1276OnDio4Irq()
{
    switch( sx1276_settings.Modem )
    {
    case MODEM_FSK:
        {
            if( sx1276_settings.FskPacketHandler.PreambleDetected == false )
            {
                sx1276_settings.FskPacketHandler.PreambleDetected = true;
            }
        }
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

static uint8_t SX1276GetPaSelect( uint32_t channel )
{
	return RF_PACONFIG_PASELECT_PABOOST;
}


__weak void SX1276SetAntSwLowPower( bool status )
{
}

__weak void SX1276SetAntSw( uint8_t opMode )
{
}

__weak void SX1276SetBoardTcxo( uint8_t state )
{
}

__weak uint32_t SX1276GetBoardTcxoWakeupTime() {
	return 0;
}

__weak bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

__weak void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276HalReadRegister( REG_PACONFIG );
    paDac = SX1276HalReadRegister( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( sx1276_settings.Channel );

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power > 0 )
        {
            if( power > 15 )
            {
                power = 15;
            }
            paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( 7 << 4 ) | ( power );
        }
        else
        {
            if( power < -4 )
            {
                power = -4;
            }
            paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( 0 << 4 ) | ( power + 4 );
        }
    }
    SX1276HalWriteRegister( REG_PACONFIG, paConfig );
    SX1276HalWriteRegister( REG_PADAC, paDac );
}


