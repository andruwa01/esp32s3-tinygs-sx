/*!
 *
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "sx1276-spi.h"
#include "sx1276-core.h"
#include "esp_log.h"

//==================================================
typedef struct {
	spi_device_interface_config_t spi_cfg;
	spi_device_handle_t spi_dev;
} sx1276_dev_t;

static const char *TAG = "spi";

/* Private variables ---------------------------------------------------------*/
static sx1276_dev_t sx_dev = {0};

static volatile bool blockingDmaFlag;

static void pre_trans(spi_transaction_t *trans) {
//	ESP_LOGI(TAG, "pre_trans");
}

static void post_trans(spi_transaction_t *trans) {
//	ESP_LOGI(TAG, "post_trans");
	blockingDmaFlag = false;
}

static void wait_blocking_flag(void) {
	while( blockingDmaFlag ) {
//		vTaskDelay(1);
	}
}


void Sx1276SpiInit(void) {

	memset(&sx_dev.spi_cfg, 0, sizeof(sx_dev.spi_cfg));
	sx_dev.spi_cfg.spics_io_num = -1;
	sx_dev.spi_cfg.clock_speed_hz = CONFIG_SPI_CLOCK_SPEED;
	sx_dev.spi_cfg.mode = 0;
	sx_dev.spi_cfg.queue_size = 1;
	sx_dev.spi_cfg.address_bits = 0;		// нет адреса
	sx_dev.spi_cfg.command_bits = 0;		// нет команды
	sx_dev.spi_cfg.pre_cb = pre_trans;
	sx_dev.spi_cfg.post_cb = post_trans;
//	sx_dev.spi_cfg.flags = SPI_DEVICE_NO_DUMMY;// | SPI_DEVICE_NO_RETURN_RESULT;
	esp_err_t ret = spi_bus_add_device(SPI2_HOST, &sx_dev.spi_cfg, &sx_dev.spi_dev);
	ESP_LOGI(TAG, "add sx device to spi bus: %d", ret);
}

void Sx1276SpiDeInit(void) {

}

void Sx1276SpiTransmitReceive( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size ) {
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8 * size;
	t.tx_buffer = txBuffer;
	t.rx_buffer = rxBuffer;
	blockingDmaFlag = true;
	spi_device_polling_transmit(sx_dev.spi_dev, &t);
	wait_blocking_flag();
}

void Sx1276SpiTransmit( uint8_t *txBuffer, uint16_t size ) {
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8 * size;
	t.tx_buffer = txBuffer;
	t.rx_buffer = NULL;
	blockingDmaFlag = true;
	spi_device_polling_transmit(sx_dev.spi_dev, &t);
	wait_blocking_flag();
}

void Sx1276SpiReceive( uint8_t *rxBuffer, uint16_t size ) {
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8 * size;
	t.tx_buffer = NULL;
	t.rx_buffer = rxBuffer;
	blockingDmaFlag = true;
	spi_device_polling_transmit(sx_dev.spi_dev, &t);
	wait_blocking_flag();
//	vTaskDelay(1);
}

uint8_t Sx1276SpiInOut(uint8_t d) {
	uint8_t b = 0;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8;
	t.tx_buffer = &d;
	t.rx_buffer = &b;
	blockingDmaFlag = true;
//	ESP_LOGI(TAG, "spi_device_transmit");
	spi_device_polling_transmit(sx_dev.spi_dev, &t);
//	ESP_LOGI(TAG, "WAIT_FOR_BLOCKING_FLAG");
	wait_blocking_flag();
//	vTaskDelay(1);
//	ESP_LOGI(TAG, "WAIT_FOR_BLOCKING_FLAG DONE");
	return b;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
