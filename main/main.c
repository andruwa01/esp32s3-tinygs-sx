#include "main.h"
#include "sx1276-core.h"
#include "sx1276-spi.h"

#include "driver/i2c.h"
#include <time.h>

static const char *TAG = "main";

void rf_data_handler(uint8_t *buff, uint8_t len) {

}


void SX1276HalReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    //uint8_t i;
    //NSS = 0;
    gpio_set_level(CONFIG_PIN_NUM_SX1278_CS, 0);

    Sx1276SpiInOut(offset & 0x7F);

    Sx1276SpiReceive(buffer, size);

    //NSS = 1;
    gpio_set_level(CONFIG_PIN_NUM_SX1278_CS, 1);
}

typedef enum {
	DS1338_OK = 0,
	DS1338_ERROR,
	DS1338_TIMEOUT,
	DS1338_ENABLE,
	DS1338_DISABLE
} ds1338_result_t;

// Device address
#define DS1338_I2C_ADDRESS			0x68

// Registers
#define DS1338_REG_SECONDS			0x00
#define DS1338_REG_MINUTES			0x01
#define DS1338_REG_HOURS			0x02
#define DS1338_REG_DAY				0x03
#define DS1338_REG_DATE				0x04
#define DS1338_REG_MONTH			0x05
#define DS1338_REG_YEAR				0x06
#define DS1338_REG_CONTROL			0x07
#define DS1338_REG_RAM_BEGIN		0x08
#define DS1338_REG_RAM_END			0x3F

ds1338_result_t ds1338_clock_check(void) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS1338_I2C_ADDRESS<<1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, DS1338_REG_SECONDS, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS1338_I2C_ADDRESS<<1) | I2C_MASTER_READ, 1);
    uint8_t data;
    i2c_master_read_byte(cmd, &data, 1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) {
    	ESP_LOGI(TAG, "i2c error");
    	return DS1338_ERROR;
    }

    ESP_LOGI(TAG, "data = 0x%x", data);

    if(data & 0x80) {
    	ESP_LOGI(TAG, "ds1338 CH=1 (oscillator disable)");
    	return DS1338_DISABLE;
    }
    ESP_LOGI(TAG, "ds1338 CH=0 (oscillator enable)");
    return DS1338_ENABLE;
}

int ds1338_clock_enable(void) {
	return 0;
}

// функция перевода из двоично - десятичной системы в десятичную.
uint8_t BCDtoDEC(uint8_t val) {
	return( (val/16*10) + (val%16) );
}

// И обратно
uint8_t DECtoBCD(uint8_t val) {
	return( (val/10*16) + (val%10) );
}



ds1338_result_t ds1338_get_time(struct tm* timeinfo) {
	uint8_t buff[7];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS1338_I2C_ADDRESS<<1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, DS1338_REG_SECONDS, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS1338_I2C_ADDRESS<<1) | I2C_MASTER_READ, 1);
    i2c_master_read(cmd, buff, 7, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if(ret != ESP_OK) {
    	ESP_LOGI(TAG, "i2c error");
    	return DS1338_ERROR;
    }

    ESP_LOGI(TAG, "buff[0] = 0x%x", buff[0]);

    timeinfo->tm_sec = BCDtoDEC(buff[0] & 0x7F);
    timeinfo->tm_min = BCDtoDEC(buff[1] & 0x7F);
    timeinfo->tm_hour = BCDtoDEC(buff[2] & 0x3F);
    timeinfo->tm_wday = BCDtoDEC(buff[3] & 0x07);
    timeinfo->tm_mday = BCDtoDEC(buff[4] & 0x3F);
    timeinfo->tm_mon = BCDtoDEC(buff[5] & 0x1F) - 1;
    timeinfo->tm_year = BCDtoDEC(buff[6]) + 2000 - 1900;
    return DS1338_OK;
}



ds1338_result_t ds1338_set_time(struct tm* timeinfo) {
	uint8_t buff[7];
    buff[0] = DECtoBCD(timeinfo->tm_sec) & 0x7F;	// auto clock enable (CH=0)
    buff[1] = DECtoBCD(timeinfo->tm_min) & 0x7F;
    buff[2] = DECtoBCD(timeinfo->tm_hour) & 0x3F;
    buff[3] = DECtoBCD(timeinfo->tm_wday) & 0x07;
    buff[4] = DECtoBCD(timeinfo->tm_mday) & 0x3F;
    buff[5] = DECtoBCD(timeinfo->tm_mon + 1) & 0x1F;
    buff[6] = DECtoBCD((timeinfo->tm_year + 1900) % 100);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS1338_I2C_ADDRESS<<1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, DS1338_REG_SECONDS, I2C_MASTER_ACK);
    i2c_master_write(cmd, buff, 7, 1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if(ret != ESP_OK) {
    	ESP_LOGI(TAG, "i2c error");
    	return DS1338_ERROR;
    }
    return DS1338_OK;
}

void app_main(void) {

	esp_err_t ret;

	int les_state = 0;
	gpio_reset_pin(CONFIG_LED_GPIO);
	gpio_set_direction(CONFIG_LED_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(CONFIG_LED_GPIO, 0);

	gpio_reset_pin(CONFIG_PIN_NUM_SX1278_CS);
	gpio_set_direction(CONFIG_PIN_NUM_SX1278_CS, GPIO_MODE_OUTPUT);
	gpio_set_level(CONFIG_PIN_NUM_SX1278_CS, 1);

	gpio_reset_pin(CONFIG_PIN_NUM_SX1278_NRESET);
	gpio_set_direction(CONFIG_PIN_NUM_SX1278_NRESET, GPIO_MODE_OUTPUT);
	gpio_set_level(CONFIG_PIN_NUM_SX1278_NRESET, 1);
	vTaskDelay(5);
	gpio_set_level(CONFIG_PIN_NUM_SX1278_NRESET, 0);
	vTaskDelay(5);
	gpio_set_level(CONFIG_PIN_NUM_SX1278_NRESET, 1);
	vTaskDelay(5);

	// Инициализация I2C
	i2c_config_t i2c_conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = CONFIG_I2C_SDA_PIN_NUM,
		.sda_pullup_en = GPIO_PULLUP_DISABLE,
		.scl_io_num = CONFIG_I2C_SCL_PIN_NUM,
		.scl_pullup_en = GPIO_PULLUP_DISABLE,
		.master.clk_speed = CONFIG_I2C_CLOCK_SPEED
	};

	// Конфигурация шины
	ret = i2c_param_config(I2C_NUM_0, &i2c_conf);
	ESP_LOGI(TAG, "i2c bus initialize: %d", ret);

	// Установка драйвера
	ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
	ESP_LOGI(TAG, "i2c driver install: %d", ret);

	gpio_install_isr_service(0);


	// Инициализация SPI
	spi_bus_config_t spi_cfg = {
		.mosi_io_num = CONFIG_PIN_NUM_MOSI,
		.miso_io_num = CONFIG_PIN_NUM_MISO,
		.sclk_io_num = CONFIG_PIN_NUM_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 0,
		.flags = 0
	};
	ret = spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);
	ESP_LOGI(TAG, "spi bus initialize: %d", ret);


	struct tm timeinfo;

//	if( ds1338_clock_check() == DS1338_DISABLE) {
//
//		timeinfo.tm_hour = 9;
//		timeinfo.tm_min = 0;
//		timeinfo.tm_sec = 0;
//		timeinfo.tm_mday = 20;
//		timeinfo.tm_mon = 10 - 1;
//		timeinfo.tm_year = 2023 - 1900;
//
//		if( ds1338_set_time(&timeinfo) != DS1338_OK) {
//			ESP_LOGE(TAG,"Error Set Time to DS1338");
//		}
//	}

//	timeinfo.tm_hour = 9;
//	timeinfo.tm_min = 0;
//	timeinfo.tm_sec = 0;
//	timeinfo.tm_mday = 20;
//	timeinfo.tm_mon = 10 - 1;
//	timeinfo.tm_year = 2023 - 1900;
//
//	if( ds1338_set_time(&timeinfo) != DS1338_OK) {
//		ESP_LOGE(TAG,"Error Set Time to DS1338");
//	}

	sx1276_init(rf_data_handler);

    while (42) {

    	uint8_t buf[16] = {0};

    	gpio_set_level(CONFIG_LED_GPIO, 1);

    	SX1276HalReadBuffer(0x42, buf, 1);

    	gpio_set_level(CONFIG_LED_GPIO, 0);

    	ESP_LOGI(TAG, "ID: 0x%x", buf[0]);

    	gpio_set_level(CONFIG_LED_GPIO, (++les_state & 1));

    	if( ds1338_get_time(&timeinfo) != DS1338_OK) {
    		ESP_LOGE(TAG,"Error Get Time from DS1338");
    	}

    	printf("time %d:%d:%d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
