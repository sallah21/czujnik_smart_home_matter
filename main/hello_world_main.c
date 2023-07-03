/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */



#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>  // printf

//Basic Commands
#define SCD4x_COMMAND_START_PERIODIC_MEASUREMENT              0x21b1
#define SCD4x_COMMAND_READ_MEASUREMENT                        0xec05 // execution time: 1ms
#define SCD4x_COMMAND_STOP_PERIODIC_MEASUREMENT               0x3f86 // execution time: 500ms

//On-chip output signal compensation
#define SCD4x_COMMAND_SET_TEMPERATURE_OFFSET                  0x241d // execution time: 1ms
#define SCD4x_COMMAND_GET_TEMPERATURE_OFFSET                  0x2318 // execution time: 1ms
#define SCD4x_COMMAND_SET_SENSOR_ALTITUDE                     0x2427 // execution time: 1ms
#define SCD4x_COMMAND_GET_SENSOR_ALTITUDE                     0x2322 // execution time: 1ms
#define SCD4x_COMMAND_SET_AMBIENT_PRESSURE                    0xe000 // execution time: 1ms

//Field calibration
#define SCD4x_COMMAND_PERFORM_FORCED_CALIBRATION              0x362f // execution time: 400ms
#define SCD4x_COMMAND_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED  0x2416 // execution time: 1ms
#define SCD4x_COMMAND_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED  0x2313 // execution time: 1ms

//Low power
#define SCD4x_COMMAND_START_LOW_POWER_PERIODIC_MEASUREMENT    0x21ac
#define SCD4x_COMMAND_GET_DATA_READY_STATUS                   0xe4b8 // execution time: 1ms

//Advanced features
#define SCD4x_COMMAND_PERSIST_SETTINGS                        0x3615 // execution time: 800ms
#define SCD4x_COMMAND_GET_SERIAL_NUMBER                       0x3682 // execution time: 1ms
#define SCD4x_COMMAND_PERFORM_SELF_TEST                       0x3639 // execution time: 10000ms
#define SCD4x_COMMAND_PERFORM_FACTORY_RESET                   0x3632 // execution time: 1200ms
#define SCD4x_COMMAND_REINIT                                  0x3646 // execution time: 20ms
#define SCD4x_COMMAND_GET_FEATURE_SET_VERSION                 0x202F // execution time: 1ms

//Low power single shot - SCD41 only
#define SCD4x_COMMAND_MEASURE_SINGLE_SHOT                     0x219d // execution time: 5000ms
#define SCD4x_COMMAND_MEASURE_SINGLE_SHOT_RHT_ONLY            0x2196 // execution time: 50ms

#define I2C_MASTER_NUM      I2C_NUM_0        // Numer interfejsu I2C
#define I2C_MASTER_SCL_IO   7               // GPIO pin dla linii SCL
#define I2C_MASTER_SDA_IO   6               // GPIO pin dla linii SDA
#define I2C_MASTER_FREQ_HZ  100000           // Szybkość transmisji I2C (400 kHz)
#define I2C_MASTER_TIMEOUT_MS  1000

#define SDC40_I2C_ADDR      0x62             // Adres czujnika SDC40

#define TAG                 "SDC40"

// Inicjalizacja interfejsu I2C
static void i2c_master_init()
{
    i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,
    .clk_flags = 0,
    
    };
   
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode,	0, 0, 0));
}


// Funkcja inicjalizująca czujnik
static esp_err_t sdc40_init()
{
    // Wysyłanie komendy inicjalizującej
    uint8_t cmd[] = {0x21, 0x20};
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd_handle, SDC40_I2C_ADDR , true));
    i2c_master_write(cmd_handle, cmd, sizeof(cmd), true);
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 10000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Oczekiwanie na inicjalizację czujnika
    return ESP_OK;
}


static esp_err_t scd40_start_periodic_measurement() {

	i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
   // ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, SDC40_I2C_ADDR, SCD4x_COMMAND_START_PERIODIC_MEASUREMENT,sizeof(SCD4x_COMMAND_START_PERIODIC_MEASUREMENT), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd_handle, SDC40_I2C_ADDR , true));
	ESP_ERROR_CHECK(i2c_master_write(cmd_handle, SCD4x_COMMAND_START_PERIODIC_MEASUREMENT, sizeof(SCD4x_COMMAND_START_PERIODIC_MEASUREMENT), true));
    i2c_master_stop(cmd_handle);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1);
    i2c_cmd_link_delete(cmd_handle);
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Oczekiwanie na inicjalizację czujnika
	return ESP_OK;
}


static esp_err_t scd40_register_read(uint8_t* co2, uint8_t* temperature, uint8_t* humidity){
	 i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    esp_err_t ret;
	i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (SDC40_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, 0x00, true); // Adres rejestru wyników pomiaru
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (SDC40_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd_handle, (uint8_t*)co2, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd_handle, (uint8_t*)co2 + 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd_handle, (uint8_t*)temperature, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd_handle, (uint8_t*)temperature + 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd_handle, (uint8_t*)humidity, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd_handle, (uint8_t*)humidity + 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd_handle);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, pdMS_TO_TICKS(1000));
  

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Błąd odczytu wartości pomiarowych: %d", ret);
		printf(ret);
		printf("\n");
        return ret;
    }
      i2c_cmd_link_delete(cmd_handle);
    *co2 = ((*co2) << 8) | ((*co2) >> 8); // Konwersja endianu CO2
    *temperature = ((*temperature) << 8) | ((*temperature) >> 8); // Konwersja endianu temperatury
    *humidity = ((*humidity) << 8) | ((*humidity) >> 8); // Konwersja endianu wilgotności

    return ESP_OK;
}



void app_main()
{

	int16_t error = 0;

	i2c_master_init();
	ESP_ERROR_CHECK(sdc40_init());

	error = scd40_start_periodic_measurement();
	    if (error) {
	        printf("Error executing scd4x_start_periodic_measurement(): %i\n",
	               error);
	    }
		else{
		printf("Waiting for first measurement... (5 sec)\n");

	    for (;;) {
	        // Read Measurement
	    	vTaskDelay(6000/ portTICK_PERIOD_MS);
	        bool data_ready_flag = false;
	        
	     
	        uint16_t co2=0;
	        int32_t temperature;
	        int32_t humidity;
	        scd40_register_read(&co2, &temperature, &humidity);
	        if (error) {
	            printf("Error executing scd4x_read_measurement(): %i\n", error);
	        } else if (co2 == 0) {
	            printf("Invalid sample detected, skipping.\n");
	        } else {
	            printf("CO2: %u\n", co2);
	            printf("Temperature: %d m°C\n", (int)temperature);
	            printf("Humidity: %d mRH\n",(int) humidity);
	        }
	    }
		}

	    



}
