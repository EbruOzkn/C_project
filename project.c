#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// I2C configuration
#define PIN_SDA 8
#define PIN_CLK 9
#define I2C_ADDRESS 0x68 // I2C address of MPU6050

// MPU6050 registers
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1   0x6B

static char tag[] = "mpu6050";

void task_mpu6050(void *ignore) {
    ESP_LOGI(tag, ">> MPU6050 Task");

    // Configure I2C communication
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = PIN_SDA;
    conf.scl_io_num = PIN_CLK;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    // Initialize variables
    uint8_t data[6]; // 6 bytes for X, Y, Z accelerometer data
    short accel_x, accel_y, accel_z;

    // Configure MPU6050
    i2c_cmd_handle_t cmd;
    vTaskDelay(200 / portTICK_PERIOD_MS);

    // Wake up MPU6050
    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0, 1)); // Clear sleep bit
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    while (1) {
        // Read accelerometer data
        cmd = i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_start(cmd));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1));
        ESP_ERROR_CHECK(i2c_master_stop(cmd));
        ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
        i2c_cmd_link_delete(cmd);

        cmd = i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_start(cmd));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));
        ESP_ERROR_CHECK(i2c_master_read(cmd, data, sizeof(data), I2C_MASTER_LAST_NACK));
        ESP_ERROR_CHECK(i2c_master_stop(cmd));
        ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
        i2c_cmd_link_delete(cmd);

        // Parse accelerometer data
        accel_x = (data[0] << 8) | data[1];
        accel_y = (data[2] << 8) | data[3];
        accel_z = (data[4] << 8) | data[5];

        // Log accelerometer data
        ESP_LOGI(tag, "Accel X: %d, Y: %d, Z: %d", accel_x, accel_y, accel_z);

        vTaskDelay(500 / portTICK_PERIOD_MS); // Delay before next read
    }

    vTaskDelete(NULL);
}
	
	void init_leds ()
	{
		
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode= GPIO_MODE_OUTPUT;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
		for (int i=0; i< NUM_LEDS; i++)
		{
			io_conf.pin_bit_mask = (1ULL <<< ledPins[i]); ESP_ERROR_CHECK (gpio_config(&io_conf));
			int map (int value, int fromLow, int fromHigh, int toLow, int toHigh)
		}
	}
	
	int map (int value, int fromLow, int fromHigh, int toLow, int toHigh)
	{
		int mapped = (value-fromLow) * (toHigh - toLow) / (fromHigh-fromLow) + toLow; 
		  // Clamping to ensure value is within bounds
    	if (mapped < toLow)  return toLow;
		if (mapped > toHigh) return toHigh;
		return mapped;
	}
	
	void update_leds (short accel_x)
	{
		int ledIndex = map (accel_x, -16384, 16384, 0, NUM_LEDS);
		for (int i=0; i< NUM_LEDS; i++)
		{
			if (i ledIndex)
			{
				gpio_set_level (ledPins[i], 1); // Turn on LED
			}
			else
			{
				gpio_set_level (ledPins [i], 0); // Turn off LED
			}
			

		}
	}

void app_main() {
    // Create task to read MPU6050 data
    xTaskCreate(task_mpu6050, "mpu6050_task", 4096, NULL, 5, NULL);
}
