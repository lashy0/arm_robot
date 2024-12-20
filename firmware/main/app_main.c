#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_vfs_dev.h"

#include "esp_log.h"

#include "i2c.h"
#include "pca9685.h"
#include "arm_robot.h"

static const char *TAG = "app_main";

#define I2C_MASTER_SCL_IO          GPIO_NUM_7
#define I2C_MASTER_SDA_IO          GPIO_NUM_6 
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000

#define PCA9685_I2C_ADDR           0x40
#define PWM_FREQUENCY              50

#define QUEUE_SIZE 20

arm_robot_t robot;

// Очередь для команд для сервоприводов
typedef struct {
    float angles[6];
} servo_command_t;

static QueueHandle_t servo_queue;

esp_err_t add_command_to_queue(float *angles) {
    servo_command_t command;
    memcpy(command.angles, angles, sizeof(command.angles));

    if (xQueueSend(servo_queue, &command, pdMS_TO_TICKS(10)) == pdPASS) {
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

void process_servo_commands(void *arg) {
    servo_command_t command;
    while (1) {
        // Ждем, пока появится команда в очереди
        if (xQueueReceive(servo_queue, &command, portMAX_DELAY) == pdPASS) {
            // Выставляем углы
            servo_pca9685_set_angle(&robot.base_servo, command.angles[0], PWM_FREQUENCY);
            servo_pca9685_set_angle(&robot.shoulder_servo, command.angles[1], PWM_FREQUENCY);
            servo_pca9685_set_angle(&robot.elbow_servo, command.angles[2], PWM_FREQUENCY);
            servo_pca9685_set_angle(&robot.wrist_servo, command.angles[3], PWM_FREQUENCY);

            ESP_LOGI(TAG, "command start");
            vTaskDelay(pdMS_TO_TICKS(25));
        }
    }
}

static void usb_serial_task(void *arg) {
    uint8_t rxbuf[128];
    char input_buffer[128];
    size_t bytes_read = 0;
    size_t input_length = 0;

    while (1) {
        bytes_read = usb_serial_jtag_read_bytes(rxbuf, 128, pdMS_TO_TICKS(10));

        if (bytes_read > 0) {
            for (size_t i = 0; i < bytes_read; i++) {
                if (rxbuf[i] == '\n') {
                    input_buffer[input_length] = '\0';

                    if (strcmp(input_buffer, "STATUS") == 0) {
                        if (uxQueueSpacesAvailable(servo_queue) > 0) {
                            usb_serial_jtag_write_bytes((const uint8_t *)"READY\n", 6, pdMS_TO_TICKS(10));
                        } else {
                            usb_serial_jtag_write_bytes((const uint8_t *)"BUSY\n", 5, pdMS_TO_TICKS(10));
                        }
                    } else {
                        // Парсинг команд и добавление в очередь
                        float angles[4] = {0};
                        int items = sscanf(input_buffer, "%f,%f,%f,%f",
                               &angles[0], &angles[1], &angles[2], &angles[3]);
                        
                        // 6
                        if (items != 4) {
                            ESP_LOGW(TAG, "Invalid input: %s", input_buffer);
                        }
                        else {
                            if (add_command_to_queue(angles) != ESP_OK) {
                                ESP_LOGW(TAG, "Queue is full. Command not added.");
                            }
                        }
                    }

                    input_length = 0;
                } else if (input_length < 128 - 1) {
                    input_buffer[input_length++] = rxbuf[i];
                } else {
                    ESP_LOGW(TAG, "Input buffer overflow, clearing buffer");
                    input_length = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    // Set level log
    esp_log_level_set("servo_pca9685", ESP_LOG_WARN);
    esp_log_level_set("arm_robot", ESP_LOG_WARN);
    
    esp_err_t ret;

    // Initialize I2C
    i2c_config_bus_t i2c_master_config = {
        .port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO
    };

    i2c_bus_t i2c_bus = {0};

    ret = i2c_master_init(&i2c_bus, &i2c_master_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus initialized failed");
        return;
    }
    ESP_LOGI(TAG, "I2C bus initialized successfully");
    // End

    // Initialize PCA9685
    pca9685_config_t pca9685_config = {
        .i2c_address = PCA9685_I2C_ADDR,
        .bus_handle = i2c_bus.handle,
        .scl_speed = I2C_MASTER_FREQ_HZ
    };

    pca9685_t pca9685 = {0};

    ret = pca9685_init(&pca9685_config, &pca9685);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 initialized failed");
        return;
    }
    ESP_LOGI(TAG, "PCA9685 initialized successfully");

    ret = pca9685_set_pwm_freq(&pca9685, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return;
    }
    ESP_LOGI(TAG, "PCA9685 set pwm frequency 50 Hz");
    // End

    // Initialize robot
    arm_robot_init(&robot, &pca9685);

    ret = arm_robot_home_state(&robot);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Robot is not set home position");
        return;
    }
    ESP_LOGI(TAG, "Robot is in home position");
    // End

    // Initialize USB Serial JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = 128,
        .tx_buffer_size = 128
    };
    ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB Serial JTAG initialized failed");
        return;
    }

    esp_vfs_usb_serial_jtag_use_driver();
    ESP_LOGI(TAG, "USB Serial JTAG initialized successfully");
    // End

    // Initialize queue commands
    servo_queue = xQueueCreate(QUEUE_SIZE, sizeof(servo_command_t));
    if (servo_queue == NULL) {
        printf("Failed to create servo command queue\n");
        return;
    }
    // End

    // Main
    xTaskCreate(process_servo_commands, "Process servo commands", 2048, NULL, 10, NULL);
    
    xTaskCreate(usb_serial_task, "USB Serial Task", 4096, NULL, 5, NULL);
    // End
}
