#include "esp_log.h"
#include "esp_err.h"
#include "arm_robot.h"

// static const char *TAG = "arm_robot";

void arm_robot_init(arm_robot_t *robot, pca9685_t *pca9685)
{
    // Init base servo
    servo_config_t base_config = {
        .channel = SERVO_BASE_NUMBER_PWM,
        .min_pulse_width = SERVO_BASE_MIN_PULSE_US,
        .max_pulse_width = SERVO_BASE_MAX_PULSE_US,
        .min_angle = 0.0f,
        .max_angle = SERVO_BASE_MAX_ANGLE,
    };

    servo_pca9685_init(&robot->base_servo, pca9685, &base_config);
    robot->base_servo.current_angle = SERVO_BASE_START_ANGLE;
    // End
    
    // Init shoulder servo
    servo_config_t shoulder_config = {
        .channel = SERVO_SHOULDER_NUMBER_PWM,
        .min_pulse_width = SERVO_SHOULDER_MIN_PULSE_US,
        .max_pulse_width = SERVO_SHOULDER_MAX_PULSE_US,
        .min_angle = 0.0f,
        .max_angle = SERVO_SHOULDER_MAX_ANGLE,
    };
    servo_pca9685_init(&robot->shoulder_servo, pca9685, &shoulder_config);
    robot->shoulder_servo.current_angle = SERVO_SHOULDER_START_ANGLE;
    // End

    // Init elbow servo
    servo_config_t elbow_config = {
        .channel = SERVO_ELBOW_NUMBER_PWM,
        .min_pulse_width = SERVO_ELBOW_MIN_PULSE_US,
        .max_pulse_width = SERVO_ELBOW_MAX_PULSE_US,
        .min_angle = 0.0f,
        .max_angle = SERVO_ELBOW_MAX_ANGLE,
    };
    servo_pca9685_init(&robot->elbow_servo, pca9685, &elbow_config);
    robot->elbow_servo.current_angle = SERVO_ELBOW_START_ANGLE;
    // End

    // Init wrist servo
    servo_config_t wrist_config = {
        .channel = SERVO_WRIST_NUMBER_PWM,
        .min_pulse_width = SERVO_WRIST_MIN_PULSE_US,
        .max_pulse_width = SERVO_WRIST_MAX_PULSE_US,
        .min_angle = 0.0f,
        .max_angle = SERVO_WRIST_MAX_ANGLE,
    };
    servo_pca9685_init(&robot->wrist_servo, pca9685, &wrist_config);
    robot->wrist_servo.current_angle = SERVO_WRIST_START_ANGLE;
    // End

    // Init wrist rotational servo
    servo_config_t wrist_rot_config = {
        .channel = SERVO_WRIST_ROT_NUMBER_PWM,
        .min_pulse_width = SERVO_WRIST_ROT_MIN_PULSE_US,
        .max_pulse_width = SERVO_WRIST_ROT_MAX_PULSE_US,
        .min_angle = 0.0f,
        .max_angle = SERVO_WRIST_ROT_MAX_ANGLE,
    };
    servo_pca9685_init(&robot->wrist_rot_servo, pca9685, &wrist_rot_config);
    robot->wrist_rot_servo.current_angle = SERVO_WRIST_ROT_START_ANGLE;
    // End
}

esp_err_t arm_robot_home_state(arm_robot_t *robot)
{
    esp_err_t ret;
    
    
    ret = servo_pca9685_set_angle(&robot->base_servo, SERVO_BASE_START_ANGLE);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = servo_pca9685_set_angle(&robot->shoulder_servo, SERVO_SHOULDER_START_ANGLE);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = servo_pca9685_set_angle(&robot->elbow_servo, SERVO_ELBOW_START_ANGLE);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = servo_pca9685_set_angle(&robot->wrist_servo, SERVO_WRIST_START_ANGLE);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = servo_pca9685_set_angle(&robot->wrist_rot_servo, SERVO_WRIST_ROT_START_ANGLE);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}