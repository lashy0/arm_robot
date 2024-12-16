#ifndef ARM_ROBOT_H
#define ARM_ROBOT_H

#include "servo_pca9685.h"

// Servo base
#define SERVO_BASE_START_ANGLE  90
#define SERVO_BASE_NUMBER_PWM   5
#define SERVO_BASE_MAX_ANGLE    180
#define SERVO_BASE_MIN_PULSE_US 610
#define SERVO_BASE_MAX_PULSE_US 2454

// Servo shoulder
#define SERVO_SHOULDER_START_ANGLE  90
#define SERVO_SHOULDER_NUMBER_PWM   4
#define SERVO_SHOULDER_MAX_ANGLE    180
#define SERVO_SHOULDER_MIN_PULSE_US 500
#define SERVO_SHOULDER_MAX_PULSE_US 2500

// Servo elbow
#define SERVO_ELBOW_START_ANGLE  90
#define SERVO_ELBOW_NUMBER_PWM   3
#define SERVO_ELBOW_MAX_ANGLE    180
#define SERVO_ELBOW_MIN_PULSE_US 695
#define SERVO_ELBOW_MAX_PULSE_US 2486

// Servo wrist
#define SERVO_WRIST_START_ANGLE  90
#define SERVO_WRIST_NUMBER_PWM   2
#define SERVO_WRIST_MAX_ANGLE    180
#define SERVO_WRIST_MIN_PULSE_US 500
#define SERVO_WRIST_MAX_PULSE_US 2500

// Servo wrist rotational
#define SERVO_WRIST_ROT_START_ANGLE  90
#define SERVO_WRIST_ROT_NUMBER_PWM   1
#define SERVO_WRIST_ROT_MAX_ANGLE    180
#define SERVO_WRIST_ROT_MIN_PULSE_US 530
#define SERVO_WRIST_ROT_MAX_PULSE_US 2530

#define NUM_SERVO 5

typedef struct {
    servo_t base_servo;
    servo_t shoulder_servo;
    servo_t elbow_servo;
    servo_t wrist_servo;
    servo_t wrist_rot_servo;
} arm_robot_t;

/**
 * @brief Initializes the robot arm by setting up all six servos
 */
void arm_robot_init(arm_robot_t *robot, pca9685_t *pca9685);

/**
 * @brief Moves the robot arm to its home (default) position
 * 
 * @param[in] robot Pointer to structure representing the robot arm
 * 
 * @return ESP_OK is Success, or ESP_FAIL
 */
esp_err_t arm_robot_home_state(arm_robot_t *robot);

// TODO: написать описание
esp_err_t arm_robot_move_servo_to_angle(arm_robot_t *robot, uint8_t channel, float angle);

#endif