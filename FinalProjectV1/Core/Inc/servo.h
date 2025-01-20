/*
 * servo.h
 *
 *  Created on: Dec 7, 2024
 *      Author: flynn
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_


#include "stm32f4xx_hal.h"  // Include the HAL header for STM32F4 series

// Define PWM pulse width limits for servo control
#define MIN_PULSE_WIDTH   1000  // Minimum pulse width for 0° (in microseconds)
#define MAX_PULSE_WIDTH   2000  // Maximum pulse width for 180° (in microseconds)

// Define the servo angle limits
#define MIN_ANGLE         0.0f  // 0° angle
#define MAX_ANGLE         180.0f // 180° angle

// Servo control structure
typedef struct {
    TIM_HandleTypeDef* htim;      // Pointer to timer handle
    uint32_t channel;             // PWM channel (TIM_CHANNEL_1, TIM_CHANNEL_2, etc.)
    uint16_t pulseWidth;
    int current_position;// Pulse width (in microseconds)
} ServoDriver;

// Function prototypes
void Servo_Init(ServoDriver* servo, TIM_HandleTypeDef* htim, uint32_t channel);
void Servo_SetPosition(ServoDriver* servo, float angle);
uint32_t Servo_GetPosition(ServoDriver *servo);


#endif /* INC_SERVO_H_ */
