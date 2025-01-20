#include "servo.h"


// Initialize the servo by specifying the timer and PWM channel
void Servo_Init(ServoDriver* servo, TIM_HandleTypeDef* htim, uint32_t channel) {
    servo->htim = htim;
    servo->channel = channel;
    servo->pulseWidth = MIN_PULSE_WIDTH;
    servo->current_position = 0;  // Initialize position to 0 degrees

    // Start the PWM signal for the given timer and channel
    HAL_TIM_PWM_Start(servo->htim, servo->channel);
    Servo_SetPosition(servo, 90);
}

// Set the position of the servo (0° to 180°)
void Servo_SetPosition(ServoDriver* servo, float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    servo->pulseWidth = 1000 + (angle / 180.0) * (1000);
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, servo->pulseWidth);
    HAL_Delay(100);
    servo->current_position = angle;
}

uint32_t Servo_GetPosition(ServoDriver *servo) {
    return servo->current_position;
}
