/*
 * my_app.c
 *
 *  Created on: May 14, 2026
 *      Author: Lap4all
 */

#include "my_config.h"
#include "tim.h"

TMC2209_HandleTypeDef motor1;
TMC2209_HandleTypeDef motor2;
TMC2209_HandleTypeDef motor3;

void Motors_Config(void)
{
    motor1.id             = 0U;
    motor1.htim           = &htim2;
    motor1.tim_channel    = TIM_CHANNEL_1;
    motor1.dir_port       = DIR_1_GPIO_Port;
    motor1.dir_pin        = DIR_1_Pin;
    motor1.en_port        = EN_1_GPIO_Port;
    motor1.en_pin         = EN_1_Pin;
    motor1.timer_clock_hz = TMC2209_TIMER_CLOCK_HZ;
    motor1.prescaler      = TMC2209_TIMER_PRESCALER;
    motor1.steps_per_rev  = TMC2209_MOTOR_STEPS_REV;
    motor1.microstep      = TMC2209_MICROSTEP_16;
    motor1.slave_address  = TMC2209_SLAVE_ADD1;

    motor2.id             = 1U;
    motor2.htim           = &htim3;
    motor2.tim_channel    = TIM_CHANNEL_1;
    motor2.dir_port       = DIR_2_GPIO_Port;
    motor2.dir_pin        = DIR_2_Pin;
    motor2.en_port        = EN_2_GPIO_Port;
    motor2.en_pin         = EN_2_Pin;
    motor2.timer_clock_hz = TMC2209_TIMER_CLOCK_HZ;
    motor2.prescaler      = TMC2209_TIMER_PRESCALER;
    motor2.steps_per_rev  = TMC2209_MOTOR_STEPS_REV;
    motor2.microstep      = TMC2209_MICROSTEP_16;
    motor2.slave_address  = TMC2209_SLAVE_ADD2;

    motor3.id             = 2U;
    motor3.htim           = &htim4;
    motor3.tim_channel    = TIM_CHANNEL_1;
    motor3.dir_port       = DIR_3_GPIO_Port;
    motor3.dir_pin        = DIR_3_Pin;
    motor3.en_port        = EN_3_GPIO_Port;
    motor3.en_pin         = EN_3_Pin;
    motor3.timer_clock_hz = TMC2209_TIMER_CLOCK_HZ;
    motor3.prescaler      = TMC2209_TIMER_PRESCALER;
    motor3.steps_per_rev  = TMC2209_MOTOR_STEPS_REV;
    motor3.microstep      = TMC2209_MICROSTEP_16;
    motor3.slave_address  = TMC2209_SLAVE_ADD3;
}

void Motors_Init(void)
{
    Motors_Config();

    (void)TMC2209_Init(&motor1);
    (void)TMC2209_Init(&motor2);
    (void)TMC2209_Init(&motor3);

    (void)TMC2209_SetStealthChop(&motor1, true);
    (void)TMC2209_SetStealthChop(&motor2, true);
    (void)TMC2209_SetStealthChop(&motor3, true);
}

void MyApp_TestStep1(void)
{
    Motors_Config();

    (void)TMC2209_Init(&motor1);
    (void)TMC2209_SetStealthChop(&motor1, true);

    TMC2209_Enable(&motor1);
    TMC2209_SetDirection(&motor1, TMC2209_DIR_CW);
    (void)TMC2209_SetSpeedRPM(&motor1, 60.0f);
    (void)TMC2209_Start(&motor1);

    HAL_Delay(2000U);

    TMC2209_Stop(&motor1);
    HAL_Delay(500U);

    TMC2209_SetDirection(&motor1, TMC2209_DIR_CCW);
    (void)TMC2209_SetSpeedRPM(&motor1, 60.0f);
    (void)TMC2209_Start(&motor1);

    HAL_Delay(2000U);

    TMC2209_Stop(&motor1);
    TMC2209_Disable(&motor1);
}
