/*
 * my_config.h
 *
 *  Created on: May 14, 2026
 *      Author: Lap4all
 *
 *  Dung de luu cau hinh chung
 *  thông số motor, timer, microstep
 */

#ifndef MYLIB_INC_MY_CONFIG_H_
#define MYLIB_INC_MY_CONFIG_H_

#include "main.h"
#include "TMC2209.h"

#define TMC2209_TIMER_CLOCK_HZ      72000000UL
#define TMC2209_TIMER_PRESCALER     71U
#define TMC2209_MOTOR_STEPS_REV     200U

extern TMC2209_HandleTypeDef motor1;
extern TMC2209_HandleTypeDef motor2;
extern TMC2209_HandleTypeDef motor3;

void Motors_Config(void);
void Motors_Init(void);
void MyApp_TestStep1(void);

#endif /* MYLIB_INC_MY_CONFIG_H_ */
