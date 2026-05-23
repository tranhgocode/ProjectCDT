/*
 * my_app.h
 *
 *  Created on: May 16, 2026
 *      Author: Lap4all
 */

#ifndef MYLIB_INC_MY_APP_H_
#define MYLIB_INC_MY_APP_H_

#include "TMC2209.h"
#include <stdint.h>

extern TMC2209_HandleTypeDef motor1;
extern TMC2209_HandleTypeDef motor2;
extern TMC2209_HandleTypeDef motor3;

/**
 * @brief  Configure motor handles with timer, GPIO, and TMC2209 addresses.
 */
void motors_config(void);

/**
 * @brief  Initialize all configured TMC2209 motor drivers.
 */
void motors_init(void);

/**
 * @brief  Initialize motor, AS5600 sensor path, USB command state, and yaw zero.
 */
void my_app_init(void);

/**
 * @brief  Calculate motor microsteps required for a relative yaw delta.
 *
 * @param  angle_cdeg: Relative yaw delta in centidegrees.
 * @return Number of motor microsteps to move.
 */
uint32_t my_app_calculate_motor_steps_from_angle(int32_t angle_cdeg);

/**
 * @brief  Main application task. Call repeatedly from the main loop.
 */
void my_app_process(void);

#endif /* MYLIB_INC_MY_APP_H_ */
