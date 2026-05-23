/*
 * my_app.h
 *
 *  Created on: May 16, 2026
 *      Author: Lap4all
 */

#ifndef MYLIB_INC_MY_APP_H_
#define MYLIB_INC_MY_APP_H_

#include <stdint.h>

#include "TMC2209.h"

extern TMC2209_HandleTypeDef motor1;
extern TMC2209_HandleTypeDef motor2;
extern TMC2209_HandleTypeDef motor3;

/**
 * @brief Configure motor handles with timer, GPIO, and TMC2209 addresses.
 */
void Motors_Config(void);

/**
 * @brief Initialize all configured TMC2209 motor drivers.
 */
void Motors_Init(void);

/**
 * @brief Initialize motor, AS5600 sensor path, and USB command state.
 */
void MyApp_SensorUsb_Init(void);

/**
 * @brief Scale AS5600 raw 12-bit angle to a 0..3999 application scale.
 *
 * @param[in] angle_raw AS5600 raw angle, range 0..4095.
 * @return Scaled angle, range 0..3999.
 */
uint16_t MyApp_ScaleAngleTo4000(uint16_t angle_raw);

/**
 * @brief Calculate motor microsteps required for a relative angle command.
 *
 * @param[in] angle_cdeg Relative angle in centidegrees.
 * @return Number of motor microsteps to move.
 */
uint32_t MyApp_CalculateMotorStepsFromAngle(int32_t angle_cdeg);

/**
 * @brief Main application task. Call repeatedly from the main loop.
 */
void My_app(void);

#endif /* MYLIB_INC_MY_APP_H_ */
