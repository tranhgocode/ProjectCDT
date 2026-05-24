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
 * @brief  Cấu hình handle motor với timer, GPIO và địa chỉ TMC2209.
 */
void motors_config(void);

/**
 * @brief  Khởi tạo tất cả driver TMC2209 đã được cấu hình.
 */
void motors_init(void);

/**
 * @brief  Khởi tạo motor, đường cảm biến AS5600, trạng thái USB và yaw zero.
 */
void my_app_init(void);

/**
 * @brief  Tính số microstep motor cần dùng cho một delta yaw tương đối.
 *
 * @param  angle_cdeg: Delta yaw tương đối, tính bằng centi-độ.
 * @return Số microstep motor cần chạy.
 */
uint32_t my_app_calculate_motor_steps_from_angle(int32_t angle_cdeg);

/**
 * @brief  Chạy tác vụ ứng dụng chính, cần gọi lặp lại trong vòng lặp main.
 */
void my_app_process(void);

#endif /* MYLIB_INC_MY_APP_H_ */
