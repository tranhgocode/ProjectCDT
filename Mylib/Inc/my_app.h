/**
 * @file    my_app.h
 * @brief   Giao diện tầng ứng dụng xử lý lệnh USB CDC.
 * @author  Lap4all
 * @date    2026-05-16
 */

#ifndef MYLIB_INC_MY_APP_H_
#define MYLIB_INC_MY_APP_H_

/**
 * @brief  Khởi tạo tầng ứng dụng và module điều khiển yaw.
 */
void my_app_init(void);

/**
 * @brief  Chạy tác vụ ứng dụng chính, cần gọi lặp lại trong vòng lặp main.
 */
void my_app_process(void);

#endif /* MYLIB_INC_MY_APP_H_ */
