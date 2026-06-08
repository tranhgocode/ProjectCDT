/**
 * @file    my_runner.h
 * @brief   Máy trạng thái thực thi quỹ đạo từ PC qua USB CDC.
 * @author  Lap4all
 * @date    2026-06-08
 */

#ifndef MYLIB_INC_MY_RUNNER_H_
#define MYLIB_INC_MY_RUNNER_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief  Trạng thái hoạt động của bộ thực thi quỹ đạo.
 */
typedef enum {
    MY_RUNNER_STATE_IDLE = 0,  /**< Đứng yên, queue vẫn nhận frame mới. */
    MY_RUNNER_STATE_RUNNING,   /**< Đang thực thi quỹ đạo theo chu kỳ 20 ms. */
} MyRunner_State_t;

/**
 * @brief  Khởi tạo bộ thực thi, hàng đợi quỹ đạo và đặt lại vị trí motor.
 */
void MyRunner_Init(void);

/**
 * @brief  Nạp byte thô USB vào bộ parser frame quỹ đạo.
 * @param  data:   Con trỏ tới mảng byte nhận từ USB CDC.
 * @param  length: Số byte cần xử lý.
 */
void MyRunner_FeedBytes(const uint8_t *data, uint16_t length);

/**
 * @brief  Bắt đầu thực thi quỹ đạo từ frame đầu tiên trong queue.
 * @note   Không có tác dụng nếu queue rỗng hoặc đang ở trạng thái RUNNING.
 */
void MyRunner_Start(void);

/**
 * @brief  Dừng thực thi ngay lập tức, xóa queue và chuyển về IDLE.
 */
void MyRunner_Stop(void);

/**
 * @brief  Trả về trạng thái hiện tại của bộ thực thi.
 * @return MY_RUNNER_STATE_IDLE hoặc MY_RUNNER_STATE_RUNNING.
 */
MyRunner_State_t MyRunner_GetState(void);

/**
 * @brief  Kiểm tra bộ thực thi có đang chạy quỹ đạo không.
 * @return true nếu đang ở trạng thái RUNNING.
 */
bool MyRunner_IsActive(void);

/**
 * @brief  Chạy tác vụ thực thi quỹ đạo không blocking, gọi lặp trong vòng main.
 * @note   Tự quản lý chu kỳ 20 ms bằng HAL_GetTick(). Khi queue cạn,
 *         tự dừng motor và chuyển về IDLE.
 */
void MyRunner_Process(void);

#endif /* MYLIB_INC_MY_RUNNER_H_ */
