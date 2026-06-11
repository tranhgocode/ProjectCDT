/**
 * @file    my_runner.h
 * @brief   Máy trạng thái thực thi quỹ đạo từ PC qua USB CDC.
 * @author  Lap4all
 * @date    2026-06-08
 */

#ifndef MYLIB_INC_MY_RUNNER_H_
#define MYLIB_INC_MY_RUNNER_H_

#include "protocol.h"
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

/* ============================================================================
 * Motion task — Docs/protocol.md §12.5
 * ============================================================================
 *
 * Mô hình BLOCKING, chạy đúng 1 lệnh mỗi lần gọi:
 *   - MotionTask_Run() được gọi liên tục trong vòng main khi app IDLE.
 *   - Queue rỗng  -> không làm gì.
 *   - Queue có lệnh -> pop 1 lệnh, gọi Motion_GotoAngles() chạy tới khi xong,
 *     rồi gửi $D. Vì blocking nên lệnh kế chỉ được lấy ở lần gọi sau, đảm bảo
 *     "chạy xong command hiện tại mới lấy command tiếp theo".
 *
 * Giai đoạn 2 (hiện tại): Motion_GotoAngles() chạy motor thật bằng trap engine
 * của my_controller (StartTrapMove -> TrapProcess loop -> FinishTrapMove).
 */

/**
 * @brief  Lấy một lệnh từ queue và thực thi (1 lệnh mỗi lần gọi).
 * @note   Gọi lặp trong vòng main khi app ở trạng thái IDLE.
 */
void MotionTask_Run(void);

/**
 * @brief  Điều khiển 3 motor chạy tới góc trong lệnh, blocking tới khi xong.
 * @param  cmd: Lệnh cần thực thi (angleN_x100 là góc tuyệt đối ×100).
 * @return true nếu chạy xong thành công, false nếu lỗi motor.
 * @note   Giai đoạn 2: blocking bằng trap engine my_controller; refresh IWDG
 *         trong vòng tick để khối blocking không gây watchdog reset.
 */
bool Motion_GotoAngles(const RobotCommand *cmd);

#endif /* MYLIB_INC_MY_RUNNER_H_ */
