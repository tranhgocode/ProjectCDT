/**
 * @file    my_queue.h
 * @brief   Hàng đợi vòng lưu RobotCommand nhận từ nhóm 4 (protocol decoder).
 * @author  Lap4all
 * @date    2026-06-11
 */

#ifndef MYLIB_INC_MY_QUEUE_H_
#define MYLIB_INC_MY_QUEUE_H_

#include "protocol.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief Số lệnh tối đa lưu trong hàng đợi. */
#define CMD_QUEUE_SIZE  10U

/**
 * @brief  Khởi tạo hàng đợi về trạng thái rỗng.
 */
void    MyQueue_Init(void);

/**
 * @brief  Thêm một RobotCommand vào cuối hàng đợi.
 * @param  cmd: Con trỏ tới lệnh cần thêm (không được NULL).
 * @return true nếu thêm thành công, false nếu hàng đợi đầy.
 */
bool    MyQueue_Push(const RobotCommand *cmd);

/**
 * @brief  Lấy một RobotCommand từ đầu hàng đợi (FIFO).
 * @param  cmd: Nơi lưu lệnh lấy ra (không được NULL).
 * @return true nếu lấy thành công, false nếu hàng đợi rỗng.
 */
bool    MyQueue_Pop(RobotCommand *cmd);

/**
 * @brief  Kiểm tra hàng đợi có đầy không.
 * @return true nếu không còn chỗ cho lệnh mới.
 */
bool    MyQueue_IsFull(void);

/**
 * @brief  Kiểm tra hàng đợi có rỗng không.
 * @return true nếu không còn lệnh nào.
 */
bool    MyQueue_IsEmpty(void);

/**
 * @brief  Trả về số lệnh đang chờ trong hàng đợi.
 * @return Số lệnh hiện có (0 .. CMD_QUEUE_SIZE).
 */
uint8_t MyQueue_Count(void);

/**
 * @brief  Xóa toàn bộ lệnh trong hàng đợi.
 */
void    MyQueue_Flush(void);

/**
 * @brief  Kiểm tra có lệnh nào bị bỏ qua do queue đầy kể từ lần gọi trước.
 * @return true nếu có drop, tự reset sau khi đọc.
 */
bool    MyQueue_WasDropped(void);

#endif /* MYLIB_INC_MY_QUEUE_H_ */
