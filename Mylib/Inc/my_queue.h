/**
 * @file    my_queue.h
 * @brief   Frame 36 byte và hàng đợi vòng cho quỹ đạo nhận từ PC qua USB CDC.
 * @author  Lap4all
 * @date    2026-06-08
 */

#ifndef MYLIB_INC_MY_QUEUE_H_
#define MYLIB_INC_MY_QUEUE_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Bố cục frame 36 byte (little-endian):
 *   [0]      0xAA          Header byte 0
 *   [1]      0x55          Header byte 1
 *   [2..5]   uint32_t      Timestep (LE)
 *   [6..9]   float         theta1 (deg, LE)
 *   [10..13] float         vel1   (deg/s, LE)
 *   [14..17] float         theta2 (deg, LE)
 *   [18..21] float         vel2   (deg/s, LE)
 *   [22..25] float         theta3 (deg, LE)
 *   [26..29] float         vel3   (deg/s, LE)
 *   [30..33] float         theta4 (deg, LE) — servo, chưa dùng
 *   [34]     uint8_t       Checksum = XOR(bytes[2..33])
 *   [35]     0xFF          Footer
 */

/** @brief Kích thước frame thô nhận từ PC, tính bằng byte. */
#define MY_QUEUE_FRAME_SIZE         36U

/** @brief Byte đầu của header frame quỹ đạo. */
#define MY_QUEUE_FRAME_HEADER_0     0xAAU

/** @brief Byte thứ hai của header frame quỹ đạo. */
#define MY_QUEUE_FRAME_HEADER_1     0x55U

/** @brief Byte footer cuối frame quỹ đạo. */
#define MY_QUEUE_FRAME_FOOTER       0xFFU

/**
 * @brief  Dữ liệu đã giải mã của một frame quỹ đạo.
 */
typedef struct {
    uint32_t timestep;    /**< Chỉ số thời gian mẫu. */
    float    theta1_deg;  /**< Góc mục tiêu motor1, tính bằng độ. */
    float    vel1_dps;    /**< Vận tốc motor1, tính bằng độ/giây. */
    float    theta2_deg;  /**< Góc mục tiêu motor2, tính bằng độ. */
    float    vel2_dps;    /**< Vận tốc motor2, tính bằng độ/giây. */
    float    theta3_deg;  /**< Góc mục tiêu motor3, tính bằng độ. */
    float    vel3_dps;    /**< Vận tốc motor3, tính bằng độ/giây. */
    float    theta4_deg;  /**< Góc khớp 4 (servo, chưa sử dụng). */
} MyQueue_Frame_t;

/**
 * @brief  Khởi tạo hàng đợi và bộ parser byte về trạng thái ban đầu.
 */
void     MyQueue_Init(void);

/**
 * @brief  Nạp byte thô từ USB CDC và tự động parse thành frame khi đủ dữ liệu.
 * @param  data:   Con trỏ tới mảng byte cần xử lý.
 * @param  length: Số byte cần xử lý.
 * @note   Parser giữ trạng thái nội bộ giữa các lần gọi — xử lý đúng frame
 *         bị tách thành nhiều lần nhận USB.
 */
void     MyQueue_FeedBytes(const uint8_t *data, uint16_t length);

/**
 * @brief  Lấy một frame từ đầu hàng đợi.
 * @param  frame: Nơi lưu frame lấy ra.
 * @return true nếu lấy thành công, false nếu hàng đợi rỗng.
 */
bool     MyQueue_Pop(MyQueue_Frame_t *frame);

/**
 * @brief  Trả về số frame hiện có trong hàng đợi.
 * @return Số frame đang chờ thực thi.
 */
uint16_t MyQueue_Count(void);

/**
 * @brief  Kiểm tra hàng đợi có rỗng không.
 * @return true nếu không còn frame nào.
 */
bool     MyQueue_IsEmpty(void);

/**
 * @brief  Kiểm tra hàng đợi có đầy không.
 * @return true nếu không còn chỗ cho frame mới.
 */
bool     MyQueue_IsFull(void);

/**
 * @brief  Xóa toàn bộ frame và đặt lại parser về trạng thái tìm header.
 */
void     MyQueue_Flush(void);

/**
 * @brief  Kiểm tra có frame nào bị drop do queue đầy kể từ lần gọi trước.
 * @return true nếu có drop, tự reset sau khi đọc.
 */
bool     MyQueue_WasDropped(void);

#endif /* MYLIB_INC_MY_QUEUE_H_ */
