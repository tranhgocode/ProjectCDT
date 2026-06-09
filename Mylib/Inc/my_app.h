/**
 * @file    my_app.h
 * @brief   Giao diện tầng ứng dụng xử lý lệnh USB CDC.
 * @author  Lap4all
 * @date    2026-05-16
 */

#ifndef MYLIB_INC_MY_APP_H_
#define MYLIB_INC_MY_APP_H_

/* Bật/tắt tính năng ứng dụng -------------------------------------------- */

/** @brief Tắt một tính năng firmware tùy chọn lúc biên dịch. */
#define MY_APP_MODULE_DISABLED                  0U

/** @brief Bật một tính năng firmware tùy chọn lúc biên dịch. */
#define MY_APP_MODULE_ENABLED                   1U

/** @brief Chỉ dùng đơn vị thô của AS5600; bỏ qua chuyển đổi float độ/radian. */
#define MY_APP_AS5600_FLOAT_UNITS               MY_APP_MODULE_DISABLED

/** @brief Cập nhật bộ đệm current_angle (float) của TMC2209 sau mỗi lần di chuyển. */
#define MY_APP_TMC2209_FLOAT_ANGLE_CACHE        MY_APP_MODULE_DISABLED

/** @brief Biên dịch các API hỗ trợ RPM/góc dùng float bên trong TMC2209. */
#define MY_APP_TMC2209_FLOAT_API                MY_APP_MODULE_DISABLED

/** @brief Đọc giá trị cảm biến trực tiếp, không qua bộ lọc Kalman. */
#define MY_APP_SENSOR_FILTER_NONE               0U

/** @brief Bật bộ lọc Kalman cho kênh AS5600 đo góc yaw. */
#define MY_APP_SENSOR_FILTER_KALMAN             1U

/** @brief Chọn chế độ lọc cảm biến yaw. */
#define MY_APP_SENSOR_FILTER_MODE               MY_APP_SENSOR_FILTER_KALMAN

/* Cài đặt chuyển động động cơ -------------------------------------------- */

/** @brief Giá trị chiều ngược chiều kim đồng hồ (CCW), khớp với TMC2209_DIR_CCW. */
#define MY_APP_MOTOR_DIR_CCW                    0U

/** @brief Giá trị chiều thuận chiều kim đồng hồ (CW), khớp với TMC2209_DIR_CW. */
#define MY_APP_MOTOR_DIR_CW                     1U

/** @brief Tần số xung STEP cho lệnh điều khiển một động cơ yaw. */
#define MY_APP_MOTOR_YAW_SPEED_HZ               3200U

/** @brief Tần số xung STEP cho lệnh điều khiển ba động cơ. */
#define MY_APP_THREE_MOTOR_SPEED_HZ             3200U

/** @brief Chiều tiến của Motor1 khi góc đích lớn hơn góc hiện tại. */
#define MY_APP_MOTOR1_FORWARD_DIR               MY_APP_MOTOR_DIR_CCW

/** @brief Tử số tỉ lệ bước của Motor1. */
#define MY_APP_MOTOR1_STEP_SCALE_NUM            90U

/** @brief Mẫu số tỉ lệ bước của Motor1. */
#define MY_APP_MOTOR1_STEP_SCALE_DEN            20U

/** @brief Chiều tiến của Motor2 khi góc đích lớn hơn góc hiện tại. */
#define MY_APP_MOTOR2_FORWARD_DIR               MY_APP_MOTOR_DIR_CW

/** @brief Tử số tỉ lệ bước của Motor2. */
#define MY_APP_MOTOR2_STEP_SCALE_NUM            90U

/** @brief Mẫu số tỉ lệ bước của Motor2. */
#define MY_APP_MOTOR2_STEP_SCALE_DEN            21U

/** @brief Chiều tiến của Motor3 khi góc đích lớn hơn góc hiện tại. */
#define MY_APP_MOTOR3_FORWARD_DIR               MY_APP_MOTOR_DIR_CCW

/** @brief Tử số tỉ lệ bước của Motor3. */
#define MY_APP_MOTOR3_STEP_SCALE_NUM            90U

/** @brief Mẫu số tỉ lệ bước của Motor3. */
#define MY_APP_MOTOR3_STEP_SCALE_DEN            21U

/* Cài đặt chế độ quỹ đạo ------------------------------------------------- */

/** @brief Số frame tối đa lưu trong hàng đợi quỹ đạo. */
#define MY_QUEUE_CAPACITY               200U

/** @brief Tốc độ STEP tối thiểu cho motor trong chế độ quỹ đạo (Hz). */
#define MY_RUNNER_MIN_SPEED_HZ          100U

/** @brief Tốc độ STEP tối đa cho motor trong chế độ quỹ đạo (Hz). */
#define MY_RUNNER_MAX_SPEED_HZ          40000U

/** @brief Chu kỳ thực thi mỗi sample quỹ đạo, tính bằng mili giây. */
#define MY_RUNNER_TICK_MS               20U

/* Cài đặt bộ lọc cảm biến ------------------------------------------------ */

/** @brief Nhiễu quá trình Kalman khi MY_APP_SENSOR_FILTER_MODE là Kalman. */
#define MY_APP_KALMAN_PROCESS_NOISE             4.0f

/** @brief Nhiễu đo lường Kalman khi MY_APP_SENSOR_FILTER_MODE là Kalman. */
#define MY_APP_KALMAN_MEASUREMENT_NOISE         64.0f

/** @brief Hiệp phương sai khởi tạo Kalman khi MY_APP_SENSOR_FILTER_MODE là Kalman. */
#define MY_APP_KALMAN_INITIAL_COVARIANCE        1000.0f

/** @brief Ngưỡng bám nhanh (centi-độ) để xử lý các bước nhảy lớn của cảm biến. */
#define MY_APP_KALMAN_FAST_TRACK_CDEG           1000.0f

/**
 * @brief  Khởi tạo tầng ứng dụng và module điều khiển yaw.
 */
void my_app_init(void);

/**
 * @brief  Chạy tác vụ ứng dụng chính, cần gọi lặp lại trong vòng lặp main.
 */
void my_app_process(void);

#endif /* MYLIB_INC_MY_APP_H_ */
