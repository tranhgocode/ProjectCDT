/**
 * @file    my_app.c
 * @brief   Tầng ứng dụng xử lý lệnh USB CDC, phản hồi yaw từ AS5600 và
 *          điều khiển chuyển động động cơ bước TMC2209.
 * @author  Lap4all
 * @date    2026-05-14
 */

#include "my_app.h"
#include "as5600.h"
#include "i2c.h"
#include "my_config.h"
#include "tca9548a.h"
#include "tim.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/** Handle công khai cho driver TMC2209 thứ nhất. */
TMC2209_HandleTypeDef motor1;
/** Handle công khai cho driver TMC2209 thứ hai. */
TMC2209_HandleTypeDef motor2;
/** Handle công khai cho driver TMC2209 thứ ba. */
TMC2209_HandleTypeDef motor3;

/** Kênh AS5600 được chọn qua bộ ghép kênh TCA9548A. */
#define MY_APP_SENSOR_CHANNEL              TCA9548A_CH0
/** Thời gian timeout cho giao dịch I2C của AS5600 và TCA9548A. */
#define MY_APP_I2C_TIMEOUT_MS              20u
/** Kích thước bộ đệm tạm truyền USB CDC, tính bằng byte. */
#define MY_APP_USB_TX_BUFFER_SIZE          192u
/** Kích thước bộ đệm nhận lệnh USB CDC, tính bằng byte. */
#define MY_APP_USB_RX_BUFFER_SIZE          64u
/** Kích thước bộ đệm chuỗi cho một trường góc đã định dạng. */
#define MY_APP_ANGLE_TEXT_BUFFER_SIZE      16u
/** Số vị trí raw AS5600 trong một vòng cơ khí. */
#define MY_APP_AS5600_RAW_STEPS            4096u
/** Một vòng đầy đủ biểu diễn bằng centi-độ. */
#define MY_APP_FULL_TURN_CDEG              36000l
/** Nửa vòng quay dùng để chọn sai lệch góc ngắn nhất. */
#define MY_APP_HALF_TURN_CDEG              18000l
/** Tốc độ động cơ mặc định khi chạy tới góc mục tiêu. */
#define MY_APP_MOTOR_SPEED_RPM             60.0f
/** Tử số tùy chọn cho hệ số scale bước cơ khí. */
#define MY_APP_STEP_SCALE_NUMERATOR        9u
/** Mẫu số tùy chọn cho hệ số scale bước cơ khí. */
#define MY_APP_STEP_SCALE_DENOMINATOR      2u
/** Nhiễu quá trình Kalman, càng lớn thì bộ lọc càng bám nhanh theo góc thật. */
#define MY_APP_KALMAN_PROCESS_NOISE        4.0f
/** Nhiễu đo Kalman, càng lớn thì bộ lọc càng giảm rung mạnh hơn. */
#define MY_APP_KALMAN_MEASUREMENT_NOISE    64.0f
/** Hiệp phương sai ban đầu giúp mẫu đầu tiên được tin cậy nhanh. */
#define MY_APP_KALMAN_INITIAL_COVARIANCE   1000.0f
/** Ngưỡng đổi góc thật để tránh Kalman làm trễ sau khi motor vừa di chuyển. */
#define MY_APP_KALMAN_FAST_TRACK_CDEG      1000.0f
#define MY_APP_USE_KALMAN_FILTER           0        /* Set to 0 để bỏ qua kalman filter */

/**
 * @brief  Các trạng thái chính của máy trạng thái ứng dụng.
 */
typedef enum {
    MY_APP_STATE_WAIT_COMMAND = 0,
    MY_APP_STATE_WAIT_MOTOR,
} my_app_state_t;

/**
 * @brief  Ngữ cảnh runtime được lưu khi bắt đầu một lần chạy tới mục tiêu.
 */
typedef struct {
    int32_t target_yaw_cdeg;        /**< Góc yaw mục tiêu, tính bằng centi-độ. */
    int32_t start_yaw_cdeg;         /**< Góc yaw phần mềm trước khi chạy. */
    int32_t move_delta_cdeg;        /**< Góc chạy tương đối đã ra lệnh. */
    int32_t start_sensor_yaw_cdeg;  /**< Góc yaw cảm biến lấy mẫu trước khi chạy. */
    uint32_t target_steps;          /**< Số microstep motor cần chạy. */
} my_app_move_context_t;

/**
 * @brief  Trạng thái bộ lọc Kalman 1 chiều cho góc tuyệt đối AS5600.
 */
typedef struct {
    float estimate_cdeg;             /**< Góc ước lượng hiện tại, tính bằng centi-độ. */
    float error_covariance;          /**< Độ không chắc chắn của giá trị ước lượng. */
    float process_noise;             /**< Nhiễu mô hình giữa hai lần lấy mẫu liên tiếp. */
    float measurement_noise;         /**< Nhiễu đo của mẫu AS5600 đọc từ I2C. */
    bool is_initialized;             /**< Cờ cho biết bộ lọc đã nhận mẫu đầu tiên. */
} my_app_kalman_filter_t;

extern USBD_HandleTypeDef hUsbDeviceFS;

/** Instance mux TCA9548A dùng để truy cập kênh cảm biến AS5600. */
static TCA9548A_Handle_t s_mux;
/** Mẫu dữ liệu AS5600 mới nhất đọc qua mux. */
static AS5600_Data_t s_as5600_data;
/** Trạng thái hiện tại của máy trạng thái lệnh/chuyển động. */
static my_app_state_t s_app_state = MY_APP_STATE_WAIT_COMMAND;
/** Thông tin chuyển động được giữ từ lúc start motor tới lúc hoàn tất. */
static my_app_move_context_t s_move_context;
/** Cờ cho biết cảm biến đã sẵn sàng để xử lý lệnh hay chưa. */
static bool s_is_sensor_ready = false;
/** Cờ chặn báo lỗi init AS5600 lặp lại liên tục qua USB. */
static bool s_has_init_error_been_reported = false;
/** Góc tuyệt đối AS5600 được chọn làm yaw zero, tính bằng centi-độ. */
static int32_t s_sensor_zero_cdeg = 0;
/** Góc yaw mục tiêu cuối cùng được phần mềm chấp nhận, tính bằng centi-độ. */
static int32_t s_current_yaw_cdeg = 0;
/** Bộ lọc Kalman làm mượt góc tuyệt đối AS5600 trước khi tính yaw. */
static my_app_kalman_filter_t s_sensor_kalman_filter;
/** Bộ đệm truyền USB dùng chung cho các dòng report đã định dạng. */
static char s_usb_tx_buffer[MY_APP_USB_TX_BUFFER_SIZE];

static bool my_app_is_tca9548a_address(uint8_t device_address);
static int8_t my_app_i2c_write(uint8_t device_address,
                               uint8_t reg,
                               uint8_t *buffer,
                               uint16_t length);
static int8_t my_app_i2c_read(uint8_t device_address,
                              uint8_t reg,
                              uint8_t *buffer,
                              uint16_t length);
static void my_app_delay_ms(uint32_t delay_ms);
static bool my_app_usb_is_ready(void);
static void my_app_usb_send_text(const char *text);
static bool my_app_is_zero_command(const uint8_t *buffer, uint16_t length);
static bool my_app_parse_target_angle_cdeg(const uint8_t *buffer,
                                           uint16_t length,
                                           int32_t *target_angle_cdeg);
static void my_app_kalman_reset(my_app_kalman_filter_t *filter);
static int32_t my_app_normalize_absolute_cdeg(int32_t angle_cdeg);
#if (MY_APP_USE_KALMAN_FILTER != 0)
static float my_app_calculate_shortest_angle_error_cdeg(float reference_cdeg,
                                                        int32_t sample_cdeg);
static int32_t my_app_kalman_update(my_app_kalman_filter_t *filter,
                                    int32_t sample_cdeg);
#endif
static int32_t my_app_normalize_sensor_yaw_cdeg(int32_t sensor_angle_cdeg);
static bool my_app_read_sensor_absolute_cdeg(int32_t *sensor_angle_cdeg);
static bool my_app_read_sensor_yaw_cdeg(int32_t *sensor_yaw_cdeg);
static void my_app_format_angle_deg(int32_t angle_cdeg,
                                    char *buffer,
                                    uint16_t buffer_size);
static TMC2209_DirectionTypeDef my_app_get_direction_from_delta(int32_t delta_cdeg);
static int32_t my_app_calculate_sensor_delta_cdeg(int32_t start_cdeg,
                                                  int32_t end_cdeg,
                                                  int32_t expected_delta_cdeg);
static bool my_app_set_zero_from_sensor(void);
static bool my_app_start_target_move(int32_t target_yaw_cdeg);
static void my_app_process_usb_command(void);
static void my_app_process_motor_done(void);

/**
 * @brief  Kiểm tra địa chỉ I2C có thuộc dải địa chỉ TCA9548A hay không.
 * @param  device_address: Địa chỉ I2C 7-bit.
 * @return true nếu là địa chỉ TCA9548A, ngược lại false.
 */
static bool my_app_is_tca9548a_address(uint8_t device_address)
{
    return ((device_address >= TCA9548A_BASE_ADDR) &&
            (device_address <= TCA9548A_ADDR_MAX));
}

/**
 * @brief  Hàm chuyển tiếp thao tác ghi HAL I2C cho driver AS5600/TCA9548A.
 * @param  device_address: Địa chỉ I2C 7-bit của thiết bị đích.
 * @param  reg: Địa chỉ thanh ghi của thiết bị có register map.
 * @param  buffer: Con trỏ tới dữ liệu cần truyền.
 * @param  length: Số byte cần truyền.
 * @return 0 nếu thành công, -1 nếu HAL I2C báo lỗi.
 * @note   Lệnh ghi điều khiển TCA9548A không dùng địa chỉ thanh ghi.
 */
static int8_t my_app_i2c_write(uint8_t device_address,
                               uint8_t reg,
                               uint8_t *buffer,
                               uint16_t length)
{
    HAL_StatusTypeDef hal_status;

    /*
     * TCA9548A chỉ nhận một byte mask chọn kênh, không có địa chỉ thanh ghi.
     * AS5600 thì đọc/ghi theo register map nên phải dùng HAL_I2C_Mem_Write().
     */
    if (my_app_is_tca9548a_address(device_address) == true) {
        (void)reg;
        hal_status = HAL_I2C_Master_Transmit(&hi2c1,
                                             (uint16_t)(device_address << 1),
                                             buffer,
                                             length,
                                             MY_APP_I2C_TIMEOUT_MS);
    } else {
        hal_status = HAL_I2C_Mem_Write(&hi2c1,
                                       (uint16_t)(device_address << 1),
                                       reg,
                                       I2C_MEMADD_SIZE_8BIT,
                                       buffer,
                                       length,
                                       MY_APP_I2C_TIMEOUT_MS);
    }

    return (hal_status == HAL_OK) ? 0 : -1;
}

/**
 * @brief  Hàm chuyển tiếp thao tác đọc HAL I2C cho driver AS5600/TCA9548A.
 * @param  device_address: Địa chỉ I2C 7-bit của thiết bị đích.
 * @param  reg: Địa chỉ thanh ghi của thiết bị có register map.
 * @param  buffer: Con trỏ tới bộ đệm nhận dữ liệu.
 * @param  length: Số byte cần nhận.
 * @return 0 nếu thành công, -1 nếu HAL I2C báo lỗi.
 * @note   Lệnh đọc điều khiển TCA9548A không dùng địa chỉ thanh ghi.
 */
static int8_t my_app_i2c_read(uint8_t device_address,
                              uint8_t reg,
                              uint8_t *buffer,
                              uint16_t length)
{
    HAL_StatusTypeDef hal_status;

    /*
     * Đọc TCA9548A là đọc trực tiếp thanh ghi điều khiển hiện tại.
     * Các cảm biến phía sau mux vẫn dùng truy cập bộ nhớ theo thanh ghi.
     */
    if (my_app_is_tca9548a_address(device_address) == true) {
        (void)reg;
        hal_status = HAL_I2C_Master_Receive(&hi2c1,
                                            (uint16_t)(device_address << 1),
                                            buffer,
                                            length,
                                            MY_APP_I2C_TIMEOUT_MS);
    } else {
        hal_status = HAL_I2C_Mem_Read(&hi2c1,
                                      (uint16_t)(device_address << 1),
                                      reg,
                                      I2C_MEMADD_SIZE_8BIT,
                                      buffer,
                                      length,
                                      MY_APP_I2C_TIMEOUT_MS);
    }

    return (hal_status == HAL_OK) ? 0 : -1;
}

/**
 * @brief  Hàm gọi lại delay truyền cho driver TCA9548A/AS5600.
 * @param  delay_ms: Thời gian delay tính bằng mili giây.
 */
static void my_app_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

/**
 * @brief  Kiểm tra USB CDC đã cấu hình và sẵn sàng truyền gói mới chưa.
 * @return true nếu USB CDC có thể truyền, ngược lại false.
 */
static bool my_app_usb_is_ready(void)
{
    USBD_CDC_HandleTypeDef *cdc_handle;

    // USB phải được host cấu hình xong trước khi gọi CDC_Transmit_FS().
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        return false;
    }

    cdc_handle = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    // TxState khác 0 nghĩa là gói trước vẫn đang được USB stack xử lý.
    if ((cdc_handle == NULL) || (cdc_handle->TxState != 0u)) {
        return false;
    }

    return true;
}

/**
 * @brief  Gửi chuỗi kết thúc null qua USB CDC với giới hạn độ dài.
 * @param  text: Chuỗi kết thúc null cần gửi.
 */
static void my_app_usb_send_text(const char *text)
{
    uint16_t text_length = 0u;

    if ((text == NULL) || (my_app_usb_is_ready() == false)) {
        return;
    }

    // Tự giới hạn chiều dài để không vượt quá buffer truyền USB dùng chung.
    while ((text[text_length] != '\0') &&
           (text_length < (MY_APP_USB_TX_BUFFER_SIZE - 1u))) {
        text_length++;
    }

    if (text_length > 0u) {
        (void)CDC_Transmit_FS((uint8_t *)text, text_length);
    }
}

/**
 * @brief  Nhận diện lệnh "zero" không phân biệt hoa thường sau khi bỏ khoảng trắng.
 * @param  buffer: Các byte lệnh nhận được.
 * @param  length: Số byte trong buffer.
 * @return true nếu lệnh yêu cầu đặt lại yaw zero, ngược lại false.
 */
static bool my_app_is_zero_command(const uint8_t *buffer, uint16_t length)
{
    uint16_t start_index = 0u;
    uint16_t end_index = length;

    // Bỏ khoảng trắng đầu lệnh để chấp nhận " zero" hoặc "\tzero".
    while ((start_index < length) &&
           ((buffer[start_index] == ' ') ||
            (buffer[start_index] == '\t') ||
            (buffer[start_index] == '\r') ||
            (buffer[start_index] == '\n'))) {
        start_index++;
    }

    // Bỏ CR/LF và khoảng trắng cuối dòng do terminal hoặc CDC gửi kèm.
    while ((end_index > start_index) &&
           ((buffer[end_index - 1u] == ' ') ||
            (buffer[end_index - 1u] == '\t') ||
            (buffer[end_index - 1u] == '\r') ||
            (buffer[end_index - 1u] == '\n'))) {
        end_index--;
    }

    if ((end_index - start_index) != 4u) {
        return false;
    }

    return (((buffer[start_index] == 'z') || (buffer[start_index] == 'Z')) &&
            ((buffer[start_index + 1u] == 'e') || (buffer[start_index + 1u] == 'E')) &&
            ((buffer[start_index + 2u] == 'r') || (buffer[start_index + 2u] == 'R')) &&
            ((buffer[start_index + 3u] == 'o') || (buffer[start_index + 3u] == 'O')));
}

/**
 * @brief  Phân tích lệnh yaw dạng số thập phân sang centi-độ.
 * @param  buffer: Các byte lệnh nhận được.
 * @param  length: Số byte trong buffer.
 * @param  target_angle_cdeg: Góc sau khi phân tích, tính bằng centi-độ.
 * @return true nếu lệnh là góc thập phân hợp lệ, ngược lại false.
 * @note   Chỉ giữ hai chữ số thập phân để khớp với cách lưu centi-độ.
 */
static bool my_app_parse_target_angle_cdeg(const uint8_t *buffer,
                                           uint16_t length,
                                           int32_t *target_angle_cdeg)
{
    uint16_t index = 0u;
    int32_t whole_deg = 0;
    int32_t frac_cdeg = 0;
    uint8_t frac_digits = 0u;
    bool has_digit = false;

    if ((buffer == NULL) || (target_angle_cdeg == NULL) || (length == 0u)) {
        return false;
    }

    while ((index < length) && ((buffer[index] == ' ') || (buffer[index] == '\t'))) {
        index++;
    }

    // Phần nguyên được nhập theo đơn vị độ, ví dụ "123".
    while ((index < length) && (buffer[index] >= '0') && (buffer[index] <= '9')) {
        has_digit = true;
        whole_deg = (whole_deg * 10) + (int32_t)(buffer[index] - '0');
        index++;
    }

    /*
     * Phần thập phân chỉ lấy tối đa hai chữ số vì module đang lưu góc theo
     * centi-độ. Ví dụ 12.3 trở thành 12.30 độ, 12.345 bị xem là không hợp lệ
     * vì còn dư ký tự sau khi parse.
     */
    if ((index < length) && (buffer[index] == '.')) {
        index++;
        while ((index < length) &&
               (buffer[index] >= '0') &&
               (buffer[index] <= '9') &&
               (frac_digits < 2u)) {
            has_digit = true;
            frac_cdeg = (frac_cdeg * 10) + (int32_t)(buffer[index] - '0');
            frac_digits++;
            index++;
        }
    }

    if (frac_digits == 1u) {
        frac_cdeg *= 10;
    }

    // Sau giá trị góc chỉ cho phép khoảng trắng hoặc CR/LF.
    while ((index < length) &&
           ((buffer[index] == ' ') ||
            (buffer[index] == '\t') ||
            (buffer[index] == '\r') ||
            (buffer[index] == '\n'))) {
        index++;
    }

    if ((has_digit == false) || (index != length)) {
        return false;
    }

    *target_angle_cdeg = (whole_deg * 100) + frac_cdeg;
    return true;
}

/**
 * @brief  Đưa bộ lọc Kalman về trạng thái chưa có mẫu đo.
 * @param  filter: Con trỏ tới trạng thái bộ lọc cần reset.
 * @note   Reset giúp lần đọc đầu tiên sau khi init bám trực tiếp vào cảm biến,
 *         tránh dùng lại ước lượng cũ từ một phiên chạy trước.
 */
static void my_app_kalman_reset(my_app_kalman_filter_t *filter)
{
    if (filter == NULL) {
        return;
    }

    filter->estimate_cdeg = 0.0f;
    filter->error_covariance = MY_APP_KALMAN_INITIAL_COVARIANCE;
    filter->process_noise = MY_APP_KALMAN_PROCESS_NOISE;
    filter->measurement_noise = MY_APP_KALMAN_MEASUREMENT_NOISE;
    filter->is_initialized = false;
}

/**
 * @brief  Chuẩn hóa góc tuyệt đối về miền 0.00 tới nhỏ hơn 360.00 độ.
 * @param  angle_cdeg: Góc cần chuẩn hóa, tính bằng centi-độ.
 * @return Góc đã chuẩn hóa, tính bằng centi-độ.
 */
static int32_t my_app_normalize_absolute_cdeg(int32_t angle_cdeg)
{
    while (angle_cdeg < 0) {
        angle_cdeg += MY_APP_FULL_TURN_CDEG;
    }

    while (angle_cdeg >= MY_APP_FULL_TURN_CDEG) {
        angle_cdeg -= MY_APP_FULL_TURN_CDEG;
    }

    return angle_cdeg;
}

/**
 * @brief  Tính sai lệch góc ngắn nhất giữa ước lượng và mẫu đo mới.
 * @param  reference_cdeg: Góc ước lượng hiện tại, tính bằng centi-độ.
 * @param  sample_cdeg: Mẫu đo mới đã chuẩn hóa, tính bằng centi-độ.
 * @return Sai lệch có dấu trong khoảng -180.00 tới +180.00 độ.
 * @note   Cách tính này giữ Kalman ổn định khi AS5600 đi qua mốc 0/360 độ.
 */
#if (MY_APP_USE_KALMAN_FILTER != 0)
static float my_app_calculate_shortest_angle_error_cdeg(float reference_cdeg,
                                                        int32_t sample_cdeg)
{
    float error_cdeg = (float)sample_cdeg - reference_cdeg;

    while (error_cdeg > (float)MY_APP_HALF_TURN_CDEG) {
        error_cdeg -= (float)MY_APP_FULL_TURN_CDEG;
    }

    while (error_cdeg < (float)-MY_APP_HALF_TURN_CDEG) {
        error_cdeg += (float)MY_APP_FULL_TURN_CDEG;
    }

    return error_cdeg;
}

/**
 * @brief  Cập nhật bộ lọc Kalman từ một mẫu góc tuyệt đối AS5600.
 * @param  filter: Con trỏ tới trạng thái bộ lọc Kalman.
 * @param  sample_cdeg: Mẫu đo góc tuyệt đối, tính bằng centi-độ.
 * @return Góc tuyệt đối đã lọc, tính bằng centi-độ.
 */
static int32_t my_app_kalman_update(my_app_kalman_filter_t *filter,
                                    int32_t sample_cdeg)
{
    float kalman_gain;
    float innovation_cdeg;

    sample_cdeg = my_app_normalize_absolute_cdeg(sample_cdeg);

    if (filter == NULL) {
        return sample_cdeg;
    }

    if (filter->is_initialized == false) {
        filter->estimate_cdeg = (float)sample_cdeg;
        filter->error_covariance = MY_APP_KALMAN_INITIAL_COVARIANCE;
        filter->is_initialized = true;
        return sample_cdeg;
    }

    filter->error_covariance += filter->process_noise;
    innovation_cdeg = my_app_calculate_shortest_angle_error_cdeg(
        filter->estimate_cdeg,
        sample_cdeg);

    /*
     * Khi motor vừa chạy xong, góc thật có thể đổi lớn giữa hai lần lấy mẫu.
     * Trường hợp này cần bám nhanh vào mẫu mới để Kalman không tạo sai số trễ.
     */
    if ((innovation_cdeg > MY_APP_KALMAN_FAST_TRACK_CDEG) ||
        (innovation_cdeg < -MY_APP_KALMAN_FAST_TRACK_CDEG)) {
        filter->estimate_cdeg = (float)sample_cdeg;
        filter->error_covariance = MY_APP_KALMAN_INITIAL_COVARIANCE;
        return sample_cdeg;
    }

    kalman_gain = filter->error_covariance /
                  (filter->error_covariance + filter->measurement_noise);
    filter->estimate_cdeg += kalman_gain * innovation_cdeg;
    filter->error_covariance *= (1.0f - kalman_gain);

    while (filter->estimate_cdeg < 0.0f) {
        filter->estimate_cdeg += (float)MY_APP_FULL_TURN_CDEG;
    }

    while (filter->estimate_cdeg >= (float)MY_APP_FULL_TURN_CDEG) {
        filter->estimate_cdeg -= (float)MY_APP_FULL_TURN_CDEG;
    }

    return (int32_t)(filter->estimate_cdeg + 0.5f);
}
#endif

/**
 * @brief  Chuẩn hóa góc tuyệt đối AS5600 thành yaw so với zero phần mềm.
 * @param  sensor_angle_cdeg: Góc cảm biến tuyệt đối, tính bằng centi-độ.
 * @return Góc yaw trong khoảng từ 0.00 tới nhỏ hơn 360.00 độ.
 */
static int32_t my_app_normalize_sensor_yaw_cdeg(int32_t sensor_angle_cdeg)
{
    int32_t yaw_cdeg = sensor_angle_cdeg - s_sensor_zero_cdeg;

    // Đưa góc âm quay vòng về miền 0..360 độ.
    while (yaw_cdeg < 0) {
        yaw_cdeg += MY_APP_FULL_TURN_CDEG;
    }

    // Đưa góc vượt một vòng quay về miền 0..360 độ.
    while (yaw_cdeg >= MY_APP_FULL_TURN_CDEG) {
        yaw_cdeg -= MY_APP_FULL_TURN_CDEG;
    }

    return yaw_cdeg;
}

/**
 * @brief  Đọc góc tuyệt đối đã lọc của AS5600 theo centi-độ.
 * @param  sensor_angle_cdeg: Góc tuyệt đối đầu ra, tính bằng centi-độ.
 * @return true nếu đọc cảm biến thành công, ngược lại false.
 * @note   Giá trị raw 12-bit của AS5600 được scale sang centi-độ để module
 *         này xử lý bằng số nguyên.
 */
static bool my_app_read_sensor_absolute_cdeg(int32_t *sensor_angle_cdeg)
{
    int32_t raw_angle_cdeg;

    if (sensor_angle_cdeg == NULL) {
        return false;
    }

    if (TCA9548A_ReadSensor(&s_mux, MY_APP_SENSOR_CHANNEL, &s_as5600_data) != TCA9548A_OK) {
        return false;
    }

    /*
     * AS5600 trả về 12-bit raw 0..4095. Nhân trước rồi chia sau để giữ độ
     * phân giải khi chuyển sang centi-độ bằng số nguyên.
     */
    raw_angle_cdeg = (int32_t)(((uint64_t)s_as5600_data.angle *
                                (uint64_t)MY_APP_FULL_TURN_CDEG) /
                               (uint64_t)MY_APP_AS5600_RAW_STEPS);

#if (MY_APP_USE_KALMAN_FILTER != 0)
    *sensor_angle_cdeg = my_app_kalman_update(&s_sensor_kalman_filter,
                                              raw_angle_cdeg);
#else
    *sensor_angle_cdeg = my_app_normalize_absolute_cdeg(raw_angle_cdeg);
#endif
    return true;
}

/**
 * @brief  Đọc góc yaw hiện tại so với zero phần mềm.
 * @param  sensor_yaw_cdeg: Góc yaw đầu ra, tính bằng centi-độ.
 * @return true nếu đọc cảm biến thành công, ngược lại false.
 */
static bool my_app_read_sensor_yaw_cdeg(int32_t *sensor_yaw_cdeg)
{
    int32_t sensor_angle_cdeg = 0;

    if (sensor_yaw_cdeg == NULL) {
        return false;
    }

    if (my_app_read_sensor_absolute_cdeg(&sensor_angle_cdeg) == false) {
        return false;
    }

    *sensor_yaw_cdeg = my_app_normalize_sensor_yaw_cdeg(sensor_angle_cdeg);
    return true;
}

/**
 * @brief  Định dạng góc centi-độ thành chuỗi độ có cố định hai chữ số lẻ.
 * @param  angle_cdeg: Góc tính bằng centi-độ.
 * @param  buffer: Bộ đệm chuỗi đích.
 * @param  buffer_size: Kích thước bộ đệm đích, tính bằng byte.
 * @note   Định dạng bằng số nguyên để không cần bật hỗ trợ printf số thực.
 */
static void my_app_format_angle_deg(int32_t angle_cdeg,
                                    char *buffer,
                                    uint16_t buffer_size)
{
    uint32_t angle_abs_cdeg;
    uint32_t angle_whole_deg;
    uint32_t angle_frac_cdeg;

    if ((buffer == NULL) || (buffer_size == 0u)) {
        return;
    }

    angle_abs_cdeg = (angle_cdeg < 0) ? (uint32_t)(-angle_cdeg) : (uint32_t)angle_cdeg;

    // Tách centi-độ thành phần nguyên và hai chữ số thập phân để in xx.yy.
    angle_whole_deg = angle_abs_cdeg / 100u;
    angle_frac_cdeg = angle_abs_cdeg % 100u;

    if (angle_cdeg < 0) {
        (void)snprintf(buffer,
                       buffer_size,
                       "-%lu.%02lu",
                       (unsigned long)angle_whole_deg,
                       (unsigned long)angle_frac_cdeg);
    } else {
        (void)snprintf(buffer,
                       buffer_size,
                       "%lu.%02lu",
                       (unsigned long)angle_whole_deg,
                       (unsigned long)angle_frac_cdeg);
    }
}

/**
 * @brief  Chuyển delta chuyển động có dấu sang hướng quay motor.
 * @param  delta_cdeg: Góc chạy tương đối, tính bằng centi-độ.
 * @return Hướng thuận chiều kim đồng hồ nếu delta không âm, ngược lại hướng
 *         ngược chiều kim đồng hồ.
 */
static TMC2209_DirectionTypeDef my_app_get_direction_from_delta(int32_t delta_cdeg)
{
    return (delta_cdeg >= 0) ? TMC2209_DIR_CW : TMC2209_DIR_CCW;
}

/**
 * @brief  Tính quãng góc cảm biến đã đo, có xét hướng khi đi qua điểm 0.
 * @param  start_cdeg: Góc yaw cảm biến ban đầu, tính bằng centi-độ.
 * @param  end_cdeg: Góc yaw cảm biến cuối, tính bằng centi-độ.
 * @param  expected_delta_cdeg: Độ lớn và hướng chạy đã ra lệnh.
 * @return Delta cảm biến có dấu, tính bằng centi-độ.
 */
static int32_t my_app_calculate_sensor_delta_cdeg(int32_t start_cdeg,
                                                  int32_t end_cdeg,
                                                  int32_t expected_delta_cdeg)
{
    int32_t sensor_delta_cdeg = end_cdeg - start_cdeg;

    /*
     * Nếu lệnh chạy chiều dương nhưng cảm biến đi qua mốc 360 -> 0, delta thô
     * sẽ âm. Cộng một vòng để giữ đúng chiều đo.
     */
    if ((expected_delta_cdeg >= 0) && (sensor_delta_cdeg < 0)) {
        sensor_delta_cdeg += MY_APP_FULL_TURN_CDEG;
    }

    /*
     * Nếu lệnh chạy chiều âm nhưng cảm biến đi qua mốc 0 -> 360, delta thô sẽ
     * dương. Trừ một vòng để giữ đúng chiều đo.
     */
    if ((expected_delta_cdeg < 0) && (sensor_delta_cdeg > 0)) {
        sensor_delta_cdeg -= MY_APP_FULL_TURN_CDEG;
    }

    return sensor_delta_cdeg;
}

/**
 * @brief  Đặt góc tuyệt đối AS5600 hiện tại làm yaw zero phần mềm.
 * @return true nếu cập nhật zero thành công, ngược lại false.
 */
static bool my_app_set_zero_from_sensor(void)
{
    int32_t sensor_angle_cdeg = 0;

    if (my_app_read_sensor_absolute_cdeg(&sensor_angle_cdeg) == false) {
        my_app_usb_send_text("ERR: cannot read sensor for zero\r\n");
        return false;
    }

    // Zero chỉ là mốc phần mềm, không ghi vào OTP hoặc register của AS5600.
    s_sensor_zero_cdeg = sensor_angle_cdeg;
    s_current_yaw_cdeg = 0;
    s_app_state = MY_APP_STATE_WAIT_COMMAND;
    my_app_usb_send_text("zero_ok yaw_deg=0.00\r\n");
    return true;
}

/**
 * @brief  Bắt đầu chạy motor từ yaw hiện tại tới yaw mục tiêu.
 * @param  target_yaw_cdeg: Góc yaw mục tiêu tuyệt đối, tính bằng centi-độ.
 * @return true nếu lệnh chạy được chấp nhận, ngược lại false.
 */
static bool my_app_start_target_move(int32_t target_yaw_cdeg)
{
    TMC2209_StatusTypeDef motor_status;
    int32_t move_delta_cdeg;
    uint32_t move_steps;

    if (my_app_read_sensor_yaw_cdeg(&s_move_context.start_sensor_yaw_cdeg) == false) {
        my_app_usb_send_text("ERR: cannot read start angle\r\n");
        return false;
    }

    // target_yaw_cdeg là góc tuyệt đối, motor chỉ cần chạy phần delta.
    move_delta_cdeg = target_yaw_cdeg - s_current_yaw_cdeg;
    move_steps = my_app_calculate_motor_steps_from_angle(move_delta_cdeg);

    // Lưu lại thông tin này để so sánh với cảm biến khi motor chạy xong.
    s_move_context.target_yaw_cdeg = target_yaw_cdeg;
    s_move_context.start_yaw_cdeg = s_current_yaw_cdeg;
    s_move_context.move_delta_cdeg = move_delta_cdeg;
    s_move_context.target_steps = move_steps;

    if (move_steps == 0u) {
        // Vẫn chuyển state để tạo một report nhất quán cho lệnh không cần chạy.
        s_app_state = MY_APP_STATE_WAIT_MOTOR;
        return true;
    }

    motor_status = TMC2209_MoveSteps(&motor1,
                                     move_steps,
                                     my_app_get_direction_from_delta(move_delta_cdeg),
                                     MY_APP_MOTOR_SPEED_RPM);
    if (motor_status != TMC2209_OK) {
        my_app_usb_send_text("ERR: motor start failed\r\n");
        return false;
    }

    s_app_state = MY_APP_STATE_WAIT_MOTOR;
    return true;
}

/**
 * @brief  Xử lý một lệnh USB CDC đang chờ khi ứng dụng ở trạng thái rảnh.
 * @note   Lệnh hợp lệ là "zero" hoặc một góc mục tiêu tuyệt đối theo độ.
 */
static void my_app_process_usb_command(void)
{
    uint8_t command_buffer[MY_APP_USB_RX_BUFFER_SIZE];
    uint16_t command_length = 0u;
    int32_t target_yaw_cdeg = 0;

    if (CDC_ReadCommand(command_buffer, sizeof(command_buffer), &command_length) == 0u) {
        return;
    }

    // Lệnh zero được ưu tiên vì không phải là một giá trị góc số.
    if (my_app_is_zero_command(command_buffer, command_length) == true) {
        (void)my_app_set_zero_from_sensor();
        return;
    }

    if (my_app_parse_target_angle_cdeg(command_buffer,
                                       command_length,
                                       &target_yaw_cdeg) == false) {
        my_app_usb_send_text("ERR: input must be zero or angle 0.00..360.00 deg\r\n");
        return;
    }

    if ((target_yaw_cdeg < 0) || (target_yaw_cdeg > MY_APP_FULL_TURN_CDEG)) {
        my_app_usb_send_text("ERR: input range is 0.00..360.00 deg\r\n");
        return;
    }

    (void)my_app_start_target_move(target_yaw_cdeg);
}

/**
 * @brief  Hoàn tất một lần chạy motor và báo sai số cảm biến qua USB CDC.
 * @note   Công cụ ghi CSV cần đúng tên các trường report được phát ra ở đây.
 */
static void my_app_process_motor_done(void)
{
    int32_t end_sensor_yaw_cdeg = 0;
    int32_t sensor_delta_cdeg;
    int32_t error_cdeg;
    int32_t report_length;
    char input_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char start_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char end_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char delta_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char error_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];

    if (motor1.state == TMC2209_RUNNING) {
        return;
    }

    // Chỉ đọc góc cuối sau khi TMC2209 đã hoàn tất số bước được yêu cầu.
    if (my_app_read_sensor_yaw_cdeg(&end_sensor_yaw_cdeg) == false) {
        my_app_usb_send_text("ERR: cannot read end angle\r\n");
        s_app_state = MY_APP_STATE_WAIT_COMMAND;
        return;
    }

    sensor_delta_cdeg = my_app_calculate_sensor_delta_cdeg(
        s_move_context.start_sensor_yaw_cdeg,
        end_sensor_yaw_cdeg,
        s_move_context.move_delta_cdeg);
    error_cdeg = s_move_context.move_delta_cdeg - sensor_delta_cdeg;

    // Cập nhật yaw phần mềm theo lệnh đã chấp nhận, còn sai số được log riêng.
    s_current_yaw_cdeg = s_move_context.target_yaw_cdeg;

    my_app_format_angle_deg(s_move_context.target_yaw_cdeg,
                            input_angle_text,
                            sizeof(input_angle_text));
    my_app_format_angle_deg(s_move_context.start_sensor_yaw_cdeg,
                            start_angle_text,
                            sizeof(start_angle_text));
    my_app_format_angle_deg(end_sensor_yaw_cdeg,
                            end_angle_text,
                            sizeof(end_angle_text));
    my_app_format_angle_deg(sensor_delta_cdeg,
                            delta_angle_text,
                            sizeof(delta_angle_text));
    my_app_format_angle_deg(error_cdeg,
                            error_angle_text,
                            sizeof(error_angle_text));

    report_length = snprintf(s_usb_tx_buffer,
                             sizeof(s_usb_tx_buffer),
                             "input_deg=%s,start_deg=%s,end_deg=%s,"
                             "delta_deg=%s,error_deg=%s,steps=%lu\r\n",
                             input_angle_text,
                             start_angle_text,
                             end_angle_text,
                             delta_angle_text,
                             error_angle_text,
                             (unsigned long)s_move_context.target_steps);

    // Format report này đang được read_uart.py dùng để ghi CSV.
    if ((report_length > 0) && (report_length < (int32_t)sizeof(s_usb_tx_buffer))) {
        (void)CDC_Transmit_FS((uint8_t *)s_usb_tx_buffer, (uint16_t)report_length);
    }

    s_app_state = MY_APP_STATE_WAIT_COMMAND;
}

/**
 * @brief  Cấu hình handle TMC2209 với timer, GPIO và địa chỉ driver.
 */
void motors_config(void)
{
    motor1.id             = 0u;
    motor1.htim           = &htim2;
    motor1.tim_channel    = TIM_CHANNEL_1;
    motor1.dir_port       = DIR_1_GPIO_Port;
    motor1.dir_pin        = DIR_1_Pin;
    motor1.en_port        = EN_1_GPIO_Port;
    motor1.en_pin         = EN_1_Pin;
    motor1.timer_clock_hz = TMC2209_TIMER_CLOCK_HZ;
    motor1.prescaler      = TMC2209_TIMER_PRESCALER;
    motor1.steps_per_rev  = TMC2209_MOTOR_STEPS_REV;
    motor1.microstep      = TMC2209_MICROSTEP_16;
    motor1.slave_address  = TMC2209_SLAVE_ADD1;

    motor2.id             = 1u;
    motor2.htim           = &htim3;
    motor2.tim_channel    = TIM_CHANNEL_1;
    motor2.dir_port       = DIR_2_GPIO_Port;
    motor2.dir_pin        = DIR_2_Pin;
    motor2.en_port        = EN_2_GPIO_Port;
    motor2.en_pin         = EN_2_Pin;
    motor2.timer_clock_hz = TMC2209_TIMER_CLOCK_HZ;
    motor2.prescaler      = TMC2209_TIMER_PRESCALER;
    motor2.steps_per_rev  = TMC2209_MOTOR_STEPS_REV;
    motor2.microstep      = TMC2209_MICROSTEP_16;
    motor2.slave_address  = TMC2209_SLAVE_ADD2;

    motor3.id             = 2u;
    motor3.htim           = &htim4;
    motor3.tim_channel    = TIM_CHANNEL_1;
    motor3.dir_port       = DIR_3_GPIO_Port;
    motor3.dir_pin        = DIR_3_Pin;
    motor3.en_port        = EN_3_GPIO_Port;
    motor3.en_pin         = EN_3_Pin;
    motor3.timer_clock_hz = TMC2209_TIMER_CLOCK_HZ;
    motor3.prescaler      = TMC2209_TIMER_PRESCALER;
    motor3.steps_per_rev  = TMC2209_MOTOR_STEPS_REV;
    motor3.microstep      = TMC2209_MICROSTEP_16;
    motor3.slave_address  = TMC2209_SLAVE_ADD3;
}

/**
 * @brief  Khởi tạo tất cả driver TMC2209 đã cấu hình và bật StealthChop.
 */
void motors_init(void)
{
    motors_config();

    (void)TMC2209_Init(&motor1);
    (void)TMC2209_Init(&motor2);
    (void)TMC2209_Init(&motor3);

    (void)TMC2209_SetStealthChop(&motor1, true);
    (void)TMC2209_SetStealthChop(&motor2, true);
    (void)TMC2209_SetStealthChop(&motor3, true);
}

/**
 * @brief  Khởi tạo motor, mux cảm biến, AS5600, trạng thái lệnh USB và yaw zero.
 */
void my_app_init(void)
{
    motors_init();

    /*
     * Driver TCA9548A và AS5600 gọi ngược lại các hàm adapter bên dưới để dùng
     * chung bus I2C1 do CubeMX sinh ra.
     */
    s_mux.i2c_write = my_app_i2c_write;
    s_mux.i2c_read = my_app_i2c_read;
    s_mux.delay_ms = my_app_delay_ms;

    // Reset trạng thái runtime trước khi thử khởi tạo đường cảm biến.
    s_is_sensor_ready = false;
    s_has_init_error_been_reported = false;
    s_app_state = MY_APP_STATE_WAIT_COMMAND;
    s_sensor_zero_cdeg = 0;
    s_current_yaw_cdeg = 0;
    my_app_kalman_reset(&s_sensor_kalman_filter);

    if (TCA9548A_Init(&s_mux, TCA9548A_ADDR_A000) != TCA9548A_OK) {
        return;
    }

    if (TCA9548A_RegisterSensor(&s_mux, MY_APP_SENSOR_CHANNEL, "AS5600_CH0") != TCA9548A_OK) {
        return;
    }

    if (TCA9548A_InitAllSensors(&s_mux) != TCA9548A_OK) {
        return;
    }

    // Chỉ cho phép xử lý lệnh sau khi mux và AS5600 đã khởi tạo thành công.
    s_is_sensor_ready = true;
    (void)my_app_set_zero_from_sensor();
}

/**
 * @brief  Chuyển delta yaw tương đối sang số microstep của motor.
 * @param  angle_cdeg: Delta yaw tương đối, tính bằng centi-độ.
 * @return Số microstep đã làm tròn cần dùng cho chuyển động.
 */
uint32_t my_app_calculate_motor_steps_from_angle(int32_t angle_cdeg)
{
    uint32_t angle_abs_cdeg;
    uint32_t scaled_steps_per_turn;

    angle_abs_cdeg = (angle_cdeg < 0) ? (uint32_t)(-angle_cdeg) : (uint32_t)angle_cdeg;

    // Số microstep trên một vòng phụ thuộc cấu hình full-step và microstep.
    scaled_steps_per_turn = (uint32_t)((uint64_t)motor1.steps_per_rev *
                                       (uint64_t)motor1.microstep);

    // Cộng nửa mẫu số để làm tròn gần nhất thay vì luôn làm tròn xuống.
    return (uint32_t)(((uint64_t)angle_abs_cdeg * scaled_steps_per_turn +
                       ((uint64_t)MY_APP_FULL_TURN_CDEG / 2u)) /
                      (uint64_t)MY_APP_FULL_TURN_CDEG);
}

/**
 * @brief  Chạy máy trạng thái ứng dụng theo kiểu không chặn.
 * @note   Gọi hàm này lặp lại trong vòng lặp main.
 */
void my_app_process(void)
{
    if (my_app_usb_is_ready() == false) {
        return;
    }

    if (s_is_sensor_ready == false) {
        if (s_has_init_error_been_reported == false) {
            // Báo lỗi init một lần để terminal không bị spam liên tục.
            my_app_usb_send_text("ERR: AS5600 init failed\r\n");
            s_has_init_error_been_reported = true;
        }
        return;
    }

    if (s_app_state == MY_APP_STATE_WAIT_COMMAND) {
        my_app_process_usb_command();
        return;
    }

    my_app_process_motor_done();
}

/**
 * @brief  Hàm gọi lại khi hoàn tất xung PWM timer để đếm bước motor.
 * @param  htim: Handle HAL timer phát sinh callback.
 * @note   Giữ callback ISR này ngắn, chỉ chuyển tiếp việc cập nhật bước.
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == motor1.htim) {
        // ISR callback chỉ cập nhật bộ đếm bước, không làm việc blocking.
        (void)TMC2209_UpdateSteps(&motor1);
    }
}
