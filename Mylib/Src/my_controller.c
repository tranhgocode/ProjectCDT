/**
 * @file    my_controller.c
 * @brief   Điều khiển motor yaw TMC2209 và đọc phản hồi góc từ AS5600.
 * @author  Lap4all
 * @date    2026-05-14
 */

#include "my_controller.h"
#include "as5600.h"
#include "i2c.h"
#include "my_app.h"
#include "my_config.h"
#include "tca9548a.h"
#include "tim.h"
#include <stdbool.h>
#include <stdint.h>

TMC2209_HandleTypeDef motor1; /**< Driver TMC2209 thứ nhất. */
TMC2209_HandleTypeDef motor2; /**< Driver TMC2209 thứ hai. */
TMC2209_HandleTypeDef motor3; /**< Driver TMC2209 thứ ba. */

/** @brief Kênh AS5600 thứ nhất, dùng làm phản hồi yaw hiện có. */
#define MY_CONTROLLER_SENSOR1_CHANNEL           TCA9548A_CH0

/** @brief Kênh AS5600 thứ hai trên cặp SD1/SC1 của TCA9548A. */
#define MY_CONTROLLER_SENSOR2_CHANNEL           TCA9548A_CH1

/** @brief Kênh AS5600 thứ ba trên cặp SD2/SC2 của TCA9548A. */
#define MY_CONTROLLER_SENSOR3_CHANNEL           TCA9548A_CH2

/** @brief Timeout giao dịch I2C, tính bằng mili giây. */
#define MY_CONTROLLER_I2C_TIMEOUT_MS            20U

/** @brief Số vị trí raw AS5600 trong một vòng cơ khí. */
#define MY_CONTROLLER_AS5600_RAW_STEPS          4096U

/** @brief Nửa vòng quay, dùng để chọn sai lệch góc ngắn nhất. */
#define MY_CONTROLLER_HALF_TURN_CDEG            18000L

/** @brief Tan so STEP mac dinh cho motor yaw, tuong duong 60 RPM voi 200 step/vong va 16 microstep. */
#define MY_CONTROLLER_MOTOR_SPEED_HZ            MY_APP_MOTOR_YAW_SPEED_HZ

/** @brief Tan so STEP mac dinh cho lenh chay dong thoi ba motor. */
#define MY_CONTROLLER_THREE_MOTOR_SPEED_HZ      MY_APP_THREE_MOTOR_SPEED_HZ

/** @brief Chiều thuận motor1 khi góc mục tiêu lớn hơn góc hiện tại. */
#define MY_CONTROLLER_MOTOR1_FORWARD_DIR         ((TMC2209_DirectionTypeDef)MY_APP_MOTOR1_FORWARD_DIR)

/** @brief Tử số scale bước motor1 theo tỉ số truyền cơ khí. */
#define MY_CONTROLLER_MOTOR1_STEP_SCALE_NUM      MY_APP_MOTOR1_STEP_SCALE_NUM

/** @brief Mẫu số scale bước motor1 theo tỉ số truyền cơ khí. */
#define MY_CONTROLLER_MOTOR1_STEP_SCALE_DEN      MY_APP_MOTOR1_STEP_SCALE_DEN

/** @brief Chiều thuận motor2 khi góc mục tiêu lớn hơn góc hiện tại. */
#define MY_CONTROLLER_MOTOR2_FORWARD_DIR         ((TMC2209_DirectionTypeDef)MY_APP_MOTOR2_FORWARD_DIR)

/** @brief Tử số scale bước motor2 theo tỉ số truyền cơ khí. */
#define MY_CONTROLLER_MOTOR2_STEP_SCALE_NUM      MY_APP_MOTOR2_STEP_SCALE_NUM

/** @brief Mẫu số scale bước motor2 theo tỉ số truyền cơ khí. */
#define MY_CONTROLLER_MOTOR2_STEP_SCALE_DEN      MY_APP_MOTOR2_STEP_SCALE_DEN

/** @brief Chiều thuận motor3 khi góc mục tiêu lớn hơn góc hiện tại. */
#define MY_CONTROLLER_MOTOR3_FORWARD_DIR         ((TMC2209_DirectionTypeDef)MY_APP_MOTOR3_FORWARD_DIR)

/** @brief Tử số scale bước motor3 theo tỉ số truyền cơ khí. */
#define MY_CONTROLLER_MOTOR3_STEP_SCALE_NUM      MY_APP_MOTOR3_STEP_SCALE_NUM

/** @brief Mẫu số scale bước motor3 theo tỉ số truyền cơ khí. */
#define MY_CONTROLLER_MOTOR3_STEP_SCALE_DEN      MY_APP_MOTOR3_STEP_SCALE_DEN

/** @brief Nhiễu quá trình giúp bộ lọc bám theo góc thật. */
#define MY_CONTROLLER_KALMAN_PROCESS_NOISE      MY_APP_KALMAN_PROCESS_NOISE

/** @brief Nhiễu đo giúp giảm rung góc đọc từ AS5600. */
#define MY_CONTROLLER_KALMAN_MEASUREMENT_NOISE  MY_APP_KALMAN_MEASUREMENT_NOISE

/** @brief Hiệp phương sai ban đầu của bộ lọc Kalman. */
#define MY_CONTROLLER_KALMAN_INITIAL_COVARIANCE MY_APP_KALMAN_INITIAL_COVARIANCE

/** @brief Ngưỡng bám nhanh khi góc thật đổi lớn. */
#define MY_CONTROLLER_KALMAN_FAST_TRACK_CDEG    MY_APP_KALMAN_FAST_TRACK_CDEG

/** @brief Đặt khác 0 để bật lọc Kalman. */
#define MY_CONTROLLER_USE_KALMAN_FILTER         MY_APP_SENSOR_FILTER_MODE

/**
 * @brief  Trạng thái bộ lọc Kalman 1 chiều cho góc tuyệt đối AS5600.
 */
typedef struct {
    float estimate_cdeg;       /**< Góc ước lượng hiện tại, centi-độ. */
    float error_covariance;    /**< Độ không chắc chắn của giá trị ước lượng. */
    float process_noise;       /**< Nhiễu mô hình giữa hai lần lấy mẫu. */
    float measurement_noise;   /**< Nhiễu đo của mẫu AS5600 đọc từ I2C. */
    bool is_initialized;       /**< Cờ cho biết bộ lọc đã nhận mẫu đầu tiên. */
} my_controller_kalman_filter_t;

/** @brief Mux TCA9548A truy cập AS5600 yaw. */
static TCA9548A_Handle_t s_mux;

/** @brief Mẫu AS5600 kênh 0 mới nhất đọc qua mux. */
static AS5600_Data_t s_sensor1_data;

/** @brief Mẫu AS5600 kênh 1 mới nhất đọc qua mux. */
static AS5600_Data_t s_sensor2_data;

/** @brief Mẫu AS5600 kênh 2 mới nhất đọc qua mux. */
static AS5600_Data_t s_sensor3_data;

/** @brief Góc tuyệt đối được chọn làm yaw zero. */
static int32_t s_sensor_zero_cdeg = 0;

static int32_t s_sensor1_zero_cdeg = 0;
static int32_t s_sensor2_zero_cdeg = 0;
static int32_t s_sensor3_zero_cdeg = 0;

/** @brief Góc yaw phần mềm sau lệnh gần nhất. */
static int32_t s_current_yaw_cdeg = 0;

/** @brief Vị trí phần mềm motor1, lấy gốc tại vị trí lúc khởi động. */
static int32_t s_motor1_current_cdeg = 0;

/** @brief Vị trí phần mềm motor2, lấy gốc tại vị trí lúc khởi động. */
static int32_t s_motor2_current_cdeg = 0;

/** @brief Vị trí phần mềm motor3, lấy gốc tại vị trí lúc khởi động. */
static int32_t s_motor3_current_cdeg = 0;

/** @brief Bộ lọc góc tuyệt đối AS5600. */
static my_controller_kalman_filter_t s_sensor_filter;

static bool prv_IsTca9548aAddress(uint8_t device_address);
static int8_t prv_I2cWrite(uint8_t device_address,
                           uint8_t reg,
                           uint8_t *buffer,
                           uint16_t length);
static int8_t prv_I2cRead(uint8_t device_address,
                          uint8_t reg,
                          uint8_t *buffer,
                          uint16_t length);
static void prv_DelayMs(uint32_t delay_ms);
static void prv_KalmanReset(my_controller_kalman_filter_t *filter);
static int32_t prv_NormalizeAbsoluteCdeg(int32_t angle_cdeg);
#if (MY_CONTROLLER_USE_KALMAN_FILTER == MY_APP_SENSOR_FILTER_KALMAN)
static float prv_CalculateShortestAngleErrorCdeg(float reference_cdeg,
                                                 int32_t sample_cdeg);
static int32_t prv_KalmanUpdate(my_controller_kalman_filter_t *filter,
                                int32_t sample_cdeg);
#elif (MY_CONTROLLER_USE_KALMAN_FILTER == MY_APP_SENSOR_FILTER_NONE)
#else
#error "Invalid MY_APP_SENSOR_FILTER_MODE setting"
#endif
static int32_t prv_NormalizeSensorYawCdeg(int32_t sensor_angle_cdeg);
static int32_t prv_ConvertSensorAngleCdeg(const AS5600_Data_t *data);
static TCA9548A_Status_t prv_ReadSensorAngleCdeg(
    TCA9548A_Channel_t channel,
    AS5600_Data_t *data,
    int32_t *angle_cdeg);
static int32_t prv_CalculateShortestThreeSensorDeltaCdeg(int32_t zero_cdeg,
                                                         int32_t angle_cdeg);
static int32_t prv_ScaleSensorDeltaCdeg(int32_t delta_cdeg,
                                        uint32_t numerator,
                                        uint32_t denominator);
static TMC2209_DirectionTypeDef prv_GetDirectionFromDelta(int32_t delta_cdeg);
static TMC2209_DirectionTypeDef prv_GetOppositeDirection(
    TMC2209_DirectionTypeDef direction);
static TMC2209_DirectionTypeDef prv_GetThreeMotorDirection(
    int32_t delta_cdeg,
    TMC2209_DirectionTypeDef forward_direction);
static uint32_t prv_CalculateMotorStepsFromCdeg(
    const TMC2209_HandleTypeDef *hmotor,
    int32_t angle_cdeg);
static uint32_t prv_ScaleMotorSteps(uint32_t steps,
                                    uint32_t numerator,
                                    uint32_t denominator);
static void prv_ResetThreeMotorOrigin(void);
static int32_t prv_CalculateSensorDeltaCdeg(int32_t start_cdeg,
                                            int32_t end_cdeg,
                                            int32_t expected_delta_cdeg);
static void prv_StopThreeMotors(void);

/**
 * @brief  Kiểm tra địa chỉ I2C có thuộc dải địa chỉ TCA9548A hay không.
 * @param  device_address: Địa chỉ I2C 7-bit.
 * @return true nếu là địa chỉ TCA9548A, ngược lại false.
 */
static bool prv_IsTca9548aAddress(uint8_t device_address)
{
    return ((device_address >= TCA9548A_BASE_ADDR) &&
            (device_address <= TCA9548A_ADDR_MAX));
}

/**
 * @brief  Chuyển tiếp thao tác ghi HAL I2C cho AS5600/TCA9548A.
 * @param  device_address: Địa chỉ I2C 7-bit của thiết bị đích.
 * @param  reg: Địa chỉ thanh ghi của thiết bị có register map.
 * @param  buffer: Con trỏ tới dữ liệu cần truyền.
 * @param  length: Số byte cần truyền.
 * @return 0 nếu thành công, -1 nếu HAL I2C báo lỗi.
 */
static int8_t prv_I2cWrite(uint8_t device_address,
                           uint8_t reg,
                           uint8_t *buffer,
                           uint16_t length)
{
    HAL_StatusTypeDef hal_status;

    /*
     * TCA9548A không có register map như AS5600, nên adapter phải chọn đúng
     * API HAL để cùng một callback phục vụ được cả mux và cảm biến phía sau.
     */
    if (prv_IsTca9548aAddress(device_address) == true) {
        (void)reg;
        hal_status = HAL_I2C_Master_Transmit(&hi2c1,
                                             (uint16_t)(device_address << 1),
                                             buffer,
                                             length,
                                             MY_CONTROLLER_I2C_TIMEOUT_MS);
    } else {
        hal_status = HAL_I2C_Mem_Write(&hi2c1,
                                       (uint16_t)(device_address << 1),
                                       reg,
                                       I2C_MEMADD_SIZE_8BIT,
                                       buffer,
                                       length,
                                       MY_CONTROLLER_I2C_TIMEOUT_MS);
    }

    return (hal_status == HAL_OK) ? 0 : -1;
}

/**
 * @brief  Chuyển tiếp thao tác đọc HAL I2C cho AS5600/TCA9548A.
 * @param  device_address: Địa chỉ I2C 7-bit của thiết bị đích.
 * @param  reg: Địa chỉ thanh ghi của thiết bị có register map.
 * @param  buffer: Con trỏ tới bộ đệm nhận dữ liệu.
 * @param  length: Số byte cần nhận.
 * @return 0 nếu thành công, -1 nếu HAL I2C báo lỗi.
 */
static int8_t prv_I2cRead(uint8_t device_address,
                          uint8_t reg,
                          uint8_t *buffer,
                          uint16_t length)
{
    HAL_StatusTypeDef hal_status;

    /*
     * Đọc control byte của TCA9548A là giao dịch master receive trực tiếp,
     * còn AS5600 cần đọc theo địa chỉ thanh ghi.
     */
    if (prv_IsTca9548aAddress(device_address) == true) {
        (void)reg;
        hal_status = HAL_I2C_Master_Receive(&hi2c1,
                                            (uint16_t)(device_address << 1),
                                            buffer,
                                            length,
                                            MY_CONTROLLER_I2C_TIMEOUT_MS);
    } else {
        hal_status = HAL_I2C_Mem_Read(&hi2c1,
                                      (uint16_t)(device_address << 1),
                                      reg,
                                      I2C_MEMADD_SIZE_8BIT,
                                      buffer,
                                      length,
                                      MY_CONTROLLER_I2C_TIMEOUT_MS);
    }

    return (hal_status == HAL_OK) ? 0 : -1;
}

/**
 * @brief  Delay dùng chung cho driver TCA9548A và AS5600.
 * @param  delay_ms: Thời gian delay tính bằng mili giây.
 */
static void prv_DelayMs(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

/**
 * @brief  Đưa bộ lọc Kalman về trạng thái chưa có mẫu đo.
 * @param  filter: Con trỏ tới trạng thái bộ lọc cần reset.
 */
static void prv_KalmanReset(my_controller_kalman_filter_t *filter)
{
    if (filter == NULL) {
        return;
    }

    filter->estimate_cdeg = 0.0f;
    filter->error_covariance = MY_CONTROLLER_KALMAN_INITIAL_COVARIANCE;
    filter->process_noise = MY_CONTROLLER_KALMAN_PROCESS_NOISE;
    filter->measurement_noise = MY_CONTROLLER_KALMAN_MEASUREMENT_NOISE;
    filter->is_initialized = false;
}

/**
 * @brief  Chuẩn hóa góc tuyệt đối về miền 0.00 tới nhỏ hơn 360.00 độ.
 * @param  angle_cdeg: Góc cần chuẩn hóa, tính bằng centi-độ.
 * @return Góc đã chuẩn hóa, tính bằng centi-độ.
 */
static int32_t prv_NormalizeAbsoluteCdeg(int32_t angle_cdeg)
{
    while (angle_cdeg < 0) {
        angle_cdeg += MY_CONTROLLER_FULL_TURN_CDEG;
    }

    while (angle_cdeg >= MY_CONTROLLER_FULL_TURN_CDEG) {
        angle_cdeg -= MY_CONTROLLER_FULL_TURN_CDEG;
    }

    return angle_cdeg;
}

#if (MY_CONTROLLER_USE_KALMAN_FILTER == MY_APP_SENSOR_FILTER_KALMAN)
/**
 * @brief  Tính sai lệch góc ngắn nhất giữa ước lượng và mẫu đo mới.
 * @param  reference_cdeg: Góc ước lượng hiện tại, tính bằng centi-độ.
 * @param  sample_cdeg: Mẫu đo mới đã chuẩn hóa, tính bằng centi-độ.
 * @return Sai lệch có dấu trong khoảng -180.00 tới +180.00 độ.
 */
static float prv_CalculateShortestAngleErrorCdeg(float reference_cdeg,
                                                 int32_t sample_cdeg)
{
    float error_cdeg = (float)sample_cdeg - reference_cdeg;

    while (error_cdeg > (float)MY_CONTROLLER_HALF_TURN_CDEG) {
        error_cdeg -= (float)MY_CONTROLLER_FULL_TURN_CDEG;
    }

    while (error_cdeg < (float)-MY_CONTROLLER_HALF_TURN_CDEG) {
        error_cdeg += (float)MY_CONTROLLER_FULL_TURN_CDEG;
    }

    return error_cdeg;
}

/**
 * @brief  Cập nhật bộ lọc Kalman từ một mẫu góc tuyệt đối AS5600.
 * @param  filter: Con trỏ tới trạng thái bộ lọc Kalman.
 * @param  sample_cdeg: Mẫu đo góc tuyệt đối, tính bằng centi-độ.
 * @return Góc tuyệt đối đã lọc, tính bằng centi-độ.
 */
static int32_t prv_KalmanUpdate(my_controller_kalman_filter_t *filter,
                                int32_t sample_cdeg)
{
    float kalman_gain;
    float innovation_cdeg;

    sample_cdeg = prv_NormalizeAbsoluteCdeg(sample_cdeg);

    if (filter == NULL) {
        return sample_cdeg;
    }

    if (filter->is_initialized == false) {
        filter->estimate_cdeg = (float)sample_cdeg;
        filter->error_covariance = MY_CONTROLLER_KALMAN_INITIAL_COVARIANCE;
        filter->is_initialized = true;
        return sample_cdeg;
    }

    filter->error_covariance += filter->process_noise;
    innovation_cdeg = prv_CalculateShortestAngleErrorCdeg(
        filter->estimate_cdeg,
        sample_cdeg);

    /*
     * Khi motor vừa chạy xong, góc thật có thể đổi lớn giữa hai lần lấy mẫu.
     * Bám nhanh vào mẫu mới giúp bộ lọc không tạo sai số trễ sau chuyển động.
     */
    if ((innovation_cdeg > MY_CONTROLLER_KALMAN_FAST_TRACK_CDEG) ||
        (innovation_cdeg < -MY_CONTROLLER_KALMAN_FAST_TRACK_CDEG)) {
        filter->estimate_cdeg = (float)sample_cdeg;
        filter->error_covariance = MY_CONTROLLER_KALMAN_INITIAL_COVARIANCE;
        return sample_cdeg;
    }

    kalman_gain = filter->error_covariance /
                  (filter->error_covariance + filter->measurement_noise);
    filter->estimate_cdeg += kalman_gain * innovation_cdeg;
    filter->error_covariance *= (1.0f - kalman_gain);

    while (filter->estimate_cdeg < 0.0f) {
        filter->estimate_cdeg += (float)MY_CONTROLLER_FULL_TURN_CDEG;
    }

    while (filter->estimate_cdeg >= (float)MY_CONTROLLER_FULL_TURN_CDEG) {
        filter->estimate_cdeg -= (float)MY_CONTROLLER_FULL_TURN_CDEG;
    }

    return (int32_t)(filter->estimate_cdeg + 0.5f);
}
#elif (MY_CONTROLLER_USE_KALMAN_FILTER == MY_APP_SENSOR_FILTER_NONE)
#else
#error "Invalid MY_APP_SENSOR_FILTER_MODE setting"
#endif

/**
 * @brief  Chuẩn hóa góc tuyệt đối AS5600 thành yaw so với zero phần mềm.
 * @param  sensor_angle_cdeg: Góc cảm biến tuyệt đối, tính bằng centi-độ.
 * @return Góc yaw trong khoảng từ 0.00 tới nhỏ hơn 360.00 độ.
 */
static int32_t prv_NormalizeSensorYawCdeg(int32_t sensor_angle_cdeg)
{
    int32_t yaw_cdeg = sensor_angle_cdeg - s_sensor_zero_cdeg;

    while (yaw_cdeg < 0) {
        yaw_cdeg += MY_CONTROLLER_FULL_TURN_CDEG;
    }

    while (yaw_cdeg >= MY_CONTROLLER_FULL_TURN_CDEG) {
        yaw_cdeg -= MY_CONTROLLER_FULL_TURN_CDEG;
    }

    return yaw_cdeg;
}

/**
 * @brief  Đổi mẫu AS5600 12-bit sang centi-độ bằng số nguyên.
 * @param  data: Mẫu AS5600 đã đọc bằng AS5600_ReadAll().
 * @return Góc tuyệt đối trong khoảng 0 tới nhỏ hơn 36000 centi-độ.
 */
static int32_t prv_ConvertSensorAngleCdeg(const AS5600_Data_t *data)
{
    if (data == NULL) {
        return 0;
    }

    return (int32_t)(((uint64_t)data->angle *
                      (uint64_t)MY_CONTROLLER_FULL_TURN_CDEG) /
                     (uint64_t)MY_CONTROLLER_AS5600_RAW_STEPS);
}

/**
 * @brief  Đọc một AS5600 qua TCA9548A và trả về góc centi-độ.
 * @param  channel: Kênh TCA9548A chứa AS5600 cần đọc.
 * @param  data: Nơi lưu toàn bộ dữ liệu AS5600.
 * @param  angle_cdeg: Nơi lưu góc đã đổi sang centi-độ.
 * @return MY_CONTROLLER_OK nếu đọc cảm biến thành công.
 */
static TCA9548A_Status_t prv_ReadSensorAngleCdeg(
    TCA9548A_Channel_t channel,
    AS5600_Data_t *data,
    int32_t *angle_cdeg)
{
    TCA9548A_Status_t sensor_status;

    if ((data == NULL) || (angle_cdeg == NULL)) {
        return TCA9548A_ERR_NULL_PTR;
    }

    sensor_status = TCA9548A_ReadSensor(&s_mux, channel, data);
    if (sensor_status != TCA9548A_OK) {
        return sensor_status;
    }

    *angle_cdeg = prv_ConvertSensorAngleCdeg(data);
    return TCA9548A_OK;
}

static int32_t prv_CalculateShortestThreeSensorDeltaCdeg(int32_t zero_cdeg,
                                                         int32_t angle_cdeg)
{
    int32_t delta_cdeg = angle_cdeg - zero_cdeg;

    while (delta_cdeg > MY_CONTROLLER_HALF_TURN_CDEG) {
        delta_cdeg -= MY_CONTROLLER_FULL_TURN_CDEG;
    }

    while (delta_cdeg < -MY_CONTROLLER_HALF_TURN_CDEG) {
        delta_cdeg += MY_CONTROLLER_FULL_TURN_CDEG;
    }

    return delta_cdeg;
}

static int32_t prv_ScaleSensorDeltaCdeg(int32_t delta_cdeg,
                                        uint32_t numerator,
                                        uint32_t denominator)
{
    uint32_t abs_delta_cdeg;
    uint32_t scaled_abs_cdeg;

    if ((delta_cdeg == 0) || (numerator == 0U)) {
        return 0;
    }

    abs_delta_cdeg = (delta_cdeg < 0) ?
        (uint32_t)(-delta_cdeg) :
        (uint32_t)delta_cdeg;

    scaled_abs_cdeg = (uint32_t)((((uint64_t)abs_delta_cdeg *
                                   (uint64_t)denominator) +
                                  ((uint64_t)numerator / 2U)) /
                                 (uint64_t)numerator);

    return (delta_cdeg < 0) ?
        -(int32_t)scaled_abs_cdeg :
        (int32_t)scaled_abs_cdeg;
}

/**
 * @brief  Chuyển delta chuyển động có dấu sang hướng quay motor.
 * @param  delta_cdeg: Góc chạy tương đối, tính bằng centi-độ.
 * @return Hướng thuận chiều kim đồng hồ nếu delta không âm.
 */
static TMC2209_DirectionTypeDef prv_GetDirectionFromDelta(int32_t delta_cdeg)
{
    return (delta_cdeg >= 0) ? TMC2209_DIR_CW : TMC2209_DIR_CCW;
}

/**
 * @brief  Lấy chiều ngược lại của một chiều quay TMC2209.
 * @param  direction: Chiều quay cần đảo.
 * @return TMC2209_DIR_CCW nếu đầu vào là CW, ngược lại trả về CW.
 */
static TMC2209_DirectionTypeDef prv_GetOppositeDirection(
    TMC2209_DirectionTypeDef direction)
{
    return (direction == TMC2209_DIR_CW) ? TMC2209_DIR_CCW : TMC2209_DIR_CW;
}

/**
 * @brief  Chọn chiều chạy ba motor từ dấu của delta góc.
 * @param  delta_cdeg: Góc cần chạy từ vị trí hiện tại tới mục tiêu.
 * @param  forward_direction: Chiều thuận của motor khi delta không âm.
 * @return Chiều thuận nếu delta dương, chiều ngược nếu delta âm.
 */
static TMC2209_DirectionTypeDef prv_GetThreeMotorDirection(
    int32_t delta_cdeg,
    TMC2209_DirectionTypeDef forward_direction)
{
    if (delta_cdeg >= 0) {
        return forward_direction;
    }

    return prv_GetOppositeDirection(forward_direction);
}

/**
 * @brief  Chuyển góc tương đối sang số microstep theo cấu hình từng motor.
 * @param  hmotor: Handle motor chứa số full-step và microstep đang dùng.
 * @param  angle_cdeg: Góc tương đối, tính bằng centi-độ.
 * @return Số microstep đã làm tròn cần phát cho motor.
 */
static uint32_t prv_CalculateMotorStepsFromCdeg(
    const TMC2209_HandleTypeDef *hmotor,
    int32_t angle_cdeg)
{
    uint32_t angle_abs_cdeg;
    uint32_t scaled_steps_per_turn;

    if (hmotor == NULL) {
        return 0U;
    }

    angle_abs_cdeg = (angle_cdeg < 0) ?
        (uint32_t)(-angle_cdeg) :
        (uint32_t)angle_cdeg;

    scaled_steps_per_turn = (uint32_t)((uint64_t)hmotor->steps_per_rev *
                                       (uint64_t)hmotor->microstep);

    // Cộng nửa mẫu số để làm tròn gần nhất thay vì luôn làm tròn xuống.
    return (uint32_t)(((uint64_t)angle_abs_cdeg * scaled_steps_per_turn +
                       ((uint64_t)MY_CONTROLLER_FULL_TURN_CDEG / 2U)) /
                      (uint64_t)MY_CONTROLLER_FULL_TURN_CDEG);
}

/**
 * @brief  Nhân số bước với hệ số tỉ số truyền và làm tròn gần nhất.
 * @param  steps: Số microstep gốc đã tính từ góc.
 * @param  numerator: Tử số hệ số scale.
 * @param  denominator: Mẫu số hệ số scale.
 * @return Số microstep sau khi scale.
 */
static uint32_t prv_ScaleMotorSteps(uint32_t steps,
                                    uint32_t numerator,
                                    uint32_t denominator)
{
    if ((steps == 0U) || (denominator == 0U)) {
        return 0U;
    }

    return (uint32_t)((((uint64_t)steps * (uint64_t)numerator) +
                       ((uint64_t)denominator / 2U)) /
                      (uint64_t)denominator);
}

/**
 * @brief  Lấy vị trí lúc khởi động làm gốc 0.00 độ cho cả ba motor.
 */
static void prv_ResetThreeMotorOrigin(void)
{
    s_motor1_current_cdeg = 0;
    s_motor2_current_cdeg = 0;
    s_motor3_current_cdeg = 0;

    motor1.current_angle = 0.0f;
    motor2.current_angle = 0.0f;
    motor3.current_angle = 0.0f;
}

/**
 * @brief  Tính quãng góc cảm biến đã đo, có xét hướng khi đi qua điểm 0.
 * @param  start_cdeg: Góc yaw cảm biến ban đầu, tính bằng centi-độ.
 * @param  end_cdeg: Góc yaw cảm biến cuối, tính bằng centi-độ.
 * @param  expected_delta_cdeg: Độ lớn và hướng chạy đã ra lệnh.
 * @return Delta cảm biến có dấu, tính bằng centi-độ.
 */
static int32_t prv_CalculateSensorDeltaCdeg(int32_t start_cdeg,
                                            int32_t end_cdeg,
                                            int32_t expected_delta_cdeg)
{
    int32_t sensor_delta_cdeg = end_cdeg - start_cdeg;

    /*
     * Khi yaw đi qua mốc 0/360, delta thô sẽ đổi dấu. Hướng lệnh đã gửi cho
     * motor là cơ sở để đưa delta cảm biến về cùng chiều vật lý cần kiểm tra.
     */
    if ((expected_delta_cdeg >= 0) && (sensor_delta_cdeg < 0)) {
        sensor_delta_cdeg += MY_CONTROLLER_FULL_TURN_CDEG;
    }

    if ((expected_delta_cdeg < 0) && (sensor_delta_cdeg > 0)) {
        sensor_delta_cdeg -= MY_CONTROLLER_FULL_TURN_CDEG;
    }

    return sensor_delta_cdeg;
}

/**
 * @brief  Dừng cả ba motor khi cần hủy lệnh chạy nhóm.
 */
static void prv_StopThreeMotors(void)
{
    TMC2209_Stop(&motor1);
    TMC2209_Stop(&motor2);
    TMC2209_Stop(&motor3);
}

/**
 * @brief  Cấu hình handle cho các driver TMC2209.
 */
void MyController_MotorsConfig(void)
{
    motor1.id             = 0U;
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

    motor2.id             = 1U;
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

    motor3.id             = 2U;
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
 * @brief  Khởi tạo các driver TMC2209 đã cấu hình.
 */
void MyController_MotorsInit(void)
{
    MyController_MotorsConfig();

    (void)TMC2209_Init(&motor1);
    (void)TMC2209_Init(&motor2);
    (void)TMC2209_Init(&motor3);

    (void)TMC2209_SetStealthChop(&motor1, true);
    (void)TMC2209_SetStealthChop(&motor2, true);
    (void)TMC2209_SetStealthChop(&motor3, true);

    /*
     * Không có cảm biến tuyệt đối cho từng trục, nên firmware dùng vị trí
     * thực tế lúc cấp nguồn làm mốc 0.00 độ cho ba motor.
     */
    prv_ResetThreeMotorOrigin();
}

/**
 * @brief  Khởi tạo motor, mux TCA9548A và AS5600 dùng cho yaw.
 * @return MY_CONTROLLER_OK nếu toàn bộ đường điều khiển sẵn sàng.
 */
MyController_Status_t MyController_Init(void)
{
    MyController_MotorsInit();

    /*
     * Driver TCA9548A và AS5600 dùng callback I2C của controller để giữ toàn
     * bộ đường phản hồi yaw trong cùng một module.
     */
    s_mux.i2c_write = prv_I2cWrite;
    s_mux.i2c_read = prv_I2cRead;
    s_mux.delay_ms = prv_DelayMs;

    s_sensor_zero_cdeg = 0;
    s_sensor1_zero_cdeg = 0;
    s_sensor2_zero_cdeg = 0;
    s_sensor3_zero_cdeg = 0;
    s_current_yaw_cdeg = 0;
    prv_KalmanReset(&s_sensor_filter);

    if (TCA9548A_Init(&s_mux, TCA9548A_ADDR_A000) != TCA9548A_OK) {
        return MY_CONTROLLER_ERR_SENSOR;
    }

    if (TCA9548A_RegisterSensor(&s_mux,
                                MY_CONTROLLER_SENSOR1_CHANNEL,
                                "AS5600_CH0") != TCA9548A_OK) {
        return MY_CONTROLLER_ERR_SENSOR;
    }

    if (TCA9548A_RegisterSensor(&s_mux,
                                MY_CONTROLLER_SENSOR2_CHANNEL,
                                "AS5600_CH1") != TCA9548A_OK) {
        return MY_CONTROLLER_ERR_SENSOR;
    }

    if (TCA9548A_RegisterSensor(&s_mux,
                                MY_CONTROLLER_SENSOR3_CHANNEL,
                                "AS5600_CH2") != TCA9548A_OK) {
        return MY_CONTROLLER_ERR_SENSOR;
    }

    if (TCA9548A_InitAllSensors(&s_mux) != TCA9548A_OK) {
        return MY_CONTROLLER_ERR_SENSOR;
    }

    if (MyController_ResetThreeSensorZero() != MY_CONTROLLER_OK) {
        return MY_CONTROLLER_ERR_SENSOR;
    }

    return MY_CONTROLLER_OK;
}

/**
 * @brief  Đặt góc AS5600 hiện tại làm mốc yaw zero phần mềm.
 * @return MY_CONTROLLER_OK nếu đọc cảm biến và cập nhật zero thành công.
 */
MyController_Status_t MyController_SetZeroFromSensor(void)
{
    int32_t sensor_angle_cdeg = 0;

    if (MyController_ReadSensorAbsoluteCdeg(&sensor_angle_cdeg) !=
        MY_CONTROLLER_OK) {
        return MY_CONTROLLER_ERR_SENSOR;
    }

    // Zero là mốc phần mềm để tránh ghi vào OTP hoặc register của AS5600.
    s_sensor_zero_cdeg = sensor_angle_cdeg;
    s_current_yaw_cdeg = 0;
    return MY_CONTROLLER_OK;
}

MyController_Status_t MyController_ResetThreeSensorZero(void)
{
    int32_t sensor1_angle_cdeg = 0;
    int32_t sensor2_angle_cdeg = 0;
    int32_t sensor3_angle_cdeg = 0;

    if (prv_ReadSensorAngleCdeg(MY_CONTROLLER_SENSOR1_CHANNEL,
                                &s_sensor1_data,
                                &sensor1_angle_cdeg) != TCA9548A_OK) {
        return MY_CONTROLLER_ERR_SENSOR;
    }

    if (prv_ReadSensorAngleCdeg(MY_CONTROLLER_SENSOR2_CHANNEL,
                                &s_sensor2_data,
                                &sensor2_angle_cdeg) != TCA9548A_OK) {
        return MY_CONTROLLER_ERR_SENSOR;
    }

    if (prv_ReadSensorAngleCdeg(MY_CONTROLLER_SENSOR3_CHANNEL,
                                &s_sensor3_data,
                                &sensor3_angle_cdeg) != TCA9548A_OK) {
        return MY_CONTROLLER_ERR_SENSOR;
    }

    s_sensor1_zero_cdeg = sensor1_angle_cdeg;
    s_sensor2_zero_cdeg = sensor2_angle_cdeg;
    s_sensor3_zero_cdeg = sensor3_angle_cdeg;
    return MY_CONTROLLER_OK;
}

/**
 * @brief  Đọc góc tuyệt đối của AS5600 theo centi-độ.
 * @param  sensor_angle_cdeg: Nơi lưu góc tuyệt đối, tính bằng centi-độ.
 * @return MY_CONTROLLER_OK nếu đọc cảm biến thành công.
 */
MyController_Status_t MyController_ReadSensorAbsoluteCdeg(
    int32_t *sensor_angle_cdeg)
{
    if (sensor_angle_cdeg == NULL) {
        return MY_CONTROLLER_ERR_NULL_PTR;
    }

    if (prv_ReadSensorAngleCdeg(MY_CONTROLLER_SENSOR1_CHANNEL,
                                &s_sensor1_data,
                                sensor_angle_cdeg) != TCA9548A_OK) {
        return MY_CONTROLLER_ERR_SENSOR;
    }

#if (MY_CONTROLLER_USE_KALMAN_FILTER == MY_APP_SENSOR_FILTER_KALMAN)
    *sensor_angle_cdeg = prv_KalmanUpdate(&s_sensor_filter,
                                          *sensor_angle_cdeg);
#elif (MY_CONTROLLER_USE_KALMAN_FILTER == MY_APP_SENSOR_FILTER_NONE)
    *sensor_angle_cdeg = prv_NormalizeAbsoluteCdeg(*sensor_angle_cdeg);
#else
#error "Invalid MY_APP_SENSOR_FILTER_MODE setting"
#endif
    return MY_CONTROLLER_OK;
}

/**
 * @brief  Đọc góc hiện tại của ba AS5600 trên TCA9548A CH0, CH1 và CH2.
 * @param  readout: Nơi lưu góc centi-độ và raw angle của ba cảm biến.
 * @return MY_CONTROLLER_OK nếu đọc đủ cả ba cảm biến.
 */
MyController_Status_t MyController_ReadThreeSensors(
    MyController_ThreeSensorReadout_t *readout)
{
    int32_t sensor_angle_cdeg = 0;
    int32_t sensor_delta_cdeg = 0;

    if (readout == NULL) {
        return MY_CONTROLLER_ERR_NULL_PTR;
    }

    /*
     * Không dừng ở cảm biến lỗi đầu tiên để USB report chỉ rõ kênh nào hỏng,
     * kênh nào vẫn đọc được.
     */
    readout->sensor1_status = (int8_t)prv_ReadSensorAngleCdeg(
        MY_CONTROLLER_SENSOR1_CHANNEL,
        &s_sensor1_data,
        &sensor_angle_cdeg);
    if (readout->sensor1_status == (int8_t)TCA9548A_OK) {
        sensor_delta_cdeg = prv_CalculateShortestThreeSensorDeltaCdeg(
            s_sensor1_zero_cdeg,
            sensor_angle_cdeg);
        readout->sensor1_angle_cdeg = prv_ScaleSensorDeltaCdeg(
            sensor_delta_cdeg,
            MY_CONTROLLER_MOTOR1_STEP_SCALE_NUM,
            MY_CONTROLLER_MOTOR1_STEP_SCALE_DEN);
        readout->sensor1_raw_angle = s_sensor1_data.raw_angle;
    } else {
        readout->sensor1_angle_cdeg = 0;
        readout->sensor1_raw_angle = 0U;
    }

    readout->sensor2_status = (int8_t)prv_ReadSensorAngleCdeg(
        MY_CONTROLLER_SENSOR2_CHANNEL,
        &s_sensor2_data,
        &sensor_angle_cdeg);
    if (readout->sensor2_status == (int8_t)TCA9548A_OK) {
        sensor_delta_cdeg = prv_CalculateShortestThreeSensorDeltaCdeg(
            s_sensor2_zero_cdeg,
            sensor_angle_cdeg);
        readout->sensor2_angle_cdeg = prv_ScaleSensorDeltaCdeg(
            sensor_delta_cdeg,
            MY_CONTROLLER_MOTOR2_STEP_SCALE_NUM,
            MY_CONTROLLER_MOTOR2_STEP_SCALE_DEN);
        readout->sensor2_raw_angle = s_sensor2_data.raw_angle;
    } else {
        readout->sensor2_angle_cdeg = 0;
        readout->sensor2_raw_angle = 0U;
    }

    readout->sensor3_status = (int8_t)prv_ReadSensorAngleCdeg(
        MY_CONTROLLER_SENSOR3_CHANNEL,
        &s_sensor3_data,
        &sensor_angle_cdeg);
    if (readout->sensor3_status == (int8_t)TCA9548A_OK) {
        sensor_delta_cdeg = prv_CalculateShortestThreeSensorDeltaCdeg(
            s_sensor3_zero_cdeg,
            sensor_angle_cdeg);
        readout->sensor3_angle_cdeg = prv_ScaleSensorDeltaCdeg(
            sensor_delta_cdeg,
            MY_CONTROLLER_MOTOR3_STEP_SCALE_NUM,
            MY_CONTROLLER_MOTOR3_STEP_SCALE_DEN);
        readout->sensor3_raw_angle = s_sensor3_data.raw_angle;
    } else {
        readout->sensor3_angle_cdeg = 0;
        readout->sensor3_raw_angle = 0U;
    }

    return MY_CONTROLLER_OK;
}

/**
 * @brief  Đọc yaw hiện tại theo mốc zero phần mềm.
 * @param  sensor_yaw_cdeg: Nơi lưu yaw hiện tại, tính bằng centi-độ.
 * @return MY_CONTROLLER_OK nếu đọc cảm biến thành công.
 */
MyController_Status_t MyController_ReadSensorYawCdeg(int32_t *sensor_yaw_cdeg)
{
    int32_t sensor_angle_cdeg = 0;

    if (sensor_yaw_cdeg == NULL) {
        return MY_CONTROLLER_ERR_NULL_PTR;
    }

    if (MyController_ReadSensorAbsoluteCdeg(&sensor_angle_cdeg) !=
        MY_CONTROLLER_OK) {
        return MY_CONTROLLER_ERR_SENSOR;
    }

    *sensor_yaw_cdeg = prv_NormalizeSensorYawCdeg(sensor_angle_cdeg);
    return MY_CONTROLLER_OK;
}

/**
 * @brief  Bắt đầu chạy motor yaw tới góc mục tiêu.
 * @param  target_yaw_cdeg: Góc yaw mục tiêu tuyệt đối, tính bằng centi-độ.
 * @param  context: Nơi lưu dữ liệu cần dùng khi kết thúc lệnh chạy.
 * @return MY_CONTROLLER_OK nếu lệnh được chấp nhận.
 */
MyController_Status_t MyController_StartTargetMove(
    int32_t target_yaw_cdeg,
    MyController_MoveContext_t *context)
{
    TMC2209_StatusTypeDef motor_status;
    int32_t move_delta_cdeg;
    uint32_t move_steps;

    if (context == NULL) {
        return MY_CONTROLLER_ERR_NULL_PTR;
    }

    if (MyController_ReadSensorYawCdeg(&context->start_sensor_yaw_cdeg) !=
        MY_CONTROLLER_OK) {
        return MY_CONTROLLER_ERR_SENSOR;
    }

    // Motor chỉ cần chạy phần delta giữa yaw hiện tại và yaw mục tiêu mới.
    move_delta_cdeg = target_yaw_cdeg - s_current_yaw_cdeg;
    move_steps = MyController_CalculateMotorStepsFromAngle(move_delta_cdeg);

    context->target_yaw_cdeg = target_yaw_cdeg;
    context->start_yaw_cdeg = s_current_yaw_cdeg;
    context->move_delta_cdeg = move_delta_cdeg;
    context->target_steps = move_steps;

    if (move_steps == 0U) {
        return MY_CONTROLLER_OK;
    }

    // my_controller dùng motor3 làm cơ cấu yaw chính của hệ hiện tại.
    motor_status = TMC2209_MoveSteps(&motor3,
                                     move_steps,
                                     prv_GetDirectionFromDelta(move_delta_cdeg),
                                     MY_CONTROLLER_MOTOR_SPEED_HZ);
    if (motor_status != TMC2209_OK) {
        return MY_CONTROLLER_ERR_MOTOR;
    }

    return MY_CONTROLLER_OK;
}

/**
 * @brief  Bắt đầu chạy đồng thời ba motor theo ba góc nhập từ USB CDC.
 * @param  command: Ba góc mục tiêu tuyệt đối 0..360 độ, tính bằng centi-độ.
 * @param  context: Nơi lưu số bước đã phát lệnh cho từng motor.
 * @return MY_CONTROLLER_OK nếu lệnh được chấp nhận.
 */
MyController_Status_t MyController_StartThreeMotorMove(
    const MyController_ThreeMotorMoveCommand_t *command,
    MyController_ThreeMotorMoveContext_t *context)
{
    TMC2209_StatusTypeDef motor_status;
    TMC2209_DirectionTypeDef motor_direction;

    if ((command == NULL) || (context == NULL)) {
        return MY_CONTROLLER_ERR_NULL_PTR;
    }

    if (MyController_IsThreeMotorMoveRunning() == true) {
        return MY_CONTROLLER_ERR_MOTOR;
    }

    context->motor1_start_cdeg = s_motor1_current_cdeg;
    context->motor2_start_cdeg = s_motor2_current_cdeg;
    context->motor3_start_cdeg = s_motor3_current_cdeg;
    context->motor1_angle_cdeg = command->motor1_angle_cdeg;
    context->motor2_angle_cdeg = command->motor2_angle_cdeg;
    context->motor3_angle_cdeg = command->motor3_angle_cdeg;
    context->motor1_delta_cdeg =
        command->motor1_angle_cdeg - s_motor1_current_cdeg;
    context->motor2_delta_cdeg =
        command->motor2_angle_cdeg - s_motor2_current_cdeg;
    context->motor3_delta_cdeg =
        command->motor3_angle_cdeg - s_motor3_current_cdeg;
    context->motor1_target_steps = prv_ScaleMotorSteps(
        prv_CalculateMotorStepsFromCdeg(&motor1, context->motor1_delta_cdeg),
        MY_CONTROLLER_MOTOR1_STEP_SCALE_NUM,
        MY_CONTROLLER_MOTOR1_STEP_SCALE_DEN);
    context->motor2_target_steps = prv_ScaleMotorSteps(
        prv_CalculateMotorStepsFromCdeg(&motor2, context->motor2_delta_cdeg),
        MY_CONTROLLER_MOTOR2_STEP_SCALE_NUM,
        MY_CONTROLLER_MOTOR2_STEP_SCALE_DEN);
    context->motor3_target_steps = prv_ScaleMotorSteps(
        prv_CalculateMotorStepsFromCdeg(&motor3, context->motor3_delta_cdeg),
        MY_CONTROLLER_MOTOR3_STEP_SCALE_NUM,
        MY_CONTROLLER_MOTOR3_STEP_SCALE_DEN);

    /*
     * Ba giá trị USB là tọa độ tuyệt đối. Dấu của delta quyết định chiều để
     * 0 -> 120 đi thuận, còn 360 -> 120 tự quay ngược về 120.
     */
    if (context->motor1_target_steps > 0U) {
        motor_direction = prv_GetThreeMotorDirection(
            context->motor1_delta_cdeg,
            MY_CONTROLLER_MOTOR1_FORWARD_DIR);
        motor_status = TMC2209_MoveSteps(&motor1,
                                         context->motor1_target_steps,
                                         motor_direction,
                                         MY_CONTROLLER_THREE_MOTOR_SPEED_HZ);
        if (motor_status != TMC2209_OK) {
            prv_StopThreeMotors();
            return MY_CONTROLLER_ERR_MOTOR;
        }
    }

    if (context->motor2_target_steps > 0U) {
        motor_direction = prv_GetThreeMotorDirection(
            context->motor2_delta_cdeg,
            MY_CONTROLLER_MOTOR2_FORWARD_DIR);
        motor_status = TMC2209_MoveSteps(&motor2,
                                         context->motor2_target_steps,
                                         motor_direction,
                                         MY_CONTROLLER_THREE_MOTOR_SPEED_HZ);
        if (motor_status != TMC2209_OK) {
            prv_StopThreeMotors();
            return MY_CONTROLLER_ERR_MOTOR;
        }
    }

    if (context->motor3_target_steps > 0U) {
        motor_direction = prv_GetThreeMotorDirection(
            context->motor3_delta_cdeg,
            MY_CONTROLLER_MOTOR3_FORWARD_DIR);
        motor_status = TMC2209_MoveSteps(&motor3,
                                         context->motor3_target_steps,
                                         motor_direction,
                                         MY_CONTROLLER_THREE_MOTOR_SPEED_HZ);
        if (motor_status != TMC2209_OK) {
            prv_StopThreeMotors();
            return MY_CONTROLLER_ERR_MOTOR;
        }
    }

    return MY_CONTROLLER_OK;
}

/**
 * @brief  Kiểm tra motor yaw chính còn đang chạy hay không.
 * @return true nếu motor yaw chính đang chạy, ngược lại false.
 */
bool MyController_IsTargetMotorRunning(void)
{
    return (motor3.state == TMC2209_RUNNING);
}

/**
 * @brief  Kiểm tra còn motor nào trong nhóm ba motor đang chạy hay không.
 * @return true nếu ít nhất một motor chưa hoàn tất lệnh.
 */
bool MyController_IsThreeMotorMoveRunning(void)
{
    return ((motor1.state == TMC2209_RUNNING) ||
            (motor2.state == TMC2209_RUNNING) ||
            (motor3.state == TMC2209_RUNNING));
}

/**
 * @brief  Hoàn tất lệnh ba motor và cập nhật vị trí phần mềm mới.
 * @param  context: Ngữ cảnh đã lưu khi bắt đầu lệnh chạy.
 * @return MY_CONTROLLER_OK nếu cả ba motor đã dừng.
 */
MyController_Status_t MyController_FinishThreeMotorMove(
    const MyController_ThreeMotorMoveContext_t *context)
{
    if (context == NULL) {
        return MY_CONTROLLER_ERR_NULL_PTR;
    }

    if (MyController_IsThreeMotorMoveRunning() == true) {
        return MY_CONTROLLER_ERR_MOTOR;
    }

    s_motor1_current_cdeg = context->motor1_angle_cdeg;
    s_motor2_current_cdeg = context->motor2_angle_cdeg;
    s_motor3_current_cdeg = context->motor3_angle_cdeg;
    return MY_CONTROLLER_OK;
}

/**
 * @brief  Hoàn tất lệnh yaw và tính sai số dựa trên AS5600.
 * @param  context: Ngữ cảnh đã lưu khi bắt đầu lệnh chạy.
 * @param  result: Nơi lưu kết quả đo sau khi motor dừng.
 * @return MY_CONTROLLER_OK nếu đọc được góc cuối và cập nhật yaw phần mềm.
 */
MyController_Status_t MyController_FinishTargetMove(
    const MyController_MoveContext_t *context,
    MyController_MoveResult_t *result)
{
    if ((context == NULL) || (result == NULL)) {
        return MY_CONTROLLER_ERR_NULL_PTR;
    }

    if (MyController_IsTargetMotorRunning() == true) {
        return MY_CONTROLLER_ERR_MOTOR;
    }

    if (MyController_ReadSensorYawCdeg(&result->end_sensor_yaw_cdeg) !=
        MY_CONTROLLER_OK) {
        return MY_CONTROLLER_ERR_SENSOR;
    }

    result->sensor_delta_cdeg = prv_CalculateSensorDeltaCdeg(
        context->start_sensor_yaw_cdeg,
        result->end_sensor_yaw_cdeg,
        context->move_delta_cdeg);
    result->error_cdeg = context->move_delta_cdeg - result->sensor_delta_cdeg;

    // Yaw phần mềm đi theo lệnh đã chấp nhận, còn sai số được trả về riêng.
    s_current_yaw_cdeg = context->target_yaw_cdeg;
    return MY_CONTROLLER_OK;
}

/**
 * @brief  Chuyển delta yaw tương đối sang số microstep của motor yaw.
 * @param  angle_cdeg: Delta yaw tương đối, tính bằng centi-độ.
 * @return Số microstep đã làm tròn cần dùng cho chuyển động.
 */
uint32_t MyController_CalculateMotorStepsFromAngle(int32_t angle_cdeg)
{
    // Dùng cấu hình motor3 để số step khớp với driver đang thực sự chạy yaw.
    return prv_CalculateMotorStepsFromCdeg(&motor3, angle_cdeg);
}

/**
 * @brief  Hàm gọi lại khi hoàn tất xung PWM timer để đếm bước motor.
 * @param  htim: Handle HAL timer phát sinh callback.
 * @note   Giữ callback ISR này ngắn, chỉ chuyển tiếp việc cập nhật bước.
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == motor1.htim) {
        (void)TMC2209_UpdateSteps(&motor1);
    }

    if (htim == motor2.htim) {
        (void)TMC2209_UpdateSteps(&motor2);
    }

    if (htim == motor3.htim) {
        (void)TMC2209_UpdateSteps(&motor3);
    }
}
