/**
 * @file    my_controller.h
 * @brief   Giao diện điều khiển motor yaw và cảm biến góc AS5600.
 * @author  Lap4all
 * @date    2026-05-14
 */

#ifndef MYLIB_INC_MY_CONTROLLER_H_
#define MYLIB_INC_MY_CONTROLLER_H_

#include "TMC2209.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief Một vòng quay, tính bằng centi-độ. */
#define MY_CONTROLLER_FULL_TURN_CDEG    36000L

extern TMC2209_HandleTypeDef motor1; /**< Driver TMC2209 thứ nhất. */
extern TMC2209_HandleTypeDef motor2; /**< Driver TMC2209 thứ hai. */
extern TMC2209_HandleTypeDef motor3; /**< Driver TMC2209 thứ ba. */

/**
 * @brief  Mã trạng thái của module điều khiển yaw.
 */
typedef enum {
    MY_CONTROLLER_OK = 0,              /**< Thao tác thành công. */
    MY_CONTROLLER_ERR_NULL_PTR = -1,   /**< Tham số con trỏ không hợp lệ. */
    MY_CONTROLLER_ERR_SENSOR = -2,     /**< Lỗi đọc hoặc init AS5600. */
    MY_CONTROLLER_ERR_MOTOR = -3,      /**< TMC2209 không nhận lệnh chạy. */
} MyController_Status_t;

/**
 * @brief  Ngữ cảnh được lưu từ lúc bắt đầu tới khi hoàn tất một lệnh yaw.
 */
typedef struct {
    int32_t target_yaw_cdeg;        /**< Góc yaw mục tiêu, centi-độ. */
    int32_t start_yaw_cdeg;         /**< Góc yaw phần mềm trước khi chạy. */
    int32_t move_delta_cdeg;        /**< Góc chạy tương đối đã ra lệnh. */
    int32_t start_sensor_yaw_cdeg;  /**< Yaw cảm biến trước khi chạy. */
    uint32_t target_steps;          /**< Số microstep motor cần chạy. */
} MyController_MoveContext_t;

/**
 * @brief  Kết quả đo sau khi một lệnh yaw hoàn tất.
 */
typedef struct {
    int32_t end_sensor_yaw_cdeg;    /**< Góc yaw cảm biến sau khi motor dừng. */
    int32_t sensor_delta_cdeg;      /**< Delta yaw cảm biến đo được. */
    int32_t error_cdeg;             /**< Sai số lệnh yaw và cảm biến. */
} MyController_MoveResult_t;

/**
 * @brief  Dữ liệu góc đọc từ ba AS5600 gắn trên TCA9548A CH0..CH2.
 */
typedef struct {
    int32_t sensor1_angle_cdeg;     /**< Góc AS5600 kênh 0, centi-độ. */
    int32_t sensor2_angle_cdeg;     /**< Góc AS5600 kênh 1, centi-độ. */
    int32_t sensor3_angle_cdeg;     /**< Góc AS5600 kênh 2, centi-độ. */
    uint16_t sensor1_raw_angle;     /**< Góc raw 12-bit của AS5600 kênh 0. */
    uint16_t sensor2_raw_angle;     /**< Góc raw 12-bit của AS5600 kênh 1. */
    uint16_t sensor3_raw_angle;     /**< Góc raw 12-bit của AS5600 kênh 2. */
    int8_t sensor1_status;          /**< Trạng thái đọc kênh 0, 0 là OK. */
    int8_t sensor2_status;          /**< Trạng thái đọc kênh 1, 0 là OK. */
    int8_t sensor3_status;          /**< Trạng thái đọc kênh 2, 0 là OK. */
} MyController_ThreeSensorReadout_t;

/**
 * @brief  Lệnh chạy đồng thời ba motor theo góc mục tiêu tuyệt đối.
 */
typedef struct {
    int32_t motor1_angle_cdeg;      /**< Góc mục tiêu motor1, centi-độ. */
    int32_t motor2_angle_cdeg;      /**< Góc mục tiêu motor2, centi-độ. */
    int32_t motor3_angle_cdeg;      /**< Góc mục tiêu motor3, centi-độ. */
} MyController_ThreeMotorMoveCommand_t;

/**
 * @brief  Ngữ cảnh lệnh chạy đồng thời ba motor.
 */
typedef struct {
    int32_t motor1_start_cdeg;      /**< Góc bắt đầu motor1, centi-độ. */
    int32_t motor2_start_cdeg;      /**< Góc bắt đầu motor2, centi-độ. */
    int32_t motor3_start_cdeg;      /**< Góc bắt đầu motor3, centi-độ. */
    int32_t motor1_angle_cdeg;      /**< Góc mục tiêu motor1, centi-độ. */
    int32_t motor2_angle_cdeg;      /**< Góc mục tiêu motor2, centi-độ. */
    int32_t motor3_angle_cdeg;      /**< Góc mục tiêu motor3, centi-độ. */
    int32_t motor1_delta_cdeg;      /**< Delta đã ra lệnh cho motor1. */
    int32_t motor2_delta_cdeg;      /**< Delta đã ra lệnh cho motor2. */
    int32_t motor3_delta_cdeg;      /**< Delta đã ra lệnh cho motor3. */
    uint32_t motor1_target_steps;   /**< Số microstep motor1 cần chạy. */
    uint32_t motor2_target_steps;   /**< Số microstep motor2 cần chạy. */
    uint32_t motor3_target_steps;   /**< Số microstep motor3 cần chạy. */
} MyController_ThreeMotorMoveContext_t;

/**
 * @brief  Cấu hình handle cho các driver TMC2209.
 */
void MyController_MotorsConfig(void);

/**
 * @brief  Khởi tạo các driver TMC2209 đã cấu hình.
 */
void MyController_MotorsInit(void);

/**
 * @brief  Khởi tạo motor, mux TCA9548A và AS5600 dùng cho yaw.
 * @return MY_CONTROLLER_OK nếu toàn bộ đường điều khiển sẵn sàng.
 */
MyController_Status_t MyController_Init(void);

/**
 * @brief  Đặt góc AS5600 hiện tại làm mốc yaw zero phần mềm.
 * @return MY_CONTROLLER_OK nếu đọc cảm biến và cập nhật zero thành công.
 */
MyController_Status_t MyController_SetZeroFromSensor(void);

/**
 * @brief  Đọc góc tuyệt đối của AS5600 theo centi-độ.
 * @param  sensor_angle_cdeg: Nơi lưu góc tuyệt đối, tính bằng centi-độ.
 * @return MY_CONTROLLER_OK nếu đọc cảm biến thành công.
 */
MyController_Status_t MyController_ReadSensorAbsoluteCdeg(
    int32_t *sensor_angle_cdeg);

/**
 * @brief  Đọc yaw hiện tại theo mốc zero phần mềm.
 * @param  sensor_yaw_cdeg: Nơi lưu yaw hiện tại, tính bằng centi-độ.
 * @return MY_CONTROLLER_OK nếu đọc cảm biến thành công.
 */
MyController_Status_t MyController_ReadSensorYawCdeg(
    int32_t *sensor_yaw_cdeg);

/**
 * @brief  Đọc góc hiện tại của ba AS5600 trên TCA9548A CH0, CH1 và CH2.
 * @param  readout: Nơi lưu góc centi-độ và raw angle của ba cảm biến.
 * @return MY_CONTROLLER_OK nếu thao tác đọc được thực hiện.
 * @note   Kiểm tra sensor1_status..sensor3_status để biết lỗi từng kênh.
 */
MyController_Status_t MyController_ReadThreeSensors(
    MyController_ThreeSensorReadout_t *readout);

/**
 * @brief  Bắt đầu chạy motor yaw tới góc mục tiêu.
 * @param  target_yaw_cdeg: Góc yaw mục tiêu tuyệt đối, tính bằng centi-độ.
 * @param  context: Nơi lưu dữ liệu cần dùng khi kết thúc lệnh chạy.
 * @return MY_CONTROLLER_OK nếu lệnh được chấp nhận.
 */
MyController_Status_t MyController_StartTargetMove(
    int32_t target_yaw_cdeg,
    MyController_MoveContext_t *context);

/**
 * @brief  Bắt đầu chạy đồng thời ba motor theo ba góc nhập từ USB CDC.
 * @param  command: Ba góc mục tiêu tuyệt đối 0..360 độ, tính bằng centi-độ.
 * @param  context: Nơi lưu số bước đã phát lệnh cho từng motor.
 * @return MY_CONTROLLER_OK nếu lệnh được chấp nhận.
 * @note   Vị trí hiện tại lúc khởi động được xem là 0.00 độ cho cả ba motor.
 */
MyController_Status_t MyController_StartThreeMotorMove(
    const MyController_ThreeMotorMoveCommand_t *command,
    MyController_ThreeMotorMoveContext_t *context);

/**
 * @brief  Kiểm tra motor yaw chính còn đang chạy hay không.
 * @return true nếu motor yaw chính đang chạy, ngược lại false.
 */
bool MyController_IsTargetMotorRunning(void);

/**
 * @brief  Kiểm tra còn motor nào trong nhóm ba motor đang chạy hay không.
 * @return true nếu ít nhất một motor chưa hoàn tất lệnh.
 */
bool MyController_IsThreeMotorMoveRunning(void);

/**
 * @brief  Hoàn tất lệnh ba motor và cập nhật vị trí phần mềm mới.
 * @param  context: Ngữ cảnh đã lưu khi bắt đầu lệnh chạy.
 * @return MY_CONTROLLER_OK nếu cả ba motor đã dừng.
 */
MyController_Status_t MyController_FinishThreeMotorMove(
    const MyController_ThreeMotorMoveContext_t *context);

/**
 * @brief  Hoàn tất lệnh yaw và tính sai số dựa trên AS5600.
 * @param  context: Ngữ cảnh đã lưu khi bắt đầu lệnh chạy.
 * @param  result: Nơi lưu kết quả đo sau khi motor dừng.
 * @return MY_CONTROLLER_OK nếu đọc được góc cuối và cập nhật yaw phần mềm.
 */
MyController_Status_t MyController_FinishTargetMove(
    const MyController_MoveContext_t *context,
    MyController_MoveResult_t *result);

/**
 * @brief  Chuyển delta yaw tương đối sang số microstep của motor yaw.
 * @param  angle_cdeg: Delta yaw tương đối, tính bằng centi-độ.
 * @return Số microstep đã làm tròn cần dùng cho chuyển động.
 */
uint32_t MyController_CalculateMotorStepsFromAngle(int32_t angle_cdeg);

#endif /* MYLIB_INC_MY_CONTROLLER_H_ */
