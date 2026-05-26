/**
 * @file    my_app.c
 * @brief   Tầng ứng dụng xử lý lệnh USB CDC cho yaw và ba motor.
 * @author  Lap4all
 * @date    2026-05-14
 */

#include "my_app.h"
#include "my_controller.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/** @brief Kích thước bộ đệm truyền USB CDC. */
#define MY_APP_USB_TX_BUFFER_SIZE      192U

/** @brief Kích thước bộ đệm nhận lệnh USB CDC. */
#define MY_APP_USB_RX_BUFFER_SIZE      64U

/** @brief Kích thước chuỗi cho một trường góc. */
#define MY_APP_ANGLE_TEXT_BUFFER_SIZE  16U

/**
 * @brief  Các trạng thái chính của máy trạng thái ứng dụng.
 */
typedef enum {
    MY_APP_STATE_WAIT_COMMAND = 0,
    MY_APP_STATE_WAIT_MOTOR,
    MY_APP_STATE_WAIT_THREE_MOTOR,
} my_app_state_t;

extern USBD_HandleTypeDef hUsbDeviceFS;

/** @brief Trạng thái hiện tại của xử lý lệnh. */
static my_app_state_t s_app_state = MY_APP_STATE_WAIT_COMMAND;

/** @brief Ngữ cảnh lệnh yaw đang chờ hoàn tất. */
static MyController_MoveContext_t s_move_context;

/** @brief Ngữ cảnh lệnh chạy đồng thời ba motor đang chờ hoàn tất. */
static MyController_ThreeMotorMoveContext_t s_three_motor_context;

/** @brief Cờ cho biết controller đã sẵn sàng xử lý lệnh. */
static bool s_is_controller_ready = false;

/** @brief Cờ cho biết nhánh cảm biến AS5600 không khởi tạo được. */
static bool s_has_sensor_init_error = false;

/** @brief Cờ chặn báo lỗi init lặp lại qua USB. */
static bool s_has_init_error_been_reported = false;

/** @brief Bộ đệm truyền USB dùng chung cho report. */
static char s_usb_tx_buffer[MY_APP_USB_TX_BUFFER_SIZE];

static bool my_app_usb_is_ready(void);
static void my_app_usb_send_text(const char *text);
static bool my_app_is_zero_command(const uint8_t *buffer, uint16_t length);
static bool my_app_parse_target_angle_cdeg(const uint8_t *buffer,
                                           uint16_t length,
                                           int32_t *target_angle_cdeg);
static bool my_app_parse_three_motor_command(
    const uint8_t *buffer,
    uint16_t length,
    MyController_ThreeMotorMoveCommand_t *command);
static void my_app_format_angle_deg(int32_t angle_cdeg,
                                    char *buffer,
                                    uint16_t buffer_size);
static bool my_app_set_zero_from_sensor(void);
static bool my_app_start_target_move(int32_t target_yaw_cdeg);
static bool my_app_start_three_motor_move(
    const MyController_ThreeMotorMoveCommand_t *command);
static void my_app_process_usb_command(void);
static void my_app_process_motor_done(void);
static void my_app_process_three_motor_done(void);

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
    if ((cdc_handle == NULL) || (cdc_handle->TxState != 0U)) {
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
    uint16_t text_length = 0U;

    if ((text == NULL) || (my_app_usb_is_ready() == false)) {
        return;
    }

    // Giới hạn chiều dài để không vượt quá buffer truyền USB dùng chung.
    while ((text[text_length] != '\0') &&
           (text_length < (MY_APP_USB_TX_BUFFER_SIZE - 1U))) {
        text_length++;
    }

    if (text_length > 0U) {
        (void)CDC_Transmit_FS((uint8_t *)text, text_length);
    }
}

/**
 * @brief  Nhận diện lệnh "zero" không phân biệt hoa thường.
 * @param  buffer: Các byte lệnh nhận được.
 * @param  length: Số byte trong buffer.
 * @return true nếu lệnh yêu cầu đặt lại yaw zero, ngược lại false.
 */
static bool my_app_is_zero_command(const uint8_t *buffer, uint16_t length)
{
    uint16_t start_index = 0U;
    uint16_t end_index = length;

    // Chấp nhận lệnh từ terminal có khoảng trắng hoặc CR/LF ở đầu dòng.
    while ((start_index < length) &&
           ((buffer[start_index] == ' ') ||
            (buffer[start_index] == '\t') ||
            (buffer[start_index] == '\r') ||
            (buffer[start_index] == '\n'))) {
        start_index++;
    }

    // Terminal thường gửi kèm CR/LF, nên bỏ phần đuôi trước khi so khớp.
    while ((end_index > start_index) &&
           ((buffer[end_index - 1U] == ' ') ||
            (buffer[end_index - 1U] == '\t') ||
            (buffer[end_index - 1U] == '\r') ||
            (buffer[end_index - 1U] == '\n'))) {
        end_index--;
    }

    if ((end_index - start_index) != 4U) {
        return false;
    }

    return (((buffer[start_index] == 'z') ||
             (buffer[start_index] == 'Z')) &&
            ((buffer[start_index + 1U] == 'e') ||
             (buffer[start_index + 1U] == 'E')) &&
            ((buffer[start_index + 2U] == 'r') ||
             (buffer[start_index + 2U] == 'R')) &&
            ((buffer[start_index + 3U] == 'o') ||
             (buffer[start_index + 3U] == 'O')));
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
    uint16_t index = 0U;
    int32_t whole_deg = 0;
    int32_t frac_cdeg = 0;
    uint8_t frac_digits = 0U;
    bool has_digit = false;

    if ((buffer == NULL) || (target_angle_cdeg == NULL) || (length == 0U)) {
        return false;
    }

    while ((index < length) &&
           ((buffer[index] == ' ') || (buffer[index] == '\t'))) {
        index++;
    }

    while ((index < length) && (buffer[index] >= '0') &&
           (buffer[index] <= '9')) {
        has_digit = true;
        whole_deg = (whole_deg * 10) + (int32_t)(buffer[index] - '0');
        index++;
    }

    /*
     * Module controller nhận centi-độ, nên app chỉ chấp nhận tối đa hai chữ
     * số thập phân để tránh làm người dùng tưởng hệ thống giữ độ phân giải hơn.
     */
    if ((index < length) && (buffer[index] == '.')) {
        index++;
        while ((index < length) &&
               (buffer[index] >= '0') &&
               (buffer[index] <= '9') &&
               (frac_digits < 2U)) {
            has_digit = true;
            frac_cdeg = (frac_cdeg * 10) + (int32_t)(buffer[index] - '0');
            frac_digits++;
            index++;
        }
    }

    if (frac_digits == 1U) {
        frac_cdeg *= 10;
    }

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
 * @brief  Kiểm tra ký tự phân tách giữa các góc trong lệnh ba motor.
 * @param  value: Ký tự cần kiểm tra.
 * @return true nếu là khoảng trắng, dấu phẩy hoặc dấu chấm phẩy.
 */
static bool my_app_is_three_motor_separator(uint8_t value)
{
    return ((value == ' ') ||
            (value == '\t') ||
            (value == ',') ||
            (value == ';'));
}

/**
 * @brief  Bỏ qua các ký tự phân tách trong lệnh ba motor.
 * @param  buffer: Các byte lệnh nhận được.
 * @param  length: Số byte trong buffer.
 * @param  index: Vị trí đọc hiện tại, được cập nhật sau khi bỏ qua.
 */
static void my_app_skip_three_motor_separators(const uint8_t *buffer,
                                               uint16_t length,
                                               uint16_t *index)
{
    while ((*index < length) &&
           (my_app_is_three_motor_separator(buffer[*index]) == true)) {
        (*index)++;
    }
}

/**
 * @brief  Phân tích một trường góc dương trong lệnh ba motor.
 * @param  buffer: Các byte lệnh nhận được.
 * @param  length: Số byte trong buffer.
 * @param  index: Vị trí đọc hiện tại, được cập nhật sau khi đọc trường.
 * @param  angle_cdeg: Góc sau khi phân tích, tính bằng centi-độ.
 * @return true nếu trường hiện tại là một số góc hợp lệ.
 */
static bool my_app_parse_angle_field_cdeg(const uint8_t *buffer,
                                          uint16_t length,
                                          uint16_t *index,
                                          int32_t *angle_cdeg)
{
    int32_t whole_deg = 0;
    int32_t frac_cdeg = 0;
    uint8_t frac_digits = 0U;
    bool has_digit = false;

    while ((*index < length) &&
           (buffer[*index] >= '0') &&
           (buffer[*index] <= '9')) {
        has_digit = true;
        whole_deg = (whole_deg * 10) + (int32_t)(buffer[*index] - '0');
        (*index)++;
    }

    if ((*index < length) && (buffer[*index] == '.')) {
        (*index)++;
        while ((*index < length) &&
               (buffer[*index] >= '0') &&
               (buffer[*index] <= '9') &&
               (frac_digits < 2U)) {
            has_digit = true;
            frac_cdeg = (frac_cdeg * 10) +
                        (int32_t)(buffer[*index] - '0');
            frac_digits++;
            (*index)++;
        }
    }

    if ((frac_digits == 2U) &&
        (*index < length) &&
        (buffer[*index] >= '0') &&
        (buffer[*index] <= '9')) {
        return false;
    }

    if (frac_digits == 1U) {
        frac_cdeg *= 10;
    }

    if (has_digit == false) {
        return false;
    }

    *angle_cdeg = (whole_deg * 100) + frac_cdeg;
    return true;
}

/**
 * @brief  Phân tích lệnh gồm ba góc mục tiêu để chạy đồng thời ba motor.
 * @param  buffer: Các byte lệnh nhận được.
 * @param  length: Số byte trong buffer.
 * @param  command: Nơi lưu ba góc mục tiêu, tính bằng centi-độ.
 * @return true nếu lệnh có đúng ba giá trị góc hợp lệ.
 * @note   Chấp nhận dạng "10 20 30", "10,20,30" hoặc "10;20;30".
 */
static bool my_app_parse_three_motor_command(
    const uint8_t *buffer,
    uint16_t length,
    MyController_ThreeMotorMoveCommand_t *command)
{
    uint16_t index = 0U;
    int32_t angles_cdeg[3] = {0};

    if ((buffer == NULL) || (command == NULL) || (length == 0U)) {
        return false;
    }

    for (uint8_t motor_index = 0U; motor_index < 3U; motor_index++) {
        my_app_skip_three_motor_separators(buffer, length, &index);

        if (my_app_parse_angle_field_cdeg(buffer,
                                          length,
                                          &index,
                                          &angles_cdeg[motor_index]) ==
            false) {
            return false;
        }

        if (angles_cdeg[motor_index] > MY_CONTROLLER_FULL_TURN_CDEG) {
            return false;
        }
    }

    while ((index < length) &&
           ((my_app_is_three_motor_separator(buffer[index]) == true) ||
            (buffer[index] == '\r') ||
            (buffer[index] == '\n'))) {
        index++;
    }

    if (index != length) {
        return false;
    }

    command->motor1_angle_cdeg = angles_cdeg[0];
    command->motor2_angle_cdeg = angles_cdeg[1];
    command->motor3_angle_cdeg = angles_cdeg[2];
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

    if ((buffer == NULL) || (buffer_size == 0U)) {
        return;
    }

    angle_abs_cdeg = (angle_cdeg < 0) ?
        (uint32_t)(-angle_cdeg) :
        (uint32_t)angle_cdeg;

    angle_whole_deg = angle_abs_cdeg / 100U;
    angle_frac_cdeg = angle_abs_cdeg % 100U;

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
 * @brief  Đặt góc cảm biến hiện tại làm yaw zero và báo kết quả qua USB.
 * @return true nếu controller cập nhật zero thành công.
 */
static bool my_app_set_zero_from_sensor(void)
{
    if (MyController_SetZeroFromSensor() != MY_CONTROLLER_OK) {
        my_app_usb_send_text("ERR: cannot read sensor for zero\r\n");
        return false;
    }

    s_app_state = MY_APP_STATE_WAIT_COMMAND;
    my_app_usb_send_text("zero_ok yaw_deg=0.00\r\n");
    return true;
}

/**
 * @brief  Bắt đầu chạy motor từ yaw hiện tại tới yaw mục tiêu.
 * @param  target_yaw_cdeg: Góc yaw mục tiêu tuyệt đối, tính bằng centi-độ.
 * @return true nếu lệnh chạy được controller chấp nhận.
 */
static bool my_app_start_target_move(int32_t target_yaw_cdeg)
{
    MyController_Status_t controller_status;

    controller_status = MyController_StartTargetMove(target_yaw_cdeg,
                                                     &s_move_context);
    if (controller_status == MY_CONTROLLER_ERR_SENSOR) {
        my_app_usb_send_text("ERR: cannot read start angle\r\n");
        return false;
    }

    if (controller_status == MY_CONTROLLER_ERR_MOTOR) {
        my_app_usb_send_text("ERR: motor start failed\r\n");
        return false;
    }

    if (controller_status != MY_CONTROLLER_OK) {
        my_app_usb_send_text("ERR: controller start failed\r\n");
        return false;
    }

    /*
     * Kể cả khi target_steps bằng 0, app vẫn đi qua trạng thái WAIT_MOTOR để
     * tạo một report cùng định dạng với các lệnh có chạy motor.
     */
    s_app_state = MY_APP_STATE_WAIT_MOTOR;
    return true;
}

/**
 * @brief  Bắt đầu lệnh chạy đồng thời ba motor từ USB CDC.
 * @param  command: Ba góc cần chạy theo thứ tự motor1, motor2, motor3.
 * @return true nếu lệnh được controller chấp nhận.
 */
static bool my_app_start_three_motor_move(
    const MyController_ThreeMotorMoveCommand_t *command)
{
    MyController_Status_t controller_status;

    controller_status = MyController_StartThreeMotorMove(
        command,
        &s_three_motor_context);
    if (controller_status == MY_CONTROLLER_ERR_MOTOR) {
        my_app_usb_send_text("ERR: three-motor start failed\r\n");
        return false;
    }

    if (controller_status != MY_CONTROLLER_OK) {
        my_app_usb_send_text("ERR: three-motor command failed\r\n");
        return false;
    }

    /*
     * Lệnh toàn số 0 vẫn đi qua trạng thái chờ để terminal nhận được report
     * cùng định dạng với các lệnh có tạo xung STEP.
     */
    s_app_state = MY_APP_STATE_WAIT_THREE_MOTOR;
    return true;
}

/**
 * @brief  Xử lý một lệnh USB CDC đang chờ khi ứng dụng ở trạng thái rảnh.
 * @note   Ba góc cho ba motor là góc mục tiêu tuyệt đối trong khoảng 0..360.
 */
static void my_app_process_usb_command(void)
{
    uint8_t command_buffer[MY_APP_USB_RX_BUFFER_SIZE];
    uint16_t command_length = 0U;
    int32_t target_yaw_cdeg = 0;
    MyController_ThreeMotorMoveCommand_t three_motor_command = {0};

    if (CDC_ReadCommand(command_buffer,
                        sizeof(command_buffer),
                        &command_length) == 0U) {
        return;
    }

    // Lệnh zero được ưu tiên vì không phải là một giá trị góc số.
    if (my_app_is_zero_command(command_buffer, command_length) == true) {
        (void)my_app_set_zero_from_sensor();
        return;
    }

    if (my_app_parse_three_motor_command(command_buffer,
                                         command_length,
                                         &three_motor_command) == true) {
        (void)my_app_start_three_motor_move(&three_motor_command);
        return;
    }

    if (my_app_parse_target_angle_cdeg(command_buffer,
                                       command_length,
                                       &target_yaw_cdeg) == false) {
        my_app_usb_send_text(
            "ERR: input must be zero, angle, or three angles in deg\r\n");
        return;
    }

    if ((target_yaw_cdeg < 0) ||
        (target_yaw_cdeg > MY_CONTROLLER_FULL_TURN_CDEG)) {
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
    MyController_MoveResult_t move_result;
    MyController_Status_t controller_status;
    int32_t report_length;
    char input_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char start_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char end_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char delta_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char error_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];

    if (MyController_IsTargetMotorRunning() == true) {
        return;
    }

    controller_status = MyController_FinishTargetMove(&s_move_context,
                                                      &move_result);
    if (controller_status == MY_CONTROLLER_ERR_SENSOR) {
        my_app_usb_send_text("ERR: cannot read end angle\r\n");
        s_app_state = MY_APP_STATE_WAIT_COMMAND;
        return;
    }

    if (controller_status != MY_CONTROLLER_OK) {
        my_app_usb_send_text("ERR: controller finish failed\r\n");
        s_app_state = MY_APP_STATE_WAIT_COMMAND;
        return;
    }

    my_app_format_angle_deg(s_move_context.target_yaw_cdeg,
                            input_angle_text,
                            sizeof(input_angle_text));
    my_app_format_angle_deg(s_move_context.start_sensor_yaw_cdeg,
                            start_angle_text,
                            sizeof(start_angle_text));
    my_app_format_angle_deg(move_result.end_sensor_yaw_cdeg,
                            end_angle_text,
                            sizeof(end_angle_text));
    my_app_format_angle_deg(move_result.sensor_delta_cdeg,
                            delta_angle_text,
                            sizeof(delta_angle_text));
    my_app_format_angle_deg(move_result.error_cdeg,
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

    // Giữ nguyên tên trường vì read_uart.py đang dùng format này để ghi CSV.
    if ((report_length > 0) &&
        (report_length < (int32_t)sizeof(s_usb_tx_buffer))) {
        (void)CDC_Transmit_FS((uint8_t *)s_usb_tx_buffer,
                              (uint16_t)report_length);
    }

    s_app_state = MY_APP_STATE_WAIT_COMMAND;
}

/**
 * @brief  Hoàn tất lệnh ba motor và báo target, delta, số bước đã chạy.
 */
static void my_app_process_three_motor_done(void)
{
    MyController_Status_t controller_status;
    MyController_ThreeSensorReadout_t sensor_readout;
    int32_t report_length;
    char motor1_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char motor2_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char motor3_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char motor1_delta_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char motor2_delta_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char motor3_delta_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char sensor1_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char sensor2_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char sensor3_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    unsigned long motor1_steps;
    unsigned long motor2_steps;
    unsigned long motor3_steps;

    if (MyController_IsThreeMotorMoveRunning() == true) {
        return;
    }

    controller_status = MyController_FinishThreeMotorMove(
        &s_three_motor_context);
    if (controller_status != MY_CONTROLLER_OK) {
        my_app_usb_send_text("ERR: three-motor finish failed\r\n");
        s_app_state = MY_APP_STATE_WAIT_COMMAND;
        return;
    }

    controller_status = MyController_ReadThreeSensors(&sensor_readout);
    if (controller_status != MY_CONTROLLER_OK) {
        my_app_usb_send_text("ERR: cannot read 3 AS5600 sensors\r\n");
        s_app_state = MY_APP_STATE_WAIT_COMMAND;
        return;
    }

    my_app_format_angle_deg(s_three_motor_context.motor1_angle_cdeg,
                            motor1_angle_text,
                            sizeof(motor1_angle_text));
    my_app_format_angle_deg(s_three_motor_context.motor2_angle_cdeg,
                            motor2_angle_text,
                            sizeof(motor2_angle_text));
    my_app_format_angle_deg(s_three_motor_context.motor3_angle_cdeg,
                            motor3_angle_text,
                            sizeof(motor3_angle_text));
    my_app_format_angle_deg(s_three_motor_context.motor1_delta_cdeg,
                            motor1_delta_text,
                            sizeof(motor1_delta_text));
    my_app_format_angle_deg(s_three_motor_context.motor2_delta_cdeg,
                            motor2_delta_text,
                            sizeof(motor2_delta_text));
    my_app_format_angle_deg(s_three_motor_context.motor3_delta_cdeg,
                            motor3_delta_text,
                            sizeof(motor3_delta_text));
    my_app_format_angle_deg(sensor_readout.sensor1_angle_cdeg,
                            sensor1_angle_text,
                            sizeof(sensor1_angle_text));
    my_app_format_angle_deg(sensor_readout.sensor2_angle_cdeg,
                            sensor2_angle_text,
                            sizeof(sensor2_angle_text));
    my_app_format_angle_deg(sensor_readout.sensor3_angle_cdeg,
                            sensor3_angle_text,
                            sizeof(sensor3_angle_text));
    motor1_steps = (unsigned long)s_three_motor_context.motor1_target_steps;
    motor2_steps = (unsigned long)s_three_motor_context.motor2_target_steps;
    motor3_steps = (unsigned long)s_three_motor_context.motor3_target_steps;

    report_length = snprintf(s_usb_tx_buffer,
                             sizeof(s_usb_tx_buffer),
                             "three_ok,m1_t=%s,m1_d=%s,m1_s=%lu,"
                             "m2_t=%s,m2_d=%s,m2_s=%lu,"
                             "m3_t=%s,m3_d=%s,m3_s=%lu,"
                             "as1=%s,as2=%s,as3=%s\r\n",
                             motor1_angle_text,
                             motor1_delta_text,
                             motor1_steps,
                             motor2_angle_text,
                             motor2_delta_text,
                             motor2_steps,
                             motor3_angle_text,
                             motor3_delta_text,
                             motor3_steps,
                             sensor1_angle_text,
                             sensor2_angle_text,
                             sensor3_angle_text);

    if ((report_length > 0) &&
        (report_length < (int32_t)sizeof(s_usb_tx_buffer))) {
        (void)CDC_Transmit_FS((uint8_t *)s_usb_tx_buffer,
                              (uint16_t)report_length);
    }

    s_app_state = MY_APP_STATE_WAIT_COMMAND;
}

/**
 * @brief  Khởi tạo tầng ứng dụng và module điều khiển yaw.
 */
void my_app_init(void)
{
    MyController_Status_t controller_status;

    s_is_controller_ready = false;
    s_has_sensor_init_error = false;
    s_has_init_error_been_reported = false;
    s_app_state = MY_APP_STATE_WAIT_COMMAND;

    controller_status = MyController_Init();
    if (controller_status == MY_CONTROLLER_ERR_SENSOR) {
        /*
         * Ba motor đã được khởi tạo trước nhánh AS5600, nên vẫn cho phép
         * lệnh chạy nhóm không cần phản hồi cảm biến.
         */
        s_is_controller_ready = true;
        s_has_sensor_init_error = true;
        return;
    }

    if (controller_status != MY_CONTROLLER_OK) {
        return;
    }

    s_is_controller_ready = true;
    (void)my_app_set_zero_from_sensor();
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

    if (s_is_controller_ready == false) {
        if (s_has_init_error_been_reported == false) {
            // Báo lỗi init một lần để terminal không bị spam liên tục.
            my_app_usb_send_text("ERR: controller init failed\r\n");
            s_has_init_error_been_reported = true;
        }
        return;
    }

    if ((s_has_sensor_init_error == true) &&
        (s_has_init_error_been_reported == false)) {
        my_app_usb_send_text(
            "WARN: AS5600 init failed, sensor report unavailable\r\n");
        s_has_init_error_been_reported = true;
        return;
    }

    if (s_app_state == MY_APP_STATE_WAIT_COMMAND) {
        my_app_process_usb_command();
        return;
    }

    if (s_app_state == MY_APP_STATE_WAIT_THREE_MOTOR) {
        my_app_process_three_motor_done();
        return;
    }

    my_app_process_motor_done();
}
