/**
 * @file    my_app.c
 * @brief   Tang ung dung xu ly lenh USB CDC cho yaw va ba motor.
 * @author  Lap4all
 * @date    2026-05-14
 */

#include "my_app.h"
#include "my_controller.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/** @brief Kich thuoc bo dem truyen USB CDC. */
#define MY_APP_USB_TX_BUFFER_SIZE      256U

/** @brief Kich thuoc bo dem nhan lenh USB CDC. */
#define MY_APP_USB_RX_BUFFER_SIZE      64U

/** @brief Kich thuoc chuoi cho mot truong goc. */
#define MY_APP_ANGLE_TEXT_BUFFER_SIZE  16U

/**
 * @brief  Cac trang thai chinh cua may trang thai ung dung.
 */
typedef enum {
    MY_APP_STATE_WAIT_COMMAND = 0,
    MY_APP_STATE_WAIT_MOTOR,
    MY_APP_STATE_WAIT_THREE_MOTOR,
} my_app_state_t;

extern USBD_HandleTypeDef hUsbDeviceFS;

/** @brief Trang thai hien tai cua xu ly lenh. */
static my_app_state_t s_app_state = MY_APP_STATE_WAIT_COMMAND;

/** @brief Ngu canh lenh yaw dang cho hoan tat. */
static MyController_MoveContext_t s_move_context;

/** @brief Ngu canh lenh chay dong thoi ba motor dang cho hoan tat. */
static MyController_ThreeMotorMoveContext_t s_three_motor_context;

/** @brief Co cho biet controller da san sang xu ly lenh. */
static bool s_is_controller_ready = false;

/** @brief Co cho biet nhanh cam bien AS5600 khong khoi tao duoc. */
static bool s_has_sensor_init_error = false;

/** @brief Co chan bao loi init lap lai qua USB. */
static bool s_has_init_error_been_reported = false;

/** @brief Bo dem truyen USB dung chung cho report. */
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
static void my_app_format_sensor_readout(int32_t angle_cdeg,
                                         int8_t sensor_status,
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
 * @brief  Kiem tra USB CDC da cau hinh va san sang truyen goi moi chua.
 * @return true neu USB CDC co the truyen, nguoc lai false.
 */
static bool my_app_usb_is_ready(void)
{
    USBD_CDC_HandleTypeDef *cdc_handle;

    // USB phai duoc host cau hinh xong truoc khi goi CDC_Transmit_FS().
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        return false;
    }

    cdc_handle = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    // TxState khac 0 nghia la goi truoc van dang duoc USB stack xu ly.
    if ((cdc_handle == NULL) || (cdc_handle->TxState != 0U)) {
        return false;
    }

    return true;
}

/**
 * @brief  Gui chuoi ket thuc null qua USB CDC voi gioi han do dai.
 * @param  text: Chuoi ket thuc null can gui.
 */
static void my_app_usb_send_text(const char *text)
{
    uint16_t text_length = 0U;

    if ((text == NULL) || (my_app_usb_is_ready() == false)) {
        return;
    }

    // Gioi han do dai de khong vuot qua bo dem truyen USB dung chung.
    while ((text[text_length] != '\0') &&
           (text_length < (MY_APP_USB_TX_BUFFER_SIZE - 1U))) {
        text_length++;
    }

    if (text_length > 0U) {
        (void)CDC_Transmit_FS((uint8_t *)text, text_length);
    }
}

/**
 * @brief  Nhan dien lenh "zero" khong phan biet chu hoa, chu thuong.
 * @param  buffer: Cac byte lenh nhan duoc.
 * @param  length: So byte trong buffer.
 * @return true neu lenh yeu cau dat lai yaw zero, nguoc lai false.
 */
static bool my_app_is_zero_command(const uint8_t *buffer, uint16_t length)
{
    uint16_t start_index = 0U;
    uint16_t end_index = length;

    // Chap nhan lenh tu terminal co khoang trang hoac CR/LF o dau dong.
    while ((start_index < length) &&
           ((buffer[start_index] == ' ') ||
            (buffer[start_index] == '\t') ||
            (buffer[start_index] == '\r') ||
            (buffer[start_index] == '\n'))) {
        start_index++;
    }

    // Terminal thuong gui kem CR/LF, nen bo phan duoi truoc khi so khop.
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
 * @brief  Phan tich lenh yaw dang so thap phan sang centi-do.
 * @param  buffer: Cac byte lenh nhan duoc.
 * @param  length: So byte trong buffer.
 * @param  target_angle_cdeg: Goc sau khi phan tich, tinh bang centi-do.
 * @return true neu lenh la goc thap phan hop le, nguoc lai false.
 * @note   Chi giu hai chu so thap phan de khop voi cach luu centi-do.
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
     * Module controller nhan centi-do, nen app chi chap nhan toi da hai chu
     * so thap phan de tranh lam nguoi dung tuong he thong giu do phan giai hon.
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
 * @brief  Kiem tra ky tu phan tach giua cac goc trong lenh ba motor.
 * @param  value: Ky tu can kiem tra.
 * @return true neu la khoang trang, dau phay hoac dau cham phay.
 */
static bool my_app_is_three_motor_separator(uint8_t value)
{
    return ((value == ' ') ||
            (value == '\t') ||
            (value == ',') ||
            (value == ';'));
}

/**
 * @brief  Bo qua cac ky tu phan tach trong lenh ba motor.
 * @param  buffer: Cac byte lenh nhan duoc.
 * @param  length: So byte trong buffer.
 * @param  index: Vi tri doc hien tai, duoc cap nhat sau khi bo qua.
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
 * @brief  Phan tich mot truong goc duong trong lenh ba motor.
 * @param  buffer: Cac byte lenh nhan duoc.
 * @param  length: So byte trong buffer.
 * @param  index: Vi tri doc hien tai, duoc cap nhat sau khi doc truong.
 * @param  angle_cdeg: Goc sau khi phan tich, tinh bang centi-do.
 * @return true neu truong hien tai la mot so goc hop le.
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
 * @brief  Phan tich lenh gom ba goc muc tieu de chay dong thoi ba motor.
 * @param  buffer: Cac byte lenh nhan duoc.
 * @param  length: So byte trong buffer.
 * @param  command: Noi luu ba goc muc tieu, tinh bang centi-do.
 * @return true neu lenh co dung ba gia tri goc hop le.
 * @note   Chap nhan dang "10 20 30", "10,20,30" hoac "10;20;30".
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
 * @brief  Dinh dang goc centi-do thanh chuoi do co co dinh hai chu so le.
 * @param  angle_cdeg: Goc tinh bang centi-do.
 * @param  buffer: Bo dem chuoi dich.
 * @param  buffer_size: Kich thuoc bo dem dich, tinh bang byte.
 * @note   Dinh dang bang so nguyen de khong can bat ho tro printf so thuc.
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
 * @brief  Dinh dang ket qua doc mot AS5600 thanh goc hoac ma loi.
 * @param  angle_cdeg: Goc doc duoc, tinh bang centi-do.
 * @param  sensor_status: Ma loi kenh TCA9548A/AS5600, 0 la doc thanh cong.
 * @param  buffer: Bo dem chuoi dich.
 * @param  buffer_size: Kich thuoc bo dem dich, tinh bang byte.
 */
static void my_app_format_sensor_readout(int32_t angle_cdeg,
                                         int8_t sensor_status,
                                         char *buffer,
                                         uint16_t buffer_size)
{
    if ((buffer == NULL) || (buffer_size == 0U)) {
        return;
    }

    if (sensor_status == 0) {
        my_app_format_angle_deg(angle_cdeg, buffer, buffer_size);
        return;
    }

    (void)snprintf(buffer, buffer_size, "ERR%d", (int)sensor_status);
}

/**
 * @brief  Dat goc cam bien hien tai lam yaw zero va bao ket qua qua USB.
 * @return true neu controller cap nhat zero thanh cong.
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
 * @brief  Bat dau chay motor tu yaw hien tai toi yaw muc tieu.
 * @param  target_yaw_cdeg: Goc yaw muc tieu tuyet doi, tinh bang centi-do.
 * @return true neu lenh chay duoc controller chap nhan.
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
     * Ke ca khi target_steps bang 0, app van di qua trang thai WAIT_MOTOR de
     * tao mot report cung dinh dang voi cac lenh co chay motor.
     */
    s_app_state = MY_APP_STATE_WAIT_MOTOR;
    return true;
}

/**
 * @brief  Bat dau lenh chay dong thoi ba motor tu USB CDC.
 * @param  command: Ba goc can chay theo thu tu motor1, motor2, motor3.
 * @return true neu lenh duoc controller chap nhan.
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
     * Lenh toan so 0 van di qua trang thai cho de terminal nhan duoc report
     * cung dinh dang voi cac lenh co tao xung STEP.
     */
    s_app_state = MY_APP_STATE_WAIT_THREE_MOTOR;
    return true;
}

/**
 * @brief  Xu ly mot lenh USB CDC dang cho khi ung dung o trang thai ranh.
 * @note   Ba goc cho ba motor la goc muc tieu tuyet doi trong khoang 0..360.
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

    // Lenh zero duoc uu tien vi khong phai la mot gia tri goc so.
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
 * @brief  Hoan tat mot lan chay motor va bao sai so cam bien qua USB CDC.
 * @note   Cong cu ghi CSV can dung ten cac truong report duoc phat ra o day.
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

    // Giu nguyen ten truong vi read_uart.py dang dung format nay de ghi CSV.
    if ((report_length > 0) &&
        (report_length < (int32_t)sizeof(s_usb_tx_buffer))) {
        (void)CDC_Transmit_FS((uint8_t *)s_usb_tx_buffer,
                              (uint16_t)report_length);
    }

    s_app_state = MY_APP_STATE_WAIT_COMMAND;
}

/**
 * @brief  Hoan tat lenh ba motor va bao target, delta, so buoc da chay.
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
    my_app_format_sensor_readout(sensor_readout.sensor1_angle_cdeg,
                                 sensor_readout.sensor1_status,
                                 sensor1_angle_text,
                                 sizeof(sensor1_angle_text));
    my_app_format_sensor_readout(sensor_readout.sensor2_angle_cdeg,
                                 sensor_readout.sensor2_status,
                                 sensor2_angle_text,
                                 sizeof(sensor2_angle_text));
    my_app_format_sensor_readout(sensor_readout.sensor3_angle_cdeg,
                                 sensor_readout.sensor3_status,
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
                             "as1=%s,as1_st=%d,"
                             "as2=%s,as2_st=%d,"
                             "as3=%s,as3_st=%d\r\n",
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
                             (int)sensor_readout.sensor1_status,
                             sensor2_angle_text,
                             (int)sensor_readout.sensor2_status,
                             sensor3_angle_text,
                             (int)sensor_readout.sensor3_status);

    if ((report_length > 0) &&
        (report_length < (int32_t)sizeof(s_usb_tx_buffer))) {
        (void)CDC_Transmit_FS((uint8_t *)s_usb_tx_buffer,
                              (uint16_t)report_length);
    }

    s_app_state = MY_APP_STATE_WAIT_COMMAND;
}

/**
 * @brief  Khoi tao tang ung dung va module dieu khien yaw.
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
         * Ba motor da duoc khoi tao truoc nhanh AS5600, nen van cho phep
         * lenh chay nhom khong can phan hoi cam bien.
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
 * @brief  Chay may trang thai ung dung theo kieu khong chan.
 * @note   Goi ham nay lap lai trong vong lap main.
 */
void my_app_process(void)
{
    if (my_app_usb_is_ready() == false) {
        return;
    }

    if (s_is_controller_ready == false) {
        if (s_has_init_error_been_reported == false) {
            // Bao loi init mot lan de terminal khong bi spam lien tuc.
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
