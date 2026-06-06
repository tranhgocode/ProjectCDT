/**
 * @file    my_app.c
 * @brief   Tang ung dung xu ly lenh USB CDC cho yaw va ba motor.
 * @author  Lap4all
 * @date    2026-05-14
 */

#include "my_app.h"
#include "my_controller.h"
#include "command.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/**
 * @brief  Cac trang thai chinh cua may trang thai ung dung.
 */
typedef enum {
    MY_APP_STATE_WAIT_COMMAND = 0,
    MY_APP_STATE_WAIT_MOTOR,
    MY_APP_STATE_WAIT_THREE_MOTOR,
} my_app_state_t;

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
static char s_usb_tx_buffer[COMMAND_USB_TX_BUFFER_SIZE];

static bool my_app_set_zero_from_sensor(void);
static bool my_app_start_target_move(int32_t target_yaw_cdeg);
static bool my_app_start_three_motor_move(
    const MyController_ThreeMotorMoveCommand_t *command);
static void my_app_process_usb_command(void);
static void my_app_process_motor_done(void);
static void my_app_process_three_motor_done(void);

/**
 * @brief  Dat goc cam bien hien tai lam yaw zero va bao ket qua qua USB.
 * @return true neu controller cap nhat zero thanh cong.
 */
static bool my_app_set_zero_from_sensor(void)
{
    if (MyController_SetZeroFromSensor() != MY_CONTROLLER_OK) {
        Command_UsbSendText("ERR: cannot read sensor for zero\r\n");
        return false;
    }

    s_app_state = MY_APP_STATE_WAIT_COMMAND;
    Command_UsbSendText("zero_ok yaw_deg=0.00\r\n");
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
        Command_UsbSendText("ERR: cannot read start angle\r\n");
        return false;
    }

    if (controller_status == MY_CONTROLLER_ERR_MOTOR) {
        Command_UsbSendText("ERR: motor start failed\r\n");
        return false;
    }

    if (controller_status != MY_CONTROLLER_OK) {
        Command_UsbSendText("ERR: controller start failed\r\n");
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
        Command_UsbSendText("ERR: three-motor start failed\r\n");
        return false;
    }

    if (controller_status != MY_CONTROLLER_OK) {
        Command_UsbSendText("ERR: three-motor command failed\r\n");
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
    uint8_t command_buffer[COMMAND_USB_RX_BUFFER_SIZE];
    uint16_t command_length = 0U;
    int32_t target_yaw_cdeg = 0;
    MyController_ThreeMotorMoveCommand_t three_motor_command = {0};

    if (Command_ReadUsb(command_buffer,
                        sizeof(command_buffer),
                        &command_length) == 0U) {
        return;
    }

    // Lenh zero duoc uu tien vi khong phai la mot gia tri goc so.
    if (Command_IsZeroCommand(command_buffer, command_length) == true) {
        (void)my_app_set_zero_from_sensor();
        return;
    }

    if (Command_ParseThreeMotorCommand(command_buffer,
                                       command_length,
                                       &three_motor_command) == true) {
        (void)my_app_start_three_motor_move(&three_motor_command);
        return;
    }

    if (Command_ParseTargetAngleCdeg(command_buffer,
                                     command_length,
                                     &target_yaw_cdeg) == false) {
        Command_UsbSendText(
            "ERR: input must be zero, angle, or three angles in deg\r\n");
        return;
    }

    if ((target_yaw_cdeg < 0) ||
        (target_yaw_cdeg > MY_CONTROLLER_FULL_TURN_CDEG)) {
        Command_UsbSendText("ERR: input range is 0.00..360.00 deg\r\n");
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
    char input_angle_text[COMMAND_ANGLE_TEXT_SIZE];
    char start_angle_text[COMMAND_ANGLE_TEXT_SIZE];
    char end_angle_text[COMMAND_ANGLE_TEXT_SIZE];
    char delta_angle_text[COMMAND_ANGLE_TEXT_SIZE];
    char error_angle_text[COMMAND_ANGLE_TEXT_SIZE];

    if (MyController_IsTargetMotorRunning() == true) {
        return;
    }

    controller_status = MyController_FinishTargetMove(&s_move_context,
                                                      &move_result);
    if (controller_status == MY_CONTROLLER_ERR_SENSOR) {
        Command_UsbSendText("ERR: cannot read end angle\r\n");
        s_app_state = MY_APP_STATE_WAIT_COMMAND;
        return;
    }

    if (controller_status != MY_CONTROLLER_OK) {
        Command_UsbSendText("ERR: controller finish failed\r\n");
        s_app_state = MY_APP_STATE_WAIT_COMMAND;
        return;
    }

    Command_FormatAngleDeg(s_move_context.target_yaw_cdeg,
                           input_angle_text,
                           sizeof(input_angle_text));
    Command_FormatAngleDeg(s_move_context.start_sensor_yaw_cdeg,
                           start_angle_text,
                           sizeof(start_angle_text));
    Command_FormatAngleDeg(move_result.end_sensor_yaw_cdeg,
                           end_angle_text,
                           sizeof(end_angle_text));
    Command_FormatAngleDeg(move_result.sensor_delta_cdeg,
                           delta_angle_text,
                           sizeof(delta_angle_text));
    Command_FormatAngleDeg(move_result.error_cdeg,
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
        Command_UsbSendBuffer((uint8_t *)s_usb_tx_buffer,
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
    char motor1_angle_text[COMMAND_ANGLE_TEXT_SIZE];
    char motor2_angle_text[COMMAND_ANGLE_TEXT_SIZE];
    char motor3_angle_text[COMMAND_ANGLE_TEXT_SIZE];
    char motor1_delta_text[COMMAND_ANGLE_TEXT_SIZE];
    char motor2_delta_text[COMMAND_ANGLE_TEXT_SIZE];
    char motor3_delta_text[COMMAND_ANGLE_TEXT_SIZE];
    char sensor1_angle_text[COMMAND_ANGLE_TEXT_SIZE];
    char sensor2_angle_text[COMMAND_ANGLE_TEXT_SIZE];
    char sensor3_angle_text[COMMAND_ANGLE_TEXT_SIZE];
    unsigned long motor1_steps;
    unsigned long motor2_steps;
    unsigned long motor3_steps;

    if (MyController_IsThreeMotorMoveRunning() == true) {
        return;
    }

    controller_status = MyController_FinishThreeMotorMove(
        &s_three_motor_context);
    if (controller_status != MY_CONTROLLER_OK) {
        Command_UsbSendText("ERR: three-motor finish failed\r\n");
        s_app_state = MY_APP_STATE_WAIT_COMMAND;
        return;
    }

    controller_status = MyController_ReadThreeSensors(&sensor_readout);
    if (controller_status != MY_CONTROLLER_OK) {
        Command_UsbSendText("ERR: cannot read 3 AS5600 sensors\r\n");
        s_app_state = MY_APP_STATE_WAIT_COMMAND;
        return;
    }

    Command_FormatAngleDeg(s_three_motor_context.motor1_angle_cdeg,
                           motor1_angle_text,
                           sizeof(motor1_angle_text));
    Command_FormatAngleDeg(s_three_motor_context.motor2_angle_cdeg,
                           motor2_angle_text,
                           sizeof(motor2_angle_text));
    Command_FormatAngleDeg(s_three_motor_context.motor3_angle_cdeg,
                           motor3_angle_text,
                           sizeof(motor3_angle_text));
    Command_FormatAngleDeg(s_three_motor_context.motor1_delta_cdeg,
                           motor1_delta_text,
                           sizeof(motor1_delta_text));
    Command_FormatAngleDeg(s_three_motor_context.motor2_delta_cdeg,
                           motor2_delta_text,
                           sizeof(motor2_delta_text));
    Command_FormatAngleDeg(s_three_motor_context.motor3_delta_cdeg,
                           motor3_delta_text,
                           sizeof(motor3_delta_text));
    Command_FormatSensorReadout(sensor_readout.sensor1_angle_cdeg,
                                sensor_readout.sensor1_status,
                                sensor1_angle_text,
                                sizeof(sensor1_angle_text));
    Command_FormatSensorReadout(sensor_readout.sensor2_angle_cdeg,
                                sensor_readout.sensor2_status,
                                sensor2_angle_text,
                                sizeof(sensor2_angle_text));
    Command_FormatSensorReadout(sensor_readout.sensor3_angle_cdeg,
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
        Command_UsbSendBuffer((uint8_t *)s_usb_tx_buffer,
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
    if (Command_UsbIsReady() == false) {
        return;
    }

    if (s_is_controller_ready == false) {
        if (s_has_init_error_been_reported == false) {
            // Bao loi init mot lan de terminal khong bi spam lien tuc.
            Command_UsbSendText("ERR: controller init failed\r\n");
            s_has_init_error_been_reported = true;
        }
        return;
    }

    if ((s_has_sensor_init_error == true) &&
        (s_has_init_error_been_reported == false)) {
        Command_UsbSendText(
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
