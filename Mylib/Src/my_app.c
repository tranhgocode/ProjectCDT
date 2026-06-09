/**
 * @file    my_app.c
 * @brief   Tang ung dung xu ly lenh USB CDC cho yaw va ba motor.
 * @author  Lap4all
 * @date    2026-05-14
 */

#include "my_app.h"
#include "my_controller.h"
#include "command.h"
#include "traj_runner.h"
#include "my_queue.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief  Cac trang thai chinh cua may trang thai ung dung.
 */
typedef enum {
    MY_APP_STATE_WAIT_COMMAND = 0,
    MY_APP_STATE_WAIT_MOTOR,
    MY_APP_STATE_WAIT_THREE_MOTOR,
    MY_APP_STATE_TRAJ_RUNNING,    /**< Dang thuc thi quy dao tu PC. */
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

/** @brief Vi tri ghi hien tai trong s_usb_tx_buffer, dung chung cho report. */
static uint16_t s_report_length = 0U;

static void s_buf_text(const char *text)
{
    Command_AppendText(s_usb_tx_buffer, sizeof(s_usb_tx_buffer),
                       &s_report_length, text);
}

static void s_buf_unsigned(uint32_t value)
{
    Command_AppendUnsigned(s_usb_tx_buffer, sizeof(s_usb_tx_buffer),
                           &s_report_length, value);
}

static void s_buf_signed(int32_t value)
{
    Command_AppendSigned(s_usb_tx_buffer, sizeof(s_usb_tx_buffer),
                         &s_report_length, value);
}

static void s_buf_angle(int32_t angle_cdeg)
{
    Command_AppendAngleDeg(angle_cdeg, s_usb_tx_buffer,
                           sizeof(s_usb_tx_buffer), &s_report_length);
}

static void s_buf_sensor(int32_t angle_cdeg, int8_t status)
{
    if (status == 0) {
        s_buf_angle(angle_cdeg);
    } else {
        s_buf_text("ERR");
        s_buf_signed((int32_t)status);
    }
}

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

    /*
     * Byte dau la 0xAA: du lieu frame nhi phan, chuyen toan bo vao parser.
     * Khong xu ly them vi frame co the bi cat thanh nhieu goi USB.
     */
    if (command_buffer[0] == MY_QUEUE_FRAME_HEADER_0) {
        MyRunner_FeedBytes(command_buffer, command_length);
        return;
    }

    // Lenh STOP: dung khan cap quy dao dang chay hoac huy bo truoc khi GO.
    if (Command_IsStopCommand(command_buffer, command_length) == true) {
        MyRunner_Stop();
        s_app_state = MY_APP_STATE_WAIT_COMMAND;
        return;
    }

    // Lenh GO: bat dau thuc thi quy dao tu queue da duoc nap san.
    if (Command_IsGoCommand(command_buffer, command_length) == true) {
        if (MyQueue_IsEmpty()) {
            Command_UsbSendText("ERR: traj queue empty, send frames first\r\n");
        } else {
            MyRunner_Start();
            s_app_state = MY_APP_STATE_TRAJ_RUNNING;
        }
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

    s_usb_tx_buffer[0] = '\0';
    s_report_length = 0U;
    s_buf_text("input_deg=");   s_buf_angle(s_move_context.target_yaw_cdeg);
    s_buf_text(",start_deg=");  s_buf_angle(s_move_context.start_sensor_yaw_cdeg);
    s_buf_text(",end_deg=");    s_buf_angle(move_result.end_sensor_yaw_cdeg);
    s_buf_text(",delta_deg=");  s_buf_angle(move_result.sensor_delta_cdeg);
    s_buf_text(",error_deg=");  s_buf_angle(move_result.error_cdeg);
    s_buf_text(",steps=");      s_buf_unsigned(s_move_context.target_steps);
    s_buf_text("\r\n");

    // Giu nguyen ten truong vi read_uart.py dang dung format nay de ghi CSV.
    if ((s_report_length > 0U) &&
        (s_report_length < (uint16_t)sizeof(s_usb_tx_buffer))) {
        Command_UsbSendBuffer((uint8_t *)s_usb_tx_buffer, s_report_length);
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

    if (MyController_IsThreeMotorMoveRunning() == true) {
        return;
    }

    controller_status = MyController_FinishThreeMotorMove(&s_three_motor_context);
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

    s_usb_tx_buffer[0] = '\0';
    s_report_length = 0U;
    s_buf_text("three_ok,m1_t="); s_buf_angle(s_three_motor_context.motor1_angle_cdeg);
    s_buf_text(",m1_d=");         s_buf_angle(s_three_motor_context.motor1_delta_cdeg);
    s_buf_text(",m1_s=");         s_buf_unsigned(s_three_motor_context.motor1_target_steps);
    s_buf_text(",m2_t=");         s_buf_angle(s_three_motor_context.motor2_angle_cdeg);
    s_buf_text(",m2_d=");         s_buf_angle(s_three_motor_context.motor2_delta_cdeg);
    s_buf_text(",m2_s=");         s_buf_unsigned(s_three_motor_context.motor2_target_steps);
    s_buf_text(",m3_t=");         s_buf_angle(s_three_motor_context.motor3_angle_cdeg);
    s_buf_text(",m3_d=");         s_buf_angle(s_three_motor_context.motor3_delta_cdeg);
    s_buf_text(",m3_s=");         s_buf_unsigned(s_three_motor_context.motor3_target_steps);
    s_buf_text(",as1=");          s_buf_sensor(sensor_readout.sensor1_angle_cdeg, sensor_readout.sensor1_status);
    s_buf_text(",as1_st=");       s_buf_signed((int32_t)sensor_readout.sensor1_status);
    s_buf_text(",as2=");          s_buf_sensor(sensor_readout.sensor2_angle_cdeg, sensor_readout.sensor2_status);
    s_buf_text(",as2_st=");       s_buf_signed((int32_t)sensor_readout.sensor2_status);
    s_buf_text(",as3=");          s_buf_sensor(sensor_readout.sensor3_angle_cdeg, sensor_readout.sensor3_status);
    s_buf_text(",as3_st=");       s_buf_signed((int32_t)sensor_readout.sensor3_status);
    s_buf_text("\r\n");

    if ((s_report_length > 0U) &&
        (s_report_length < (uint16_t)sizeof(s_usb_tx_buffer))) {
        Command_UsbSendBuffer((uint8_t *)s_usb_tx_buffer, s_report_length);
    }

    (void)MyController_ResetThreeSensorZero();
    s_app_state = MY_APP_STATE_WAIT_COMMAND;
}

/**
 * @brief  Xu ly trang thai thuc thi quy dao: nap frame moi va kiem tra ket thuc.
 */
static void my_app_process_traj(void)
{
    uint8_t  traj_buf[COMMAND_USB_RX_BUFFER_SIZE];
    uint16_t traj_len = 0U;

    /* Nhan va phan loai du lieu USB trong khi dang chay quy dao. */
    if (Command_ReadUsb(traj_buf, sizeof(traj_buf), &traj_len) != 0U) {
        if (Command_IsStopCommand(traj_buf, traj_len) == true) {
            MyRunner_Stop();
            s_app_state = MY_APP_STATE_WAIT_COMMAND;
            return;
        }
        /* Tat ca du lieu khac (frame bo sung tu PC) dua vao parser. */
        MyRunner_FeedBytes(traj_buf, traj_len);
    }

    MyRunner_Process();

    /* Runner tu chuyen ve IDLE khi queue can; dong bo trang thai my_app. */
    if (MyRunner_IsActive() == false) {
        s_app_state = MY_APP_STATE_WAIT_COMMAND;
    }
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

    MyRunner_Init();

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

    if (s_app_state == MY_APP_STATE_TRAJ_RUNNING) {
        my_app_process_traj();
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
