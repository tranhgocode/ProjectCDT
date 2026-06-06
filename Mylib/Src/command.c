/**
 * @file    command.c
 * @brief   Phan tich cu phap lenh va giao tiep USB CDC.
 * @author  Lap4all
 * @date    2026-05-17
 */

#include "command.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

static bool command_is_three_motor_separator(uint8_t value);
static void command_skip_three_motor_separators(const uint8_t *buffer,
                                                uint16_t length,
                                                uint16_t *index);
static bool command_parse_angle_field_cdeg(const uint8_t *buffer,
                                           uint16_t length,
                                           uint16_t *index,
                                           int32_t *angle_cdeg);

/**
 * @brief  Kiem tra USB CDC da cau hinh va san sang truyen goi moi chua.
 * @return true neu USB CDC co the truyen, nguoc lai false.
 */
bool Command_UsbIsReady(void)
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
void Command_UsbSendText(const char *text)
{
    uint16_t text_length = 0U;

    if ((text == NULL) || (Command_UsbIsReady() == false)) {
        return;
    }

    // Gioi han do dai de khong vuot qua bo dem truyen USB dung chung.
    while ((text[text_length] != '\0') &&
           (text_length < (COMMAND_USB_TX_BUFFER_SIZE - 1U))) {
        text_length++;
    }

    if (text_length > 0U) {
        (void)CDC_Transmit_FS((uint8_t *)text, text_length);
    }
}

/**
 * @brief  Gui bo dem nhi phan qua USB CDC, co kiem tra TxState truoc khi gui.
 * @param  buf: Con tro den du lieu can gui.
 * @param  length: So byte can gui.
 */
void Command_UsbSendBuffer(uint8_t *buf, uint16_t length)
{
    if ((buf == NULL) || (length == 0U) || (Command_UsbIsReady() == false)) {
        return;
    }

    (void)CDC_Transmit_FS(buf, length);
}

/**
 * @brief  Doc mot lenh tu bo dem nhan USB CDC.
 * @param  buffer: Bo dem dich chua du lieu nhan.
 * @param  buffer_size: Kich thuoc bo dem dich, tinh bang byte.
 * @param  length: So byte thuc te nhan duoc.
 * @return 1 neu co lenh cho xu ly, 0 neu bo dem trong.
 */
uint8_t Command_ReadUsb(uint8_t *buffer, uint16_t buffer_size,
                        uint16_t *length)
{
    return CDC_ReadCommand(buffer, buffer_size, length);
}

/**
 * @brief  Nhan dien lenh "zero" khong phan biet chu hoa, chu thuong.
 * @param  buffer: Cac byte lenh nhan duoc.
 * @param  length: So byte trong buffer.
 * @return true neu lenh yeu cau dat lai yaw zero, nguoc lai false.
 */
bool Command_IsZeroCommand(const uint8_t *buffer, uint16_t length)
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
bool Command_ParseTargetAngleCdeg(const uint8_t *buffer,
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
static bool command_is_three_motor_separator(uint8_t value)
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
static void command_skip_three_motor_separators(const uint8_t *buffer,
                                                uint16_t length,
                                                uint16_t *index)
{
    while ((*index < length) &&
           (command_is_three_motor_separator(buffer[*index]) == true)) {
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
static bool command_parse_angle_field_cdeg(const uint8_t *buffer,
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
            frac_cdeg = (frac_cdeg * 10) + (int32_t)(buffer[*index] - '0');
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
bool Command_ParseThreeMotorCommand(const uint8_t *buffer,
                                    uint16_t length,
                                    MyController_ThreeMotorMoveCommand_t *command)
{
    uint16_t index = 0U;
    int32_t angles_cdeg[3] = {0};

    if ((buffer == NULL) || (command == NULL) || (length == 0U)) {
        return false;
    }

    for (uint8_t motor_index = 0U; motor_index < 3U; motor_index++) {
        command_skip_three_motor_separators(buffer, length, &index);

        if (command_parse_angle_field_cdeg(buffer,
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
           ((command_is_three_motor_separator(buffer[index]) == true) ||
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
void Command_FormatAngleDeg(int32_t angle_cdeg,
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
void Command_FormatSensorReadout(int32_t angle_cdeg,
                                 int8_t sensor_status,
                                 char *buffer,
                                 uint16_t buffer_size)
{
    if ((buffer == NULL) || (buffer_size == 0U)) {
        return;
    }

    if (sensor_status == 0) {
        Command_FormatAngleDeg(angle_cdeg, buffer, buffer_size);
        return;
    }

    (void)snprintf(buffer, buffer_size, "ERR%d", (int)sensor_status);
}
