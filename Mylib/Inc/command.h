/**
 * @file    command.h
 * @brief   Giao dien phan tich lenh va truyen nhan USB CDC.
 * @author  Lap4all
 * @date    2026-05-17
 */

#ifndef INC_COMMAND_H_
#define INC_COMMAND_H_

#include "my_controller.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief Kich thuoc bo dem truyen USB CDC, tinh bang byte. */
#define COMMAND_USB_TX_BUFFER_SIZE  256U

/** @brief Kich thuoc bo dem nhan lenh USB CDC, tinh bang byte. */
#define COMMAND_USB_RX_BUFFER_SIZE   64U

/** @brief Kich thuoc chuoi cho mot truong goc, tinh ca ky tu ket thuc null. */
#define COMMAND_ANGLE_TEXT_SIZE      16U

/**
 * @brief  Kiem tra USB CDC da cau hinh va san sang truyen goi moi chua.
 * @return true neu USB CDC co the truyen, nguoc lai false.
 */
bool    Command_UsbIsReady(void);

/**
 * @brief  Gui chuoi ket thuc null qua USB CDC voi gioi han do dai.
 * @param  text: Chuoi ket thuc null can gui.
 */
void    Command_UsbSendText(const char *text);

/**
 * @brief  Gui bo dem nhi phan qua USB CDC.
 * @param  buf: Con tro den du lieu can gui.
 * @param  length: So byte can gui.
 */
void    Command_UsbSendBuffer(uint8_t *buf, uint16_t length);

/**
 * @brief  Doc mot lenh tu bo dem nhan USB CDC.
 * @param  buffer: Bo dem dich chua du lieu nhan.
 * @param  buffer_size: Kich thuoc bo dem dich, tinh bang byte.
 * @param  length: So byte thuc te nhan duoc.
 * @return 1 neu co lenh cho xu ly, 0 neu bo dem trong.
 */
uint8_t Command_ReadUsb(uint8_t *buffer, uint16_t buffer_size,
                        uint16_t *length);

/**
 * @brief  Nhan dien lenh "zero" khong phan biet chu hoa, chu thuong.
 * @param  buffer: Cac byte lenh nhan duoc.
 * @param  length: So byte trong buffer.
 * @return true neu lenh yeu cau dat lai yaw zero, nguoc lai false.
 */
bool Command_IsZeroCommand(const uint8_t *buffer, uint16_t length);

/**
 * @brief  Phan tich lenh yaw dang so thap phan sang centi-do.
 * @param  buffer: Cac byte lenh nhan duoc.
 * @param  length: So byte trong buffer.
 * @param  target_angle_cdeg: Goc sau khi phan tich, tinh bang centi-do.
 * @return true neu lenh la goc thap phan hop le, nguoc lai false.
 * @note   Chi giu hai chu so thap phan de khop voi cach luu centi-do.
 */
bool Command_ParseTargetAngleCdeg(const uint8_t *buffer, uint16_t length,
                                  int32_t *target_angle_cdeg);

/**
 * @brief  Phan tich lenh gom ba goc muc tieu de chay dong thoi ba motor.
 * @param  buffer: Cac byte lenh nhan duoc.
 * @param  length: So byte trong buffer.
 * @param  command: Noi luu ba goc muc tieu, tinh bang centi-do.
 * @return true neu lenh co dung ba gia tri goc hop le.
 * @note   Chap nhan dang "10 20 30", "10,20,30" hoac "10;20;30".
 */
bool Command_ParseThreeMotorCommand(
    const uint8_t *buffer,
    uint16_t length,
    MyController_ThreeMotorMoveCommand_t *command);

/**
 * @brief  Dinh dang goc centi-do thanh chuoi do co co dinh hai chu so le.
 * @param  angle_cdeg: Goc tinh bang centi-do.
 * @param  buffer: Bo dem chuoi dich.
 * @param  buffer_size: Kich thuoc bo dem dich, tinh bang byte.
 * @note   Dinh dang bang so nguyen de khong can bat ho tro printf so thuc.
 */
void Command_FormatAngleDeg(int32_t angle_cdeg, char *buffer,
                            uint16_t buffer_size);

/**
 * @brief  Dinh dang ket qua doc mot AS5600 thanh goc hoac ma loi.
 * @param  angle_cdeg: Goc doc duoc, tinh bang centi-do.
 * @param  sensor_status: Ma loi kenh TCA9548A/AS5600, 0 la doc thanh cong.
 * @param  buffer: Bo dem chuoi dich.
 * @param  buffer_size: Kich thuoc bo dem dich, tinh bang byte.
 */
void Command_FormatSensorReadout(int32_t angle_cdeg, int8_t sensor_status,
                                 char *buffer, uint16_t buffer_size);

#endif /* INC_COMMAND_H_ */
