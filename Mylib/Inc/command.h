/**
 * @file    command.h
 * @brief   Giao diện phân tích lệnh và truyền nhận USB CDC.
 * @author  Lap4all
 * @date    2026-06-10
 */

#ifndef INC_COMMAND_H_
#define INC_COMMAND_H_

#include "my_controller.h"
#include "protocol.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief Kích thước bộ đệm truyền USB CDC, tính bằng byte. */
#define COMMAND_USB_TX_BUFFER_SIZE  256U

/** @brief Kích thước bộ đệm nhận lệnh USB CDC, tính bằng byte. */
#define COMMAND_USB_RX_BUFFER_SIZE   128U

/** @brief Kích thước ring buffer nhận byte từ USB CDC. */
#define USB_RX_LINE_BUFFER_SIZE      256U

bool Command_UsbIsReady(void);
void Command_UsbSendText(const char *text);
void Command_UsbSendBuffer(uint8_t *buf, uint16_t length);

/* ----------------------------------------------------------------------------
 * Response sender (contract nhóm 4 — Docs/protocol.md §14.3)
 *   ACK : $A,<seq>\n
 *   DONE: $D,<seq>,<angle1_x100>,<angle2_x100>,<angle3_x100>\n
 *   ERR : $E,<seq>,<CODE>\n   (CODE: FORMAT | RANGE | QUEUE_FULL | MOTOR)
 * -------------------------------------------------------------------------- */

/** @brief Báo đã nhận và đẩy lệnh vào queue thành công. */
void Response_SendACK(uint8_t seq);

/** @brief Báo motion task đã chạy xong một lệnh, echo lại góc x100 thô. */
void Response_SendDONE(const RobotCommand *cmd);

/** @brief Báo lỗi cho một seq với mã lỗi dạng chuỗi. */
void Response_SendERR(uint8_t seq, const char *err_code);

/**
 * @brief  Đẩy byte thô từ CDC_Receive_FS vào ring buffer nội bộ.
 * @note   Gọi từ ngắt USB — chỉ lưu byte, không parse, không điều khiển motor.
 */
void USB_RX_PushBytes(uint8_t *data, uint32_t len);

/**
 * @brief  Lấy một dòng hoàn chỉnh (kết thúc bằng '\\n') ra khỏi ring buffer.
 * @param  line:    Buffer đầu ra, được null-terminate, không chứa '\\n'/'\\r'.
 * @param  max_len: Kích thước buffer đầu ra tính bằng byte (bao gồm '\\0').
 * @return 1 nếu có dòng hoàn chỉnh, 0 nếu chưa đủ dữ liệu.
 */
int USB_RX_GetLine(char *line, uint16_t max_len);

bool Command_IsZeroCommand(const uint8_t *buffer, uint16_t length);
bool Command_IsGoCommand(const uint8_t *buffer, uint16_t length);
bool Command_IsStopCommand(const uint8_t *buffer, uint16_t length);

/**
 * @brief  Phân tích lệnh gồm ba góc mục tiêu để chạy đồng thời ba motor.
 * @note   Chấp nhận dạng "10 20 30", "10,20,30" hoặc "10;20;30".
 */
bool Command_ParseThreeMotorCommand(const uint8_t *buffer, uint16_t length,
                                    MyController_ThreeMotorMoveCommand_t *command);

/* String builder helpers */
void Command_AppendChar(char *buffer, uint16_t buffer_size, uint16_t *index,
                        char value);
void Command_AppendText(char *buffer, uint16_t buffer_size, uint16_t *index,
                        const char *text);
void Command_AppendUnsigned(char *buffer, uint16_t buffer_size, uint16_t *index,
                            uint32_t value);
void Command_AppendSigned(char *buffer, uint16_t buffer_size, uint16_t *index,
                          int32_t value);
void Command_AppendAngleDeg(int32_t angle_cdeg, char *buffer,
                            uint16_t buffer_size, uint16_t *index);

#endif /* INC_COMMAND_H_ */
