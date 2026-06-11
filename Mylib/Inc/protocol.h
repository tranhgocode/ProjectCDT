/*
 * protocol.h
 *
 *  Created on: May 17, 2026
 *      Author: Lap4all
 */

#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

#include <stdint.h>

typedef struct
{
    uint8_t seq;
    int32_t angle1_x100;
    int32_t angle2_x100;
    int32_t angle3_x100;
} RobotCommand;

typedef enum
{
    PROTO_OK = 0,
    PROTO_ERR_FORMAT,
    PROTO_ERR_RANGE,
    PROTO_ERR_UNKNOWN_TYPE
} ProtocolStatus;

/**
 * Decode một dòng text thành RobotCommand.
 * Format: $M,<seq>,<angle1_x100>,<angle2_x100>,<angle3_x100>
 * Góc hợp lệ: -18000 <= angle_x100 <= 18000
 */
ProtocolStatus Protocol_DecodeMoveLine(const char *line, RobotCommand *cmd);

#endif /* INC_PROTOCOL_H_ */
