/*
 * protocol.c
 *
 *  Created on: May 17, 2026
 *      Author: Lap4all
 */

#include "protocol.h"
#include <stdlib.h>

#define ANGLE_MIN (-18000L)
#define ANGLE_MAX  18000L

/* Parse one comma-separated signed integer field.
 * p     : pointer to start of field text
 * out   : parsed value
 * next  : set to character after the digits (should be ',' or '\0'/'\r'/'\n')
 * Returns 0 on success, -1 if no digits were found or a decimal point follows. */
static int parse_field(const char *p, long *out, const char **next)
{
    char *end;
    if (*p == '\0') return -1;
    *out = strtol(p, &end, 10);
    if (end == p) return -1;          /* không có chữ số nào */
    if (*end == '.') return -1;       /* float — sai định dạng */
    *next = end;
    return 0;
}

ProtocolStatus Protocol_DecodeMoveLine(const char *line, RobotCommand *cmd)
{
    if (line == NULL || cmd == NULL) return PROTO_ERR_FORMAT;

    /* Mặc định seq = 0 để nhánh lỗi (FORMAT khi chưa đọc được seq) vẫn echo
     * đúng theo contract: "hello" -> $E,0,FORMAT. */
    cmd->seq = 0U;

    /* Kiểm tra ký tự '$' đầu dòng */
    if (line[0] != '$') return PROTO_ERR_FORMAT;

    /* Đọc type và dấu phẩy ngay sau */
    char type = line[1];
    if (type == '\0' || line[2] != ',') return PROTO_ERR_FORMAT;

    if (type != 'M') return PROTO_ERR_UNKNOWN_TYPE;

    const char *p = &line[3]; /* trỏ vào seq */
    long seq, a1, a2, a3;
    const char *end;

    /* seq */
    if (parse_field(p, &seq, &end) != 0) return PROTO_ERR_FORMAT;
    if (*end != ',') return PROTO_ERR_FORMAT;
    if (seq < 0 || seq > 255) return PROTO_ERR_FORMAT;
    /* Giữ seq ngay khi đọc được để nhánh lỗi field sau ($M,2,9000,4550 ->
     * $E,2,FORMAT) hoặc lỗi range ($M,3,999999,.. -> $E,3,RANGE) echo đúng. */
    cmd->seq = (uint8_t)seq;
    p = end + 1;

    /* angle1 */
    if (parse_field(p, &a1, &end) != 0) return PROTO_ERR_FORMAT;
    if (*end != ',') return PROTO_ERR_FORMAT;
    p = end + 1;

    /* angle2 */
    if (parse_field(p, &a2, &end) != 0) return PROTO_ERR_FORMAT;
    if (*end != ',') return PROTO_ERR_FORMAT;
    p = end + 1;

    /* angle3 — phải là field cuối cùng */
    if (parse_field(p, &a3, &end) != 0) return PROTO_ERR_FORMAT;
    if (*end != '\0' && *end != '\r' && *end != '\n') return PROTO_ERR_FORMAT;

    /* Kiểm tra giới hạn góc */
    if (a1 < ANGLE_MIN || a1 > ANGLE_MAX) return PROTO_ERR_RANGE;
    if (a2 < ANGLE_MIN || a2 > ANGLE_MAX) return PROTO_ERR_RANGE;
    if (a3 < ANGLE_MIN || a3 > ANGLE_MAX) return PROTO_ERR_RANGE;

    cmd->seq         = (uint8_t)seq;
    cmd->angle1_x100 = (int32_t)a1;
    cmd->angle2_x100 = (int32_t)a2;
    cmd->angle3_x100 = (int32_t)a3;

    return PROTO_OK;
}
