/**
 * @file    my_queue.c
 * @brief   Hàng đợi vòng lưu RobotCommand cho motion task.
 * @author  Lap4all
 * @date    2026-06-11
 */

#include "my_queue.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ============================================================================
 * Private variables
 * ============================================================================ */

/** @brief Mảng lưu các lệnh — circular buffer. */
static RobotCommand s_cmds[CMD_QUEUE_SIZE];

/** @brief Chỉ số đọc tiếp theo. */
static uint8_t s_head;

/** @brief Chỉ số ghi tiếp theo. */
static uint8_t s_tail;

/** @brief Số lệnh hiện có trong hàng đợi. */
static uint8_t s_count;

/** @brief Cờ báo có lệnh bị drop do queue đầy, tự reset khi MyQueue_WasDropped() đọc. */
static bool s_drop_flag;

/* ============================================================================
 * Exported function definitions
 * ============================================================================ */

void MyQueue_Init(void)
{
    s_head      = 0U;
    s_tail      = 0U;
    s_count     = 0U;
    s_drop_flag = false;
}

bool MyQueue_Push(const RobotCommand *cmd)
{
    if (cmd == NULL)
    {
        return false;
    }

    if (s_count >= CMD_QUEUE_SIZE)
    {
        s_drop_flag = true;
        return false;
    }

    s_cmds[s_tail] = *cmd;
    s_tail = (s_tail + 1U) % CMD_QUEUE_SIZE;
    s_count++;
    return true;
}

bool MyQueue_Pop(RobotCommand *cmd)
{
    if ((cmd == NULL) || (s_count == 0U))
    {
        return false;
    }

    *cmd   = s_cmds[s_head];
    s_head = (s_head + 1U) % CMD_QUEUE_SIZE;
    s_count--;
    return true;
}

bool MyQueue_IsFull(void)
{
    return (s_count >= CMD_QUEUE_SIZE);
}

bool MyQueue_IsEmpty(void)
{
    return (s_count == 0U);
}

uint8_t MyQueue_Count(void)
{
    return s_count;
}

void MyQueue_Flush(void)
{
    s_head      = 0U;
    s_tail      = 0U;
    s_count     = 0U;
    s_drop_flag = false;
}

bool MyQueue_WasDropped(void)
{
    bool v      = s_drop_flag;
    s_drop_flag = false;
    return v;
}
