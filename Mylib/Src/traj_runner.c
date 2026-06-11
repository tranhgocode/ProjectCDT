/**
 * @file    traj_runner.c
 * @brief   Thực thi quỹ đạo: lấy frame từ queue, chạy trap profile, báo cáo.
 * @author  Lap4all
 * @date    2026-06-10
 */

#include "traj_runner.h"
#include "my_queue.h"
#include "my_controller.h"
#include "my_app.h"
#include "command.h"
#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * Private variables
 * ============================================================================ */

static MyRunner_State_t                     s_state;

/** @brief Vị trí phần mềm sau frame gần nhất, dùng để báo cáo ack. */
static int32_t                              s_pos1_cdeg;
static int32_t                              s_pos2_cdeg;
static int32_t                              s_pos3_cdeg;

/** @brief Ngữ cảnh trap của frame đang chạy. */
static MyController_ThreeMotorMoveContext_t s_trap_ctx;

/** @brief true khi đang có trap move chạy, false khi rảnh. */
static bool                                 s_trap_active;

/* ============================================================================
 * Private function prototypes
 * ============================================================================ */

static void prv_stop_all_motors(void);
static void prv_send_report(void);

/* ============================================================================
 * Private function definitions
 * ============================================================================ */

static void prv_stop_all_motors(void)
{
    TMC2209_Stop(&motor1);
    TMC2209_Stop(&motor2);
    TMC2209_Stop(&motor3);
}

static void prv_send_report(void)
{
    char     buf[COMMAND_USB_TX_BUFFER_SIZE];
    uint16_t idx = 0U;

    if (!Command_UsbIsReady())
    {
        return;
    }

    buf[0] = '\0';
    Command_AppendText(buf, sizeof(buf), &idx, "ack,q=");
    Command_AppendUnsigned(buf, sizeof(buf), &idx, (uint32_t)MyQueue_Count());
    Command_AppendText(buf, sizeof(buf), &idx, ",m1=");
    Command_AppendAngleDeg(s_pos1_cdeg, buf, sizeof(buf), &idx);
    Command_AppendText(buf, sizeof(buf), &idx, ",m2=");
    Command_AppendAngleDeg(s_pos2_cdeg, buf, sizeof(buf), &idx);
    Command_AppendText(buf, sizeof(buf), &idx, ",m3=");
    Command_AppendAngleDeg(s_pos3_cdeg, buf, sizeof(buf), &idx);
    if (MyQueue_WasDropped())
    {
        Command_AppendText(buf, sizeof(buf), &idx, ",drop=1");
    }
    Command_AppendText(buf, sizeof(buf), &idx, "\r\n");

    if (idx > 0U)
    {
        Command_UsbSendBuffer((uint8_t *)buf, idx);
    }
}

/* ============================================================================
 * Exported function definitions
 * ============================================================================ */

void MyRunner_Init(void)
{
    MyQueue_Init();
    s_state       = MY_RUNNER_STATE_IDLE;
    s_pos1_cdeg   = 0;
    s_pos2_cdeg   = 0;
    s_pos3_cdeg   = 0;
    s_trap_active = false;
}

void MyRunner_FeedBytes(const uint8_t *data, uint16_t length)
{
    (void)data;
    (void)length;
}

void MyRunner_Start(void)
{
    if ((s_state == MY_RUNNER_STATE_RUNNING) || MyQueue_IsEmpty())
    {
        return;
    }

    s_pos1_cdeg   = 0;
    s_pos2_cdeg   = 0;
    s_pos3_cdeg   = 0;
    s_trap_active = false;
    s_state       = MY_RUNNER_STATE_RUNNING;
}

void MyRunner_Stop(void)
{
    if (s_trap_active)
    {
        MyController_AbortTrap();
        s_trap_active = false;
    }
    else
    {
        prv_stop_all_motors();
    }

    MyQueue_Flush();
    s_state = MY_RUNNER_STATE_IDLE;
    Command_UsbSendText("traj_stop\r\n");
}

MyRunner_State_t MyRunner_GetState(void)
{
    return s_state;
}

bool MyRunner_IsActive(void)
{
    return (s_state == MY_RUNNER_STATE_RUNNING);
}

/**
 * @brief  Chạy tác vụ thực thi quỹ đạo, gọi lặp trong vòng main.
 *
 * Logic:
 *   1. Nếu đang có trap move → tick profile, đợi hoàn tất.
 *   2. Khi trap xong → cập nhật vị trí phần mềm, gửi ack.
 *   3. Pop frame kế tiếp và bắt đầu trap mới.
 *   4. Queue cạn → gửi "traj_done", về IDLE.
 */
void MyRunner_Process(void)
{
    RobotCommand                         frame;
    MyController_ThreeMotorMoveCommand_t cmd;

    if (s_state != MY_RUNNER_STATE_RUNNING)
    {
        return;
    }

    /* Chờ trap move hiện tại hoàn tất. */
    if (s_trap_active)
    {
        MyController_TrapProcess();

        if (!MyController_IsTrapMoveDone())
        {
            return;
        }

        /* Trap done — cập nhật vị trí phần mềm và gửi báo cáo. */
        (void)MyController_FinishTrapMove(&s_trap_ctx);
        s_pos1_cdeg   = s_trap_ctx.motor1_angle_cdeg;
        s_pos2_cdeg   = s_trap_ctx.motor2_angle_cdeg;
        s_pos3_cdeg   = s_trap_ctx.motor3_angle_cdeg;
        s_trap_active = false;
        prv_send_report();
    }

    /* Queue cạn → hoàn tất quỹ đạo. */
    if (MyQueue_IsEmpty())
    {
        s_state = MY_RUNNER_STATE_IDLE;
        Command_UsbSendText("traj_done\r\n");
        return;
    }

    /* Pop và bắt đầu frame kế tiếp bằng trap profile. */
    (void)MyQueue_Pop(&frame);

    cmd.motor1_angle_cdeg = frame.angle1_x100;
    cmd.motor2_angle_cdeg = frame.angle2_x100;
    cmd.motor3_angle_cdeg = frame.angle3_x100;

    if (MyController_StartTrapMove(&cmd, &s_trap_ctx) == MY_CONTROLLER_OK)
    {
        s_trap_active = true;
    }
    else
    {
        Command_UsbSendText("ERR: trap start failed\r\n");
    }
}
