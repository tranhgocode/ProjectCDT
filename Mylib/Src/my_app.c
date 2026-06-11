/**
 * @file    my_app.c
 * @brief   Tầng ứng dụng: binary frame 20B + lệnh text "a b c" qua USB CDC.
 * @author  Lap4all
 * @date    2026-06-10
 */

#include "my_app.h"
#include "my_controller.h"
#include "command.h"
#include "traj_runner.h"
#include "my_queue.h"
#include "protocol.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* ============================================================================
 * Private typedef
 * ============================================================================ */

typedef enum {
    MY_APP_STATE_IDLE = 0,        /**< Chờ nhận frame hoặc lệnh. */
    MY_APP_STATE_TRAJ_RUNNING,    /**< Đang thực thi quỹ đạo binary frame. */
    MY_APP_STATE_TRAP_RUNNING,    /**< Đang chạy profile hình thang 3 motor. */
} my_app_state_t;

/* ============================================================================
 * Private variables
 * ============================================================================ */

static my_app_state_t                   s_app_state        = MY_APP_STATE_IDLE;
static bool                             s_controller_ready = false;
static bool                             s_error_reported   = false;
static MyController_ThreeMotorMoveContext_t s_trap_context;

/* Bộ đệm dùng chung cho các báo cáo kết quả. */
static char     s_tx_buf[COMMAND_USB_TX_BUFFER_SIZE];
static uint16_t s_tx_len;

/* ============================================================================
 * Private helpers — string builder wrappers
 * ============================================================================ */

static void s_text(const char *t)
{
    Command_AppendText(s_tx_buf, sizeof(s_tx_buf), &s_tx_len, t);
}
static void s_angle(int32_t cdeg)
{
    Command_AppendAngleDeg(cdeg, s_tx_buf, sizeof(s_tx_buf), &s_tx_len);
}
static void s_uint(uint32_t v)
{
    Command_AppendUnsigned(s_tx_buf, sizeof(s_tx_buf), &s_tx_len, v);
}
static void s_int(int32_t v)
{
    Command_AppendSigned(s_tx_buf, sizeof(s_tx_buf), &s_tx_len, v);
}
static void s_flush(void)
{
    if ((s_tx_len > 0U) && (s_tx_len < (uint16_t)sizeof(s_tx_buf)))
    {
        Command_UsbSendBuffer((uint8_t *)s_tx_buf, s_tx_len);
    }
}
static void s_reset(void)
{
    s_tx_buf[0] = '\0';
    s_tx_len    = 0U;
}

/* ============================================================================
 * Private function prototypes
 * ============================================================================ */

static void my_app_process_idle(void);
static void my_app_process_traj(void);
static void my_app_process_trap(void);

/* ============================================================================
 * Private function definitions
 * ============================================================================ */

/**
 * @brief  Đẩy một dòng "$M,seq,a1,a2,a3" vào queue và trả lời ACK/ERR.
 * @note   Đây là mắt xích nhóm 4/5: decode -> push -> ACK. Motion task
 *         (MotionTask_Run) sẽ pop và chạy ở bước sau.
 */
static void my_app_handle_move_line(const char *line)
{
    RobotCommand   cmd;
    ProtocolStatus st = Protocol_DecodeMoveLine(line, &cmd);

    if (st == PROTO_OK)
    {
        if (MyQueue_Push(&cmd)) { Response_SendACK(cmd.seq); }
        else                    { Response_SendERR(cmd.seq, "QUEUE_FULL"); }
    }
    else if (st == PROTO_ERR_RANGE)
    {
        Response_SendERR(cmd.seq, "RANGE");
    }
    else /* PROTO_ERR_FORMAT, PROTO_ERR_UNKNOWN_TYPE */
    {
        Response_SendERR(cmd.seq, "FORMAT");
    }
}

/**
 * @brief  Xử lý lệnh khi IDLE — rút cạn các dòng đang chờ trong ring buffer.
 *
 *   "$M,…"   → decode → push queue → $A/$E  (protocol nhóm 4)
 *   "STOP"   → dừng + flush queue            (legacy bring-up)
 *   "GO"     → go_ok + chạy trajectory       (legacy bring-up)
 *   "ZERO"   → hiệu chỉnh sensor             (legacy bring-up)
 *   "a b c"  → trap profile 3 motor          (legacy bring-up)
 *   khác     → $E,0,FORMAT
 *
 * Rút cạn nhiều dòng/lần để một burst "$M,6…\n$M,7…\n" được ACK liên tiếp
 * (A6, A7) trước khi motion chạy, đúng thứ tự contract §17.7.
 */
static void my_app_process_idle(void)
{
    char     line[COMMAND_USB_RX_BUFFER_SIZE];
    uint16_t len;
    MyController_ThreeMotorMoveCommand_t three_cmd = {0};

    while (USB_RX_GetLine(line, sizeof(line)) != 0)
    {
        len = (uint16_t)strlen(line);
        if (len == 0U) { continue; }

        /* STOP (legacy) */
        if (Command_IsStopCommand((const uint8_t *)line, len))
        {
            MyRunner_Stop();
            continue;
        }

        /* GO (legacy) — đổi state, thoát IDLE */
        if (Command_IsGoCommand((const uint8_t *)line, len))
        {
            if (MyQueue_IsEmpty())
            {
                Command_UsbSendText("ERR: queue empty\r\n");
                continue;
            }
            Command_UsbSendText("go_ok\r\n");
            MyRunner_Start();
            s_app_state = MY_APP_STATE_TRAJ_RUNNING;
            return;
        }

        /* ZERO (legacy) */
        if (Command_IsZeroCommand((const uint8_t *)line, len))
        {
            if (MyController_SetZeroFromSensor() == MY_CONTROLLER_OK)
                Command_UsbSendText("zero_ok\r\n");
            else
                Command_UsbSendText("ERR: sensor read failed\r\n");
            continue;
        }

        /* "a b c" — trap profile thủ công (không bắt đầu bằng '$') */
        if ((line[0] != '$') &&
            Command_ParseThreeMotorCommand((const uint8_t *)line, len, &three_cmd))
        {
            if (MyController_StartTrapMove(&three_cmd, &s_trap_context) != MY_CONTROLLER_OK)
            {
                Command_UsbSendText("ERR: trap start failed\r\n");
                continue;
            }
            s_app_state = MY_APP_STATE_TRAP_RUNNING;
            return;
        }

        /* Còn lại: protocol $M hoặc dòng lỗi → $A/$E */
        my_app_handle_move_line(line);
    }
}

/** @brief Trajectory execution state. */
static void my_app_process_traj(void)
{
    char     line[COMMAND_USB_RX_BUFFER_SIZE];
    uint16_t len;

    if (USB_RX_GetLine(line, sizeof(line)) != 0)
    {
        len = (uint16_t)strlen(line);
        if (Command_IsStopCommand((const uint8_t *)line, len))
        {
            MyRunner_Stop();
            s_app_state = MY_APP_STATE_IDLE;
            return;
        }
    }

    MyRunner_Process();

    if (!MyRunner_IsActive())
    {
        s_app_state = MY_APP_STATE_IDLE;
    }
}

/** @brief Trap-profile 3-motor execution state. */
static void my_app_process_trap(void)
{
    MyController_Status_t       status;
    MyController_ThreeSensorReadout_t sr;
    char     line[COMMAND_USB_RX_BUFFER_SIZE];
    uint16_t len;

    /* Cho phép STOP trong khi chạy. */
    if (USB_RX_GetLine(line, sizeof(line)) != 0)
    {
        len = (uint16_t)strlen(line);
        if (Command_IsStopCommand((const uint8_t *)line, len))
        {
            MyController_AbortTrap();
            Command_UsbSendText("trap_aborted\r\n");
            s_app_state = MY_APP_STATE_IDLE;
            return;
        }
        Command_UsbSendText("ERR: busy, trap running\r\n");
    }

    MyController_TrapProcess();

    if (!MyController_IsTrapMoveDone()) { return; }

    /* Hoàn tất — đọc cảm biến và gửi báo cáo. */
    status = MyController_FinishTrapMove(&s_trap_context);
    if (status != MY_CONTROLLER_OK)
    {
        Command_UsbSendText("ERR: trap finish failed\r\n");
        s_app_state = MY_APP_STATE_IDLE;
        return;
    }

    status = MyController_ReadThreeSensors(&sr);
    if (status != MY_CONTROLLER_OK)
    {
        Command_UsbSendText("ERR: sensor read failed\r\n");
        s_app_state = MY_APP_STATE_IDLE;
        return;
    }

    s_reset();
    s_text("three_ok");
    s_text(",m1_t="); s_angle(s_trap_context.motor1_angle_cdeg);
    s_text(",m1_d="); s_angle(s_trap_context.motor1_delta_cdeg);
    s_text(",m1_s="); s_uint(s_trap_context.motor1_target_steps);
    s_text(",m2_t="); s_angle(s_trap_context.motor2_angle_cdeg);
    s_text(",m2_d="); s_angle(s_trap_context.motor2_delta_cdeg);
    s_text(",m2_s="); s_uint(s_trap_context.motor2_target_steps);
    s_text(",m3_t="); s_angle(s_trap_context.motor3_angle_cdeg);
    s_text(",m3_d="); s_angle(s_trap_context.motor3_delta_cdeg);
    s_text(",m3_s="); s_uint(s_trap_context.motor3_target_steps);
    s_text(",as1="); s_angle(sr.sensor1_angle_cdeg);
    s_text(",as1_st="); s_int((int32_t)sr.sensor1_status);
    s_text(",as2="); s_angle(sr.sensor2_angle_cdeg);
    s_text(",as2_st="); s_int((int32_t)sr.sensor2_status);
    s_text(",as3="); s_angle(sr.sensor3_angle_cdeg);
    s_text(",as3_st="); s_int((int32_t)sr.sensor3_status);
    s_text("\r\n");
    s_flush();

    (void)MyController_ResetThreeSensorZero();
    s_app_state = MY_APP_STATE_IDLE;
}

/* ============================================================================
 * Exported functions
 * ============================================================================ */

void my_app_init(void)
{
    MyController_Status_t status;

    s_app_state        = MY_APP_STATE_IDLE;
    s_controller_ready = false;
    s_error_reported   = false;

    MyRunner_Init();

    status = MyController_Init();
    if (status == MY_CONTROLLER_ERR_SENSOR)
    {
        s_controller_ready = true; /* Motor OK, sensor lỗi — vẫn chạy traj. */
        return;
    }
    if (status != MY_CONTROLLER_OK) { return; }

    s_controller_ready = true;
    if (MyController_SetZeroFromSensor() != MY_CONTROLLER_OK)
    {
        Command_UsbSendText("WARN: sensor zero failed at init\r\n");
    }
}

void my_app_process(void)
{
    if (!Command_UsbIsReady()) { return; }

    if (!s_controller_ready)
    {
        if (!s_error_reported)
        {
            Command_UsbSendText("ERR: controller init failed\r\n");
            s_error_reported = true;
        }
        return;
    }

    switch (s_app_state)
    {
    case MY_APP_STATE_TRAJ_RUNNING: my_app_process_traj(); break;
    case MY_APP_STATE_TRAP_RUNNING: my_app_process_trap(); break;
    default:
        my_app_process_idle();
        /* Còn IDLE (không có GO/trap) → motion task pop & chạy 1 lệnh. */
        if (s_app_state == MY_APP_STATE_IDLE)
        {
            MotionTask_Run();
        }
        break;
    }
}
