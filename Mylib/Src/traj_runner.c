/**
 * @file    my_runner.c
 * @brief   Thực thi quỹ đạo 50 Hz: lấy frame từ queue, điều khiển motor, báo cáo.
 * @author  Lap4all
 * @date    2026-06-08
 */

#include "traj_runner.h"
#include "my_queue.h"
#include "my_controller.h"
#include "my_app.h"
#include "my_config.h"
#include "command.h"
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * Private define
 * ============================================================================ */

/** @brief Chiều thuận Motor1 dùng trong chế độ quỹ đạo. */
#define MY_RUNNER_MOTOR1_FORWARD_DIR   ((TMC2209_DirectionTypeDef)MY_APP_MOTOR1_FORWARD_DIR)

/** @brief Chiều thuận Motor2 dùng trong chế độ quỹ đạo. */
#define MY_RUNNER_MOTOR2_FORWARD_DIR   ((TMC2209_DirectionTypeDef)MY_APP_MOTOR2_FORWARD_DIR)

/** @brief Chiều thuận Motor3 dùng trong chế độ quỹ đạo. */
#define MY_RUNNER_MOTOR3_FORWARD_DIR   ((TMC2209_DirectionTypeDef)MY_APP_MOTOR3_FORWARD_DIR)

/* ============================================================================
 * Private variables
 * ============================================================================ */

/** @brief Trạng thái hiện tại của bộ thực thi. */
static MyRunner_State_t s_state;

/** @brief Vị trí phần mềm motor1 trong chế độ quỹ đạo, tính bằng centi-độ. */
static int32_t s_pos1_cdeg;

/** @brief Vị trí phần mềm motor2 trong chế độ quỹ đạo, tính bằng centi-độ. */
static int32_t s_pos2_cdeg;

/** @brief Vị trí phần mềm motor3 trong chế độ quỹ đạo, tính bằng centi-độ. */
static int32_t s_pos3_cdeg;

/** @brief Giá trị HAL_GetTick() của lần tick gần nhất. */
static uint32_t s_last_tick_ms;

/** @brief Chỉ số timestep của frame vừa thực thi gần nhất. */
static uint32_t s_current_timestep;

/* ============================================================================
 * Private function prototypes
 * ============================================================================ */

static uint32_t prv_vel_to_hz(float vel_dps, uint32_t steps_per_turn);
static uint32_t prv_calc_steps(const TMC2209_HandleTypeDef *hmotor,
                               int32_t delta_cdeg);
static uint32_t prv_scale_steps(uint32_t steps, uint32_t num, uint32_t den);
static TMC2209_DirectionTypeDef prv_motor_dir(int32_t delta_cdeg,
                                              TMC2209_DirectionTypeDef forward_dir);
static void prv_stop_all_motors(void);
static void prv_dispatch_frame(const MyQueue_Frame_t *frame);
static void prv_send_report(void);

/* ============================================================================
 * Private function definitions
 * ============================================================================ */

/**
 * @brief  Chuyển vận tốc deg/s sang tần số xung STEP (Hz).
 * @param  vel_dps:       Vận tốc, tính bằng độ/giây (lấy giá trị tuyệt đối).
 * @param  steps_per_turn: steps_per_rev × microstep của motor.
 * @return Tần số Hz đã giới hạn trong [MY_RUNNER_MIN_SPEED_HZ, MAX].
 */
static uint32_t prv_vel_to_hz(float vel_dps, uint32_t steps_per_turn)
{
    float hz;

    if (vel_dps < 0.0f)
    {
        vel_dps = -vel_dps;
    }

    hz = (vel_dps * (float)steps_per_turn) / 360.0f;

    if (hz < (float)MY_RUNNER_MIN_SPEED_HZ)
    {
        hz = (float)MY_RUNNER_MIN_SPEED_HZ;
    }
    if (hz > (float)MY_RUNNER_MAX_SPEED_HZ)
    {
        hz = (float)MY_RUNNER_MAX_SPEED_HZ;
    }

    return (uint32_t)hz;
}

/**
 * @brief  Chuyển góc tương đối sang số microstep theo cấu hình motor.
 * @param  hmotor:    Handle motor chứa steps_per_rev và microstep.
 * @param  delta_cdeg: Góc tương đối, tính bằng centi-độ.
 * @return Số microstep cần phát (chưa scale tỉ số truyền).
 */
static uint32_t prv_calc_steps(const TMC2209_HandleTypeDef *hmotor,
                               int32_t delta_cdeg)
{
    uint32_t abs_cdeg;
    uint32_t steps_per_turn;

    if (hmotor == NULL)
    {
        return 0U;
    }

    abs_cdeg = (delta_cdeg < 0) ?
        (uint32_t)(-delta_cdeg) :
        (uint32_t)delta_cdeg;

    steps_per_turn = (uint32_t)hmotor->steps_per_rev *
                     (uint32_t)hmotor->microstep;

    /* Làm tròn gần nhất. */
    return (uint32_t)(((uint64_t)abs_cdeg * steps_per_turn +
                       ((uint64_t)MY_CONTROLLER_FULL_TURN_CDEG / 2U)) /
                      (uint64_t)MY_CONTROLLER_FULL_TURN_CDEG);
}

/**
 * @brief  Nhân số bước với hệ số tỉ số truyền cơ khí.
 * @param  steps:       Số microstep gốc tính từ góc.
 * @param  num:         Tử số hệ số scale.
 * @param  den:         Mẫu số hệ số scale.
 * @return Số microstep sau khi scale, làm tròn gần nhất.
 */
static uint32_t prv_scale_steps(uint32_t steps, uint32_t num, uint32_t den)
{
    if ((steps == 0U) || (den == 0U))
    {
        return 0U;
    }

    return (uint32_t)((((uint64_t)steps * (uint64_t)num) +
                       ((uint64_t)den / 2U)) /
                      (uint64_t)den);
}

/**
 * @brief  Chọn chiều chạy motor từ dấu của delta và chiều thuận.
 * @param  delta_cdeg:  Góc cần chạy, centi-độ.
 * @param  forward_dir: Chiều motor quay khi delta không âm.
 * @return Chiều quay thực tế.
 */
static TMC2209_DirectionTypeDef prv_motor_dir(int32_t delta_cdeg,
                                              TMC2209_DirectionTypeDef forward_dir)
{
    if (delta_cdeg >= 0)
    {
        return forward_dir;
    }

    return (forward_dir == TMC2209_DIR_CW) ? TMC2209_DIR_CCW : TMC2209_DIR_CW;
}

/** @brief Dừng cả ba motor step ngay lập tức. */
static void prv_stop_all_motors(void)
{
    TMC2209_Stop(&motor1);
    TMC2209_Stop(&motor2);
    TMC2209_Stop(&motor3);
}

/**
 * @brief  Thực thi một frame: tính delta, tốc độ và phát lệnh cho 3 motor step.
 * @param  frame: Frame quỹ đạo vừa pop khỏi queue.
 * @note   theta4 được lưu vào s_pos4 nhưng chưa điều khiển servo.
 */
static void prv_dispatch_frame(const MyQueue_Frame_t *frame)
{
    int32_t  delta1_cdeg;
    int32_t  delta2_cdeg;
    int32_t  delta3_cdeg;
    uint32_t steps1;
    uint32_t steps2;
    uint32_t steps3;
    uint32_t hz1;
    uint32_t hz2;
    uint32_t hz3;
    int32_t  target1_cdeg;
    int32_t  target2_cdeg;
    int32_t  target3_cdeg;

    target1_cdeg = (int32_t)(frame->theta1_deg * 100.0f);
    target2_cdeg = (int32_t)(frame->theta2_deg * 100.0f);
    target3_cdeg = (int32_t)(frame->theta3_deg * 100.0f);

    delta1_cdeg = target1_cdeg - s_pos1_cdeg;
    delta2_cdeg = target2_cdeg - s_pos2_cdeg;
    delta3_cdeg = target3_cdeg - s_pos3_cdeg;

    steps1 = prv_scale_steps(prv_calc_steps(&motor1, delta1_cdeg),
                             MY_APP_MOTOR1_STEP_SCALE_NUM,
                             MY_APP_MOTOR1_STEP_SCALE_DEN);
    steps2 = prv_scale_steps(prv_calc_steps(&motor2, delta2_cdeg),
                             MY_APP_MOTOR2_STEP_SCALE_NUM,
                             MY_APP_MOTOR2_STEP_SCALE_DEN);
    steps3 = prv_scale_steps(prv_calc_steps(&motor3, delta3_cdeg),
                             MY_APP_MOTOR3_STEP_SCALE_NUM,
                             MY_APP_MOTOR3_STEP_SCALE_DEN);

    hz1 = prv_vel_to_hz(frame->vel1_dps,
                        (uint32_t)motor1.steps_per_rev *
                        (uint32_t)motor1.microstep);
    hz2 = prv_vel_to_hz(frame->vel2_dps,
                        (uint32_t)motor2.steps_per_rev *
                        (uint32_t)motor2.microstep);
    hz3 = prv_vel_to_hz(frame->vel3_dps,
                        (uint32_t)motor3.steps_per_rev *
                        (uint32_t)motor3.microstep);

    /*
     * Dừng chuyển động cũ trước khi phát lệnh mới để tránh tích lũy bước thừa
     * khi frame mới đến trước khi frame cũ hoàn tất.
     */
    prv_stop_all_motors();

    if (steps1 > 0U)
    {
        (void)TMC2209_MoveSteps(&motor1,
                                steps1,
                                prv_motor_dir(delta1_cdeg, MY_RUNNER_MOTOR1_FORWARD_DIR),
                                hz1);
    }

    if (steps2 > 0U)
    {
        (void)TMC2209_MoveSteps(&motor2,
                                steps2,
                                prv_motor_dir(delta2_cdeg, MY_RUNNER_MOTOR2_FORWARD_DIR),
                                hz2);
    }

    if (steps3 > 0U)
    {
        (void)TMC2209_MoveSteps(&motor3,
                                steps3,
                                prv_motor_dir(delta3_cdeg, MY_RUNNER_MOTOR3_FORWARD_DIR),
                                hz3);
    }

    /* Cập nhật vị trí phần mềm theo góc đã ra lệnh. */
    s_pos1_cdeg = target1_cdeg;
    s_pos2_cdeg = target2_cdeg;
    s_pos3_cdeg = target3_cdeg;
    s_current_timestep = frame->timestep;

    /* theta4 nhận nhưng chưa điều khiển — servo sẽ bổ sung sau. */
    (void)frame->theta4_deg;
}

/**
 * @brief  Gửi báo cáo trạng thái quỹ đạo qua USB CDC.
 * @note   Định dạng: traj_t=N,q=N,m1=D.DD,m2=D.DD,m3=D.DD\r\n
 */
static void prv_send_report(void)
{
    char     buf[COMMAND_USB_TX_BUFFER_SIZE];
    uint16_t idx = 0U;

    if (Command_UsbIsReady() == false)
    {
        return;
    }

    buf[0] = '\0';
    Command_AppendText(buf, sizeof(buf), &idx, "traj_t=");
    Command_AppendUnsigned(buf, sizeof(buf), &idx, s_current_timestep);
    Command_AppendText(buf, sizeof(buf), &idx, ",q=");
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

/**
 * @brief  Khởi tạo bộ thực thi, hàng đợi quỹ đạo và đặt lại vị trí motor.
 */
void MyRunner_Init(void)
{
    MyQueue_Init();
    s_state            = MY_RUNNER_STATE_IDLE;
    s_pos1_cdeg        = 0;
    s_pos2_cdeg        = 0;
    s_pos3_cdeg        = 0;
    s_last_tick_ms     = 0U;
    s_current_timestep = 0U;
}

/**
 * @brief  Nạp byte thô USB vào bộ parser frame quỹ đạo.
 * @param  data:   Con trỏ tới mảng byte nhận từ USB CDC.
 * @param  length: Số byte cần xử lý.
 */
void MyRunner_FeedBytes(const uint8_t *data, uint16_t length)
{
    MyQueue_FeedBytes(data, length);
}

/**
 * @brief  Bắt đầu thực thi quỹ đạo từ frame đầu tiên trong queue.
 */
void MyRunner_Start(void)
{
    if ((s_state == MY_RUNNER_STATE_RUNNING) || MyQueue_IsEmpty())
    {
        return;
    }

    s_pos1_cdeg        = 0;
    s_pos2_cdeg        = 0;
    s_pos3_cdeg        = 0;
    s_current_timestep = 0U;
    s_last_tick_ms     = HAL_GetTick();
    s_state            = MY_RUNNER_STATE_RUNNING;
}

/**
 * @brief  Dừng thực thi ngay lập tức, xóa queue và chuyển về IDLE.
 */
void MyRunner_Stop(void)
{
    prv_stop_all_motors();
    MyQueue_Flush();
    s_state = MY_RUNNER_STATE_IDLE;
    Command_UsbSendText("traj_stop\r\n");
}

/**
 * @brief  Trả về trạng thái hiện tại của bộ thực thi.
 * @return MY_RUNNER_STATE_IDLE hoặc MY_RUNNER_STATE_RUNNING.
 */
MyRunner_State_t MyRunner_GetState(void)
{
    return s_state;
}

/**
 * @brief  Kiểm tra bộ thực thi có đang chạy quỹ đạo không.
 * @return true nếu đang ở trạng thái RUNNING.
 */
bool MyRunner_IsActive(void)
{
    return (s_state == MY_RUNNER_STATE_RUNNING);
}

/**
 * @brief  Chạy tác vụ thực thi quỹ đạo không blocking, gọi lặp trong vòng main.
 */
void MyRunner_Process(void)
{
    MyQueue_Frame_t frame;
    uint32_t        now_ms;

    if (s_state != MY_RUNNER_STATE_RUNNING)
    {
        return;
    }

    now_ms = HAL_GetTick();

    if ((now_ms - s_last_tick_ms) < MY_RUNNER_TICK_MS)
    {
        return;
    }

    s_last_tick_ms = now_ms;

    if (MyQueue_IsEmpty())
    {
        /* Queue cạn: dừng motor và báo hoàn tất. */
        prv_stop_all_motors();
        s_state = MY_RUNNER_STATE_IDLE;
        Command_UsbSendText("traj_done\r\n");
        return;
    }

    (void)MyQueue_Pop(&frame);
    prv_dispatch_frame(&frame);
    prv_send_report();
}
