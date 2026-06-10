/**
 * @file    trajectory.h
 * @brief   Quản lý quỹ đạo vận tốc hình thang cho động cơ bước.
 * @author  Lap4all
 * @date    2026-05-17
 */

#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

#include <stdint.h>
#include <stdbool.h>

#define TRAJ_DT_MIN_S   0.001f  /**< Bước thời gian cập nhật tối thiểu (s). */
#define TRAJ_DT_MAX_S   0.050f  /**< Bước thời gian cập nhật tối đa (s). */

/**
 * @brief Pha hiện tại của hành trình vận tốc hình thang.
 */
typedef enum
{
    TRAJ_STATE_IDLE   = 0, /**< Chưa khởi động hoặc đứng yên. */
    TRAJ_STATE_ACCEL  = 1, /**< Pha tăng tốc từ v=0 đến v_max. */
    TRAJ_STATE_CRUISE = 2, /**< Pha chạy ổn định tốc độ v_max. */
    TRAJ_STATE_DECEL  = 3, /**< Pha giảm tốc từ v_max về v=0. */
    TRAJ_STATE_DONE   = 4, /**< Đã đến vị trí mục tiêu, hành trình kết thúc. */
} Traj_State_t;

/**
 * @brief Handle chứa toàn bộ trạng thái của một hành trình vận tốc hình thang.
 *
 * Dùng @ref Traj_Init để khởi tạo trước khi gọi @ref Traj_Update mỗi chu kỳ.
 */
typedef struct
{
    float        target_pos;  /**< Vị trí mục tiêu (độ). */
    float        current_pos; /**< Vị trí hiện tại được tính toán (độ). */
    float        v_max_dps;   /**< Vận tốc cực đại (độ/giây). Có thể bị hạ nếu quãng đường quá ngắn. */
    float        acc_dps2;    /**< Gia tốc và giảm tốc (độ/giây²). */
    float        v_cur_dps;   /**< Vận tốc tức thời trong pha đang chạy (độ/giây). */
    float        remaining;   /**< Quãng đường còn lại đến đích (độ). */
    float        total_dist;  /**< Tổng quãng đường của hành trình (độ). */
    float        acc_dist;    /**< Quãng đường thuộc pha tăng tốc (độ). */
    float        dec_dist;    /**< Quãng đường thuộc pha giảm tốc (độ). */
    float        cruise_dist; /**< Quãng đường thuộc pha ổn định (độ). */
    float        direction;   /**< Chiều chuyển động: +1.0f (thuận) hoặc -1.0f (ngược). */
    Traj_State_t state;       /**< Pha hiện tại của hành trình. @see Traj_State_t */
} Traj_Handle_t;

/**
 * @brief  Khởi tạo hành trình vận tốc hình thang từ vị trí hiện tại đến vị trí mục tiêu.
 * @param[in,out] traj        Con trỏ đến handle hành trình.
 * @param[in]     target_deg  Vị trí đích (độ).
 * @param[in]     current_deg Vị trí xuất phát (độ).
 * @param[in]     v_max_dps   Vận tốc tối đa mong muốn (độ/giây), giá trị dương.
 * @param[in]     acc_dps2    Gia tốc (độ/giây²), giá trị dương.
 * @note  Nếu quãng đường quá ngắn để đạt v_max, hàm tự hạ v_max về đỉnh thực tế
 *        và bỏ qua pha CRUISE (profile trở thành hình tam giác).
 */
void Traj_Init(Traj_Handle_t *traj,
               float target_deg, float current_deg,
               float v_max_dps, float acc_dps2);

/**
 * @brief  Cập nhật vị trí theo profile hình thang, gọi mỗi chu kỳ điều khiển.
 * @param[in,out] traj  Con trỏ đến handle hành trình.
 * @param[in]     dt_s  Bước thời gian kể từ lần gọi trước (giây).
 *                      Được kẹp vào [TRAJ_DT_MIN_S, TRAJ_DT_MAX_S].
 * @return Vị trí hiện tại được tính toán (độ).
 */
float Traj_Update(Traj_Handle_t *traj, float dt_s);

/**
 * @brief  Kiểm tra hành trình đã hoàn thành chưa.
 * @param[in] traj  Con trỏ đến handle hành trình.
 * @return true nếu đã đến đích (TRAJ_STATE_DONE), false nếu còn đang chạy.
 */
bool Traj_IsDone(const Traj_Handle_t *traj);

#endif /* INC_TRAJECTORY_H_ */
