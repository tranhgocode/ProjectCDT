/**
 * @file    trajectory.c
 * @brief   Triển khai quỹ đạo vận tốc hình thang cho động cơ bước.
 * @author  Lap4all
 * @date    2026-05-17
 */

#include "trajectory.h"

#include <math.h>

/* -------------------------------------------------------------------------
 * Private function prototypes
 * ---------------------------------------------------------------------- */

static void prv_ComputeDistances(Traj_Handle_t *traj);

/* -------------------------------------------------------------------------
 * Exported functions
 * ---------------------------------------------------------------------- */

void Traj_Init(Traj_Handle_t *traj,
               float target_deg, float current_deg,
               float v_max_dps, float acc_dps2)
{
    traj->target_pos  = target_deg;
    traj->current_pos = current_deg;
    traj->v_max_dps   = fabsf(v_max_dps);
    traj->acc_dps2    = fabsf(acc_dps2);
    traj->v_cur_dps   = 0.0f;
    traj->total_dist  = fabsf(target_deg - current_deg);
    traj->remaining   = traj->total_dist;
    traj->direction   = (target_deg >= current_deg) ? 1.0f : -1.0f;

    // Khoảng cách quá nhỏ, không cần di chuyển
    if (traj->total_dist < 0.0001f)
    {
        traj->state = TRAJ_STATE_DONE;
        return;
    }

    prv_ComputeDistances(traj);
    traj->state = TRAJ_STATE_ACCEL;
}

float Traj_Update(Traj_Handle_t *traj, float dt_s)
{
    if (dt_s < TRAJ_DT_MIN_S) { dt_s = TRAJ_DT_MIN_S; }
    if (dt_s > TRAJ_DT_MAX_S) { dt_s = TRAJ_DT_MAX_S; }

    if (traj->state == TRAJ_STATE_DONE)
    {
        return traj->target_pos;
    }

    float dv = traj->acc_dps2 * dt_s;

    switch (traj->state)
    {
    case TRAJ_STATE_ACCEL:
        traj->v_cur_dps += dv;
        if (traj->v_cur_dps >= traj->v_max_dps)
        {
            traj->v_cur_dps = traj->v_max_dps;
            traj->state = (traj->cruise_dist > 0.0f) ? TRAJ_STATE_CRUISE
                                                      : TRAJ_STATE_DECEL;
        }
        break;

    case TRAJ_STATE_CRUISE:
        /* Giữ v_max, chuyển sang DECEL khi remaining đủ nhỏ (xử lý bên dưới). */
        break;

    case TRAJ_STATE_DECEL:
        traj->v_cur_dps -= dv;
        if (traj->v_cur_dps <= 0.0f)
        {
            traj->v_cur_dps = 0.0f;
            traj->state     = TRAJ_STATE_DONE;
        }
        break;

    default:
        break;
    }

    float ds = traj->v_cur_dps * dt_s;

    // Chuyển sang DECEL khi quãng đường còn lại vừa chạm ngưỡng dec_dist
    if (traj->state == TRAJ_STATE_ACCEL || traj->state == TRAJ_STATE_CRUISE)
    {
        if (traj->remaining - ds <= traj->dec_dist)
        {
            traj->state = TRAJ_STATE_DECEL;
        }
    }

    if (ds > traj->remaining)
    {
        ds = traj->remaining;
    }

    traj->remaining   -= ds;
    traj->current_pos += traj->direction * ds;

    if (traj->remaining < 0.0001f)
    {
        traj->current_pos = traj->target_pos;
        traj->state       = TRAJ_STATE_DONE;
    }

    return traj->current_pos;
}

bool Traj_IsDone(const Traj_Handle_t *traj)
{
    return (traj->state == TRAJ_STATE_DONE);
}

/* -------------------------------------------------------------------------
 * Private functions
 * ---------------------------------------------------------------------- */

/*
 * Tính acc_dist, dec_dist và cruise_dist theo v_max_dps và acc_dps2 hiện tại.
 * Nếu quãng đường không đủ để đạt v_max, hạ v_max về đỉnh thực tế để profile
 * trở thành hình tam giác (accel rồi decel, không có cruise).
 */
static void prv_ComputeDistances(Traj_Handle_t *traj)
{
    float d_acc = (traj->v_max_dps * traj->v_max_dps) / (2.0f * traj->acc_dps2);

    if (traj->total_dist >= 2.0f * d_acc)
    {
        traj->acc_dist = d_acc;
        traj->dec_dist = d_acc;
    }
    else
    {
        /* Quãng đường ngắn: tính lại v_max theo đỉnh hình tam giác */
        float v_peak    = sqrtf(traj->total_dist * traj->acc_dps2);
        traj->v_max_dps = v_peak;
        traj->acc_dist  = (v_peak * v_peak) / (2.0f * traj->acc_dps2);
        traj->dec_dist  = traj->acc_dist;
    }

    traj->cruise_dist = traj->total_dist - traj->acc_dist - traj->dec_dist;
    if (traj->cruise_dist < 0.0f)
    {
        traj->cruise_dist = 0.0f;
    }
}
