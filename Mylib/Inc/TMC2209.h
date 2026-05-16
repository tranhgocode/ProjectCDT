/*
 * TMC2209.h
 *
 *  Created on: May 14, 2026
 *      Author: Lap4all
 */

#ifndef MYLIB_INC_TMC2209_H_
#define MYLIB_INC_TMC2209_H_

#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "stm32f1xx_hal.h"


/* ============================================================================
 * CHUYEN DOI: doi sang dung UART cua ban
 *   extern UART_HandleTypeDef huart3;
 *   #define TMC2209_UART   (&huart3)
 * ============================================================================ */
extern UART_HandleTypeDef   huart3;
#define TMC2209_UART        (&huart3)
 
/* Timeout UART (ms) */
#define TMC2209_UART_TIMEOUT    10U
 
/* So driver toi da */
#define TMC2209_MAX_DRIVERS     3U
 
/* ============================================================================
 * TMC2209 REGISTER MAP
 * ============================================================================ */
#define TMC2209_REG_GCONF       0x00U
#define TMC2209_REG_GSTAT       0x01U
#define TMC2209_REG_IFCNT       0x02U
#define TMC2209_REG_SLAVECONF   0x03U
#define TMC2209_REG_IOIN        0x06U
#define TMC2209_REG_IHOLD_IRUN  0x10U
#define TMC2209_REG_TPOWERDOWN  0x11U
#define TMC2209_REG_TSTEP       0x12U
#define TMC2209_REG_TPWMTHRS    0x13U
#define TMC2209_REG_TCOOLTHRS   0x14U
#define TMC2209_REG_VACTUAL     0x22U
#define TMC2209_REG_SGTHRS      0x40U
#define TMC2209_REG_SG_RESULT   0x41U
#define TMC2209_REG_MSCNT       0x6AU
#define TMC2209_REG_CHOPCONF    0x6CU
#define TMC2209_REG_DRV_STATUS  0x6FU
#define TMC2209_REG_PWMCONF     0x70U
#define TMC2209_REG_PWM_SCALE   0x71U
 
/* ============================================================================
 * UART DATAGRAM
 * ============================================================================ */
#define TMC2209_SYNC_BYTE       0x05U
#define TMC2209_MASTER_ADDR     0x00U
#define TMC2209_WRITE_BIT       0x80U
#define TMC2209_WRITE_DATAGRAM_LEN  8U
#define TMC2209_READ_REQ_LEN        4U
#define TMC2209_READ_REPLY_LEN      8U
 
/* ============================================================================
 * ENUMS
 * ============================================================================ */
 
typedef enum
{
    TMC2209_OK          = 0x00U,
    TMC2209_ERROR       = 0x01U,
    TMC2209_TIMEOUT     = 0x02U,
    TMC2209_CRC_ERROR   = 0x03U
} TMC2209_StatusTypeDef;
 
typedef enum
{
    TMC2209_DIR_CCW = 0U,   /* Quay nguoc chieu kim dong ho */
    TMC2209_DIR_CW  = 1U    /* Quay thuan chieu kim dong ho */
} TMC2209_DirectionTypeDef;
 
typedef enum
{
    TMC2209_ENABLE  = 0U,   /* Chan EN active-LOW */
    TMC2209_DISABLE = 1U
} TMC2209_EnableTypeDef;
 
typedef enum
{
    TMC2209_SLAVE_ADD1 = 0x00U,
    TMC2209_SLAVE_ADD2 = 0x01U,
    TMC2209_SLAVE_ADD3 = 0x02U,
    TMC2209_SLAVE_ADD4 = 0x03U
} TMC2209_SlaveAddressTypeDef;
 
typedef enum
{
    TMC2209_MICROSTEP_FULL = 1U,
    TMC2209_MICROSTEP_2    = 2U,
    TMC2209_MICROSTEP_4    = 4U,
    TMC2209_MICROSTEP_8    = 8U,
    TMC2209_MICROSTEP_16   = 16U,
    TMC2209_MICROSTEP_32   = 32U,
    TMC2209_MICROSTEP_64   = 64U,
    TMC2209_MICROSTEP_128  = 128U,
    TMC2209_MICROSTEP_256  = 256U
} TMC2209_MicrostepTypeDef;
 
typedef enum
{
    TMC2209_STOPPED = 0U,
    TMC2209_RUNNING = 1U
} TMC2209_StateTypeDef;
 
/* ============================================================================
 * HANDLE STRUCT
 * ============================================================================ */
typedef struct
{
    uint8_t id;                         /* ID driver (0..2) */
 
    /* --- PWM STEP --- */
    TIM_HandleTypeDef  *htim;           /* Timer tao xung STEP */
    uint32_t            tim_channel;    /* Kenh PWM (TIM_CHANNEL_1 ... TIM_CHANNEL_4) */
 
    /* --- GPIO DIR / EN --- */
    GPIO_TypeDef       *dir_port;
    uint16_t            dir_pin;
    GPIO_TypeDef       *en_port;
    uint16_t            en_pin;
 
    /* --- Timer clock --- */
    uint32_t            timer_clock_hz; /* Clock cap vao timer sau APB prescaler */
    uint32_t            prescaler;      /* Prescaler cua timer (gia tri da set trong CubeMX) */
 
    /* --- Motor config --- */
    uint16_t                steps_per_rev;  /* So buoc full-step / vong */
    TMC2209_MicrostepTypeDef microstep;     /* So microstep */
 
    /* --- Trang thai --- */
    TMC2209_DirectionTypeDef direction;
    TMC2209_EnableTypeDef    enable_state;
    TMC2209_StateTypeDef     state;
 
    /* --- Toc do & vi tri --- */
    uint32_t speed_hz;          /* Tan so xung STEP hien tai (Hz) */
    uint32_t target_steps;      /* So buoc can di */
    uint32_t current_steps;     /* So buoc da di */
    float    current_angle;     /* Goc hien tai (do) */
    float    target_angle;      /* Goc muc tieu (do) */
 
    /* --- UART address --- */
    TMC2209_SlaveAddressTypeDef slave_address;
 
    /* --- UART register cache (tuy chon) --- */
    uint32_t reg_gconf;
    uint32_t reg_chopconf;
    uint32_t reg_ihold_irun;
 
} TMC2209_HandleTypeDef;
 
/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */
 
/* --- Khoi tao & cau hinh co ban ------------------------------------------ */
 
/**
 * @brief  Khoi tao driver TMC2209: cai dat GPIO, UART, ghi cac register mac dinh
 * @param  hmotor  Con tro toi struct TMC2209_HandleTypeDef da duoc nguoi dung dien
 * @retval TMC2209_StatusTypeDef
 */
TMC2209_StatusTypeDef TMC2209_Init(TMC2209_HandleTypeDef *hmotor);
 
/**
 * @brief  Dat dong dien chay / giu (0..31 ~ 0..100% Imax chip)
 * @param  run_current  0..31
 * @param  hold_current 0..31
 * @param  hold_delay   0..15 (x 2^18 clock, do tre truoc giam dong)
 */
TMC2209_StatusTypeDef TMC2209_SetCurrent(TMC2209_HandleTypeDef *hmotor, uint8_t run_current, uint8_t hold_current, uint8_t hold_delay);
 
/**
 * @brief  Dat so microstep (cap nhat ca CHOPCONF va steps_per_rev cache)
 */
TMC2209_StatusTypeDef TMC2209_SetMicrostep(TMC2209_HandleTypeDef *hmotor, TMC2209_MicrostepTypeDef microstep);
 
/**
 * @brief  Bat / tat StealthChop (giam on ao o toc do thap)
 * @param  enable  true = StealthChop, false = SpreadCycle
 */
TMC2209_StatusTypeDef TMC2209_SetStealthChop(TMC2209_HandleTypeDef *hmotor, bool enable);
 
/* --- Enable / Disable motor ----------------------------------------------- */
 
/**
 * @brief  Bat dong co (keo chan EN xuong thap)
 */
void TMC2209_Enable(TMC2209_HandleTypeDef *hmotor);
 
/**
 * @brief  Tat dong co (dua chan EN len cao -> driver tri)
 */
void TMC2209_Disable(TMC2209_HandleTypeDef *hmotor);
 
/* --- Dieu khien huong quay ----------------------------------------------- */
 
/**
 * @brief  Dat chieu quay
 */
void TMC2209_SetDirection(TMC2209_HandleTypeDef *hmotor, TMC2209_DirectionTypeDef dir);
 
/* --- Dieu khien toc do (PWM) ---------------------------------------------- */
 
/**
 * @brief  Dat toc do theo tan so xung STEP (Hz)
 *         Ham tu dong tinh ARR cua Timer
 * @param  freq_hz  Tan so xung mong muon
 */
TMC2209_StatusTypeDef TMC2209_SetSpeedHz(TMC2209_HandleTypeDef *hmotor, uint32_t freq_hz);
 
/**
 * @brief  Dat toc do theo vong/phut (RPM)
 */
TMC2209_StatusTypeDef TMC2209_SetSpeedRPM(TMC2209_HandleTypeDef *hmotor, float rpm);
 
/**
 * @brief  Bat dau quay (start PWM)
 */
TMC2209_StatusTypeDef TMC2209_Start(TMC2209_HandleTypeDef *hmotor);
 
/**
 * @brief  Dung quay (stop PWM)
 */
void TMC2209_Stop(TMC2209_HandleTypeDef *hmotor);
 
/* --- Di chuyen theo so buoc / goc ---------------------------------------- */
 
/**
 * @brief  Quay them N buoc (khong blocking)
 *         Goi TMC2209_UpdateSteps() trong interrupt dem buoc de biet khi nao dung
 */
TMC2209_StatusTypeDef TMC2209_MoveSteps(TMC2209_HandleTypeDef *hmotor, uint32_t steps, TMC2209_DirectionTypeDef dir, float speed_rpm);
 
/**
 * @brief  Quay den goc tuyet doi (do, 0..360)
 */
TMC2209_StatusTypeDef TMC2209_MoveToAngle(TMC2209_HandleTypeDef *hmotor, float angle_deg, float speed_rpm);
 
/**
 * @brief  Goi trong Timer Period Elapsed ISR hoac PWM Pulse Finished ISR
 *         de dem buoc va tu dong Stop khi du
 * @retval true neu motor vua dung (da di du buoc)
 */
bool TMC2209_UpdateSteps(TMC2209_HandleTypeDef *hmotor);
 
/* --- Doc trang thai driver ------------------------------------------------ */
 
/**
 * @brief  Doc register DRV_STATUS
 */
TMC2209_StatusTypeDef TMC2209_ReadDrvStatus(TMC2209_HandleTypeDef *hmotor, uint32_t *status_out);
 
/**
 * @brief  Doc register SG_RESULT (stallguard)
 */
TMC2209_StatusTypeDef TMC2209_ReadSGResult(TMC2209_HandleTypeDef *hmotor, uint16_t *sg_out);
 
/**
 * @brief  Kiem tra co bi ket dong co khong (qua stallguard)
 * @param  threshold  Nguong SG (0..255) - gia tri nho = de bi ket
 * @retval true neu dang bi ket
 */
bool TMC2209_IsStalled(TMC2209_HandleTypeDef *hmotor, uint8_t threshold);
 
/**
 * @brief  Kiem tra driver co loi nhiet / short khong
 * @retval true neu co loi
 */
bool TMC2209_HasFault(TMC2209_HandleTypeDef *hmotor);
 
/* --- UART register read/write thap cap ------------------------------------ */
 
/**
 * @brief  Ghi 1 register TMC2209 qua UART
 */
TMC2209_StatusTypeDef TMC2209_WriteReg(TMC2209_HandleTypeDef *hmotor, uint8_t reg_addr, uint32_t value);
 
/**
 * @brief  Doc 1 register TMC2209 qua UART
 */
TMC2209_StatusTypeDef TMC2209_ReadReg(TMC2209_HandleTypeDef *hmotor, uint8_t reg_addr, uint32_t *value_out);
 

/**
 * @brief  Tinh tan so xung tuong ung voi RPM
 *         freq = (RPM * steps_per_rev * microstep) / 60
 */
static inline uint32_t TMC2209_RPM_to_Hz(const TMC2209_HandleTypeDef *hmotor, float rpm)
{
    return (uint32_t)((rpm * (float)hmotor->steps_per_rev * (float)hmotor->microstep) / 60.0f);
}
 
/**
 * @brief  Tinh RPM tuong ung voi tan so xung
 */
static inline float TMC2209_Hz_to_RPM(const TMC2209_HandleTypeDef *hmotor, uint32_t hz)
{
    return ((float)hz * 60.0f) / ((float)hmotor->steps_per_rev * (float)hmotor->microstep);
}



#endif /* MYLIB_INC_TMC2209_H_ */
