/*
 * TMC2209.c
 *
 *  Created on: May 14, 2026
 *      Author: Lap4all
 *******************************************************************************
 * @file    tmc2209.c
 * @brief   Implementation thu vien TMC2209
 *          UART3 + PWM STEP control cho toi da 3 driver
 ******************************************************************************
 *
 * KET NOI PHAN CUNG (goi y):
 *  TMC2209 #1: UART addr 0x00, chan STEP -> TIM_CH1, DIR -> PA0, EN -> PA1
 *  TMC2209 #2: UART addr 0x01, chan STEP -> TIM_CH2, DIR -> PA2, EN -> PA3
 *  TMC2209 #3: UART addr 0x02, chan STEP -> TIM_CH3, DIR -> PA4, EN -> PA5
 *
 *  TX (PA2/PB10/...) noi chung 1 duong den tat ca PDN_UART cua cac TMC2209
 *  RX (PA3/PB11/...) noi chung 1 duong nhan tu cac TMC2209
 *  Phan biet driver bang UART slave address (set bang 2 chan MS1, MS2 cua TMC2209)
 *
 * UART: 115200 baud, 8N1, SINGLE-WIRE half-duplex (hoac full-duplex)
 *
 ******************************************************************************
 */


#include "TMC2209.h"
#include "my_config.h"


/* ============================================================================
 * PRIVATE: CRC-8 DATAGRAM
 * TMC2209 dung CRC-8 voi polynomial 0x07 (byte-by-byte, init=0)
 * ============================================================================ */
 
static uint8_t _TMC2209_CalcCRC(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t byte = data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if ((crc >> 7) ^ (byte & 0x01))
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc = (uint8_t)(crc << 1);
            byte >>= 1;
        }
    }
    return crc;
}
 
/* ============================================================================
 * PRIVATE: UART DATAGRAM BUILD & SEND
 * ============================================================================ */
 
/**
 * @brief  Ghi register TMC2209 qua UART
 *         Write datagram: SYNC | ADDR | REG|0x80 | DATA[3] DATA[2] DATA[1] DATA[0] | CRC
 */
TMC2209_StatusTypeDef TMC2209_WriteReg(TMC2209_HandleTypeDef *hmotor,
                                       uint8_t reg_addr,
                                       uint32_t value)
{
    uint8_t buf[TMC2209_WRITE_DATAGRAM_LEN];
 
    buf[0] = TMC2209_SYNC_BYTE;
    buf[1] = (uint8_t)hmotor->slave_address;
    buf[2] = reg_addr | TMC2209_WRITE_BIT;
    buf[3] = (uint8_t)((value >> 24) & 0xFF);
    buf[4] = (uint8_t)((value >> 16) & 0xFF);
    buf[5] = (uint8_t)((value >>  8) & 0xFF);
    buf[6] = (uint8_t)((value      ) & 0xFF);
    buf[7] = _TMC2209_CalcCRC(buf, 7);
 
    HAL_StatusTypeDef ret = HAL_UART_Transmit(TMC2209_UART, buf,
                                               TMC2209_WRITE_DATAGRAM_LEN,
                                               TMC2209_UART_TIMEOUT);
    if (ret == HAL_TIMEOUT) return TMC2209_TIMEOUT;
    if (ret != HAL_OK)      return TMC2209_ERROR;
    return TMC2209_OK;
}
 
/**
 * @brief  Doc register TMC2209 qua UART
 *         Read request (4 byte) -> nhan reply (8 byte)
 *
 * LUU Y: TMC2209 se echo lai 4 byte request truoc khi gui 8 byte reply.
 *        Neu dung UART full-duplex, can flush/ignore 4 byte dau nhan duoc.
 *        Neu dung UART half-duplex (single-wire), phan cung tu xu ly.
 *        Doan code nay gia su full-duplex co RX/TX rieng.
 */
TMC2209_StatusTypeDef TMC2209_ReadReg(TMC2209_HandleTypeDef *hmotor,
                                      uint8_t reg_addr,
                                      uint32_t *value_out)
{
    uint8_t req[TMC2209_READ_REQ_LEN];
    uint8_t echo[TMC2209_READ_REQ_LEN];
    uint8_t reply[TMC2209_READ_REPLY_LEN];
 
    /* --- Build read request --- */
    req[0] = TMC2209_SYNC_BYTE;
    req[1] = (uint8_t)hmotor->slave_address;
    req[2] = reg_addr & 0x7FU;  /* bit write = 0 */
    req[3] = _TMC2209_CalcCRC(req, 3);
 
    HAL_StatusTypeDef ret;
 
    /* --- Gui request --- */
    ret = HAL_UART_Transmit(TMC2209_UART, req, TMC2209_READ_REQ_LEN, TMC2209_UART_TIMEOUT);
    if (ret == HAL_TIMEOUT) return TMC2209_TIMEOUT;
    if (ret != HAL_OK)      return TMC2209_ERROR;
 
    /* --- Nhan echo (TMC2209 echo lai 4 byte request) --- */
    ret = HAL_UART_Receive(TMC2209_UART, echo, TMC2209_READ_REQ_LEN, TMC2209_UART_TIMEOUT);
    if (ret == HAL_TIMEOUT) return TMC2209_TIMEOUT;
    if (ret != HAL_OK)      return TMC2209_ERROR;
 
    /* --- Nhan reply 8 byte --- */
    ret = HAL_UART_Receive(TMC2209_UART, reply, TMC2209_READ_REPLY_LEN, TMC2209_UART_TIMEOUT);
    if (ret == HAL_TIMEOUT) return TMC2209_TIMEOUT;
    if (ret != HAL_OK)      return TMC2209_ERROR;
 
    /* --- Kiem tra CRC reply --- */
    uint8_t crc_calc = _TMC2209_CalcCRC(reply, 7);
    if (crc_calc != reply[7]) return TMC2209_CRC_ERROR;
 
    /* --- Lay gia tri 4 byte data (MSB truoc) --- */
    *value_out = ((uint32_t)reply[3] << 24)
               | ((uint32_t)reply[4] << 16)
               | ((uint32_t)reply[5] <<  8)
               | ((uint32_t)reply[6]);
 
    return TMC2209_OK;
}
 
/* ============================================================================
 * PRIVATE: CAP NHAT PWM (tinh ARR tu freq)
 * Timer freq xung = timer_clock_hz / (prescaler+1) / (ARR+1)
 *   => ARR = timer_clock_hz / (prescaler+1) / freq_hz  - 1
 * Duty cycle 50% => CCR = (ARR+1)/2
 * ============================================================================ */
 
static TMC2209_StatusTypeDef _TMC2209_SetPWMFreq(TMC2209_HandleTypeDef *hmotor,
                                                  uint32_t freq_hz)
{
    if (freq_hz == 0) return TMC2209_ERROR;
 
    uint32_t timer_input_clk = hmotor->timer_clock_hz / (hmotor->prescaler + 1U);
    uint32_t arr = (timer_input_clk / freq_hz);
    if (arr < 2U) arr = 2U;   /* Dam bao it nhat duty co y nghia */
    arr -= 1U;
 
    uint32_t ccr = (arr + 1U) / 2U;  /* duty 50% */
 
    __HAL_TIM_SET_AUTORELOAD(hmotor->htim, arr);
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->tim_channel, ccr);
 
    hmotor->speed_hz = freq_hz;
    return TMC2209_OK;
}
 
/* ============================================================================
 * PRIVATE: Chuyen microstep sang gia tri bit MRES trong CHOPCONF
 * MRES[3:0]: 0=256, 1=128, 2=64, 3=32, 4=16, 5=8, 6=4, 7=2, 8=full
 * ============================================================================ */
 
static uint8_t _TMC2209_MicrostepToMRES(TMC2209_MicrostepTypeDef ms)
{
    switch (ms)
    {
        case TMC2209_MICROSTEP_256:  return 0U;
        case TMC2209_MICROSTEP_128:  return 1U;
        case TMC2209_MICROSTEP_64:   return 2U;
        case TMC2209_MICROSTEP_32:   return 3U;
        case TMC2209_MICROSTEP_16:   return 4U;
        case TMC2209_MICROSTEP_8:    return 5U;
        case TMC2209_MICROSTEP_4:    return 6U;
        case TMC2209_MICROSTEP_2:    return 7U;
        case TMC2209_MICROSTEP_FULL: return 8U;
        default:                     return 4U; /* mac dinh 16 */
    }
}
 
/* ============================================================================
 * KHOI TAO
 * ============================================================================ */
 
TMC2209_StatusTypeDef TMC2209_Init(TMC2209_HandleTypeDef *hmotor)
{
    TMC2209_StatusTypeDef ret;
 
    /* --- 1. Tat driver, set huong mac dinh --- */
    HAL_GPIO_WritePin(hmotor->en_port,  hmotor->en_pin,  GPIO_PIN_SET);   /* EN=HIGH -> disable */
    HAL_GPIO_WritePin(hmotor->dir_port, hmotor->dir_pin, GPIO_PIN_RESET); /* DIR=0 */
 
    hmotor->enable_state = TMC2209_DISABLE;
    hmotor->direction    = TMC2209_DIR_CCW;
    hmotor->state        = TMC2209_STOPPED;
    hmotor->current_steps = 0;
    hmotor->current_angle = 0.0f;
 
    /* Cho driver on dinh sau reset */
    HAL_Delay(50);
 
    /* --- 2. Cau hinh GCONF ---
     *   bit 0  (I_scale_analog)    = 0  -> dung thanh ghi noi
     *   bit 1  (internal_Rsense)   = 0  -> Rsense ngoai
     *   bit 2  (en_spreadCycle)    = 0  -> StealthChop mac dinh
     *   bit 3  (shaft)             = 0
     *   bit 4  (index_otpw)        = 0
     *   bit 5  (index_step)        = 0
     *   bit 6  (pdn_disable)       = 1  -> tat PDN, dung UART
     *   bit 7  (mstep_reg_select)  = 1  -> chon microstep qua register
     *   bit 8  (multistep_filt)    = 1  -> bo loc nhieu xung STEP
     */
    hmotor->reg_gconf = (1U << 6) | (1U << 7) | (1U << 8);
    ret = TMC2209_WriteReg(hmotor, TMC2209_REG_GCONF, hmotor->reg_gconf);
    if (ret != TMC2209_OK) return ret;
 
    /* --- 3. Cau hinh CHOPCONF ---
     *   MRES  [27:24] = microstep
     *   vsense [17]   = 0
     *   tbl    [16:15]= 01 (blank time 24 clock)
     *   hend   [10:7] = 0
     *   hstrt  [6:4]  = 4
     *   toff   [3:0]  = 3 (bat buoc phai != 0)
     */
    uint8_t mres = _TMC2209_MicrostepToMRES(hmotor->microstep);
    hmotor->reg_chopconf = ((uint32_t)mres << 24)
                         | (0x01U << 15)    /* tbl=01 */
                         | (4U    <<  4)    /* hstrt=4 */
                         | (3U         );   /* toff=3  */
    ret = TMC2209_WriteReg(hmotor, TMC2209_REG_CHOPCONF, hmotor->reg_chopconf);
    if (ret != TMC2209_OK) return ret;
 
    /* --- 4. Cau hinh IHOLD_IRUN mac dinh ---
     *   IHOLD [4:0]   = 8  (25% Imax giu)
     *   IRUN  [12:8]  = 20 (63% Imax chay)
     *   IHOLDDELAY [19:16] = 6
     */
    hmotor->reg_ihold_irun = (6U << 16) | (20U << 8) | (8U);
    ret = TMC2209_WriteReg(hmotor, TMC2209_REG_IHOLD_IRUN, hmotor->reg_ihold_irun);
    if (ret != TMC2209_OK) return ret;
 
    /* --- 5. TPOWERDOWN: thoi gian cho truoc giam dong (0..255 x 2^18 clk) --- */
    ret = TMC2209_WriteReg(hmotor, TMC2209_REG_TPOWERDOWN, 20U);
    if (ret != TMC2209_OK) return ret;
 
    /* --- 6. Dat toc do mac dinh (100 Hz) --- */
    _TMC2209_SetPWMFreq(hmotor, 100U);
    hmotor->speed_hz = 100U;
 
    return TMC2209_OK;
}
 
/* ============================================================================
 * CAU HINH DONG DIEN
 * ============================================================================ */
 
TMC2209_StatusTypeDef TMC2209_SetCurrent(TMC2209_HandleTypeDef *hmotor,
                                         uint8_t run_current,
                                         uint8_t hold_current,
                                         uint8_t hold_delay)
{
    if (run_current  > 31U) run_current  = 31U;
    if (hold_current > 31U) hold_current = 31U;
    if (hold_delay   > 15U) hold_delay   = 15U;
 
    hmotor->reg_ihold_irun = ((uint32_t)hold_delay   << 16)
                           | ((uint32_t)run_current   <<  8)
                           | ((uint32_t)hold_current       );
 
    return TMC2209_WriteReg(hmotor, TMC2209_REG_IHOLD_IRUN, hmotor->reg_ihold_irun);
}
 
/* ============================================================================
 * CAU HINH MICROSTEP
 * ============================================================================ */
 
TMC2209_StatusTypeDef TMC2209_SetMicrostep(TMC2209_HandleTypeDef *hmotor,
                                           TMC2209_MicrostepTypeDef microstep)
{
    hmotor->microstep = microstep;
    uint8_t mres = _TMC2209_MicrostepToMRES(microstep);
 
    /* Cap nhat bit MRES trong chopconf cache, giu nguyen cac bit khac */
    hmotor->reg_chopconf &= ~(0x0FUL << 24);                   /* xoa MRES cu */
    hmotor->reg_chopconf |=  ((uint32_t)mres << 24);           /* ghi MRES moi */
 
    return TMC2209_WriteReg(hmotor, TMC2209_REG_CHOPCONF, hmotor->reg_chopconf);
}
 
/* ============================================================================
 * STEALTHCHOP / SPREADCYCLE
 * ============================================================================ */
 
TMC2209_StatusTypeDef TMC2209_SetStealthChop(TMC2209_HandleTypeDef *hmotor, bool enable)
{
    if (enable)
        hmotor->reg_gconf &= ~(1U << 2); /* en_spreadCycle=0 -> StealthChop */
    else
        hmotor->reg_gconf |=  (1U << 2); /* en_spreadCycle=1 -> SpreadCycle  */
 
    return TMC2209_WriteReg(hmotor, TMC2209_REG_GCONF, hmotor->reg_gconf);
}
 
/* ============================================================================
 * ENABLE / DISABLE
 * ============================================================================ */
 
void TMC2209_Enable(TMC2209_HandleTypeDef *hmotor)
{
    HAL_GPIO_WritePin(hmotor->en_port, hmotor->en_pin, GPIO_PIN_RESET); /* EN=LOW */
    hmotor->enable_state = TMC2209_ENABLE;
}
 
void TMC2209_Disable(TMC2209_HandleTypeDef *hmotor)
{
    HAL_GPIO_WritePin(hmotor->en_port, hmotor->en_pin, GPIO_PIN_SET);   /* EN=HIGH */
    hmotor->enable_state = TMC2209_DISABLE;
}
 
/* ============================================================================
 * HUONG QUAY
 * ============================================================================ */
 
void TMC2209_SetDirection(TMC2209_HandleTypeDef *hmotor,
                          TMC2209_DirectionTypeDef dir)
{
    hmotor->direction = dir;
    HAL_GPIO_WritePin(hmotor->dir_port, hmotor->dir_pin,
                      (dir == TMC2209_DIR_CW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
 
/* ============================================================================
 * TOC DO PWM
 * ============================================================================ */
 
TMC2209_StatusTypeDef TMC2209_SetSpeedHz(TMC2209_HandleTypeDef *hmotor,
                                         uint32_t freq_hz)
{
    return _TMC2209_SetPWMFreq(hmotor, freq_hz);
}
 
TMC2209_StatusTypeDef TMC2209_SetSpeedRPM(TMC2209_HandleTypeDef *hmotor,
                                          float rpm)
{
    if (rpm <= 0.0f) return TMC2209_ERROR;
    uint32_t hz = TMC2209_RPM_to_Hz(hmotor, rpm);
    if (hz == 0U) hz = 1U;
    return _TMC2209_SetPWMFreq(hmotor, hz);
}
 
/* ============================================================================
 * START / STOP
 * ============================================================================ */
 
TMC2209_StatusTypeDef TMC2209_Start(TMC2209_HandleTypeDef *hmotor)
{
    if (hmotor->enable_state == TMC2209_DISABLE)
        TMC2209_Enable(hmotor);
 
    HAL_StatusTypeDef ret = HAL_TIM_PWM_Start_IT(hmotor->htim, hmotor->tim_channel);
    if (ret != HAL_OK) return TMC2209_ERROR;
 
    hmotor->state = TMC2209_RUNNING;
    return TMC2209_OK;
}
 
void TMC2209_Stop(TMC2209_HandleTypeDef *hmotor)
{
    (void)HAL_TIM_PWM_Stop_IT(hmotor->htim, hmotor->tim_channel);
    hmotor->state = TMC2209_STOPPED;
}
 
/* ============================================================================
 * DI CHUYEN THEO SO BUOC
 * ============================================================================ */
 
TMC2209_StatusTypeDef TMC2209_MoveSteps(TMC2209_HandleTypeDef *hmotor,
                                        uint32_t steps,
                                        TMC2209_DirectionTypeDef dir,
                                        float speed_rpm)
{
    TMC2209_StatusTypeDef ret;
 
    TMC2209_SetDirection(hmotor, dir);
 
    ret = TMC2209_SetSpeedRPM(hmotor, speed_rpm);
    if (ret != TMC2209_OK) return ret;
 
    hmotor->target_steps  = steps;
    hmotor->current_steps = 0U;
 
    return TMC2209_Start(hmotor);
}
 
/* ============================================================================
 * DI CHUYEN THEO GOC
 * ============================================================================ */
 
TMC2209_StatusTypeDef TMC2209_MoveToAngle(TMC2209_HandleTypeDef *hmotor,
                                          float angle_deg,
                                          float speed_rpm)
{
    /* Tinh delta goc */
    float delta = angle_deg - hmotor->current_angle;
 
    TMC2209_DirectionTypeDef dir;
    if (delta < 0.0f)
    {
        dir   = TMC2209_DIR_CCW;
        delta = -delta;
    }
    else
    {
        dir = TMC2209_DIR_CW;
    }
 
    /* Tinh so buoc can di
     *   steps = delta_deg / 360 * steps_per_rev * microstep
     */
    float steps_f = (delta / 360.0f) * (float)hmotor->steps_per_rev * (float)hmotor->microstep;
    uint32_t steps = (uint32_t)(steps_f + 0.5f);  /* lam tron */
 
    if (steps == 0U) return TMC2209_OK;  /* da o dung vi tri */
 
    hmotor->target_angle = angle_deg;
 
    return TMC2209_MoveSteps(hmotor, steps, dir, speed_rpm);
}
 
/* ============================================================================
 * CAP NHAT BUOC - GOI TRONG INTERRUPT
 *
 * VI DU: Goi trong HAL_TIM_PWM_PulseFinishedCallback() hoac
 *         timer period elapsed callback neu chia nho.
 *
 * void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
 * {
 *     if (htim == motor1.htim) TMC2209_UpdateSteps(&motor1);
 *     if (htim == motor2.htim) TMC2209_UpdateSteps(&motor2);
 *     if (htim == motor3.htim) TMC2209_UpdateSteps(&motor3);
 * }
 * ============================================================================ */
 
bool TMC2209_UpdateSteps(TMC2209_HandleTypeDef *hmotor)
{
    if (hmotor->state != TMC2209_RUNNING) return false;
    if (hmotor->target_steps == 0U)       return false; /* che do chay lien tuc */
 
    hmotor->current_steps++;
 
    if (hmotor->current_steps >= hmotor->target_steps)
    {
        TMC2209_Stop(hmotor);
 
        /* Cap nhat goc hien tai */
        float delta_deg = ((float)hmotor->target_steps /
                           ((float)hmotor->steps_per_rev * (float)hmotor->microstep)) * 360.0f;
 
        if (hmotor->direction == TMC2209_DIR_CW)
            hmotor->current_angle += delta_deg;
        else
            hmotor->current_angle -= delta_deg;
 
        /* Giu goc trong [0, 360) */
        while (hmotor->current_angle <   0.0f)   hmotor->current_angle += 360.0f;
        while (hmotor->current_angle >= 360.0f)  hmotor->current_angle -= 360.0f;
 
        hmotor->target_steps  = 0U;
        hmotor->current_steps = 0U;
        return true;
    }
    return false;
}
 
/* ============================================================================
 * DOC TRANG THAI DRIVER
 * ============================================================================ */
 
TMC2209_StatusTypeDef TMC2209_ReadDrvStatus(TMC2209_HandleTypeDef *hmotor,
                                            uint32_t *status_out)
{
    return TMC2209_ReadReg(hmotor, TMC2209_REG_DRV_STATUS, status_out);
}
 
TMC2209_StatusTypeDef TMC2209_ReadSGResult(TMC2209_HandleTypeDef *hmotor,
                                           uint16_t *sg_out)
{
    uint32_t val = 0;
    TMC2209_StatusTypeDef ret = TMC2209_ReadReg(hmotor, TMC2209_REG_SG_RESULT, &val);
    if (ret == TMC2209_OK)
        *sg_out = (uint16_t)(val & 0x1FFU);  /* bit [8:0] */
    return ret;
}
 
bool TMC2209_IsStalled(TMC2209_HandleTypeDef *hmotor, uint8_t threshold)
{
    uint16_t sg = 0;
    if (TMC2209_ReadSGResult(hmotor, &sg) != TMC2209_OK) return false;
    return (sg < (uint16_t)threshold);
}
 
bool TMC2209_HasFault(TMC2209_HandleTypeDef *hmotor)
{
    uint32_t status = 0;
    if (TMC2209_ReadDrvStatus(hmotor, &status) != TMC2209_OK) return false;
 
    /* DRV_STATUS bit 27: overtemperature prewarning
     *             bit 25: short to GND phase B
     *             bit 24: short to GND phase A
     *             bit 26: open load phase B
     *             bit 29: ot (overtemperature shutdown)
     */
    uint32_t fault_mask = (1U << 29) | (1U << 25) | (1U << 24);
    return ((status & fault_mask) != 0U);
}
