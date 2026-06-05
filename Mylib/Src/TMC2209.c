/**
 * @file    TMC2209.c
 * @brief   Trien khai driver TMC2209 qua UART va PWM STEP.
 * @author  Lap4all
 * @date    2026-05-14
 *
 * @note Ket noi goi y:
 *       TMC2209 #1: UART addr 0x00, STEP -> TIM_CH1, DIR -> PA0, EN -> PA1.
 *       TMC2209 #2: UART addr 0x01, STEP -> TIM_CH2, DIR -> PA2, EN -> PA3.
 *       TMC2209 #3: UART addr 0x02, STEP -> TIM_CH3, DIR -> PA4, EN -> PA5.
 *
 * @note Cac driver co the dung chung duong TX/RX UART va phan biet bang
 *       slave address duoc cau hinh tu chan MS1/MS2 cua TMC2209.
 */

#include "TMC2209.h"
#include "my_config.h"

/* Private functions -------------------------------------------------------- */

/**
 * @brief  Tinh CRC-8 cho datagram UART cua TMC2209.
 * @param[in] data: Mang byte can tinh CRC.
 * @param[in] len: So byte dau vao.
 * @return Gia tri CRC-8 voi polynomial 0x07.
 */
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

/* Public functions: UART register access ---------------------------------- */

/**
 * @brief  Ghi mot register TMC2209 qua UART.
 * @param[in] hmotor: Con tro den handle TMC2209.
 * @param[in] reg_addr: Dia chi register can ghi.
 * @param[in] value: Gia tri 32-bit can ghi.
 * @return Trang thai giao tiep UART.
 *
 * @note Write datagram gom SYNC, ADDR, REG|0x80, DATA[31:0] va CRC.
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
 * @brief  Doc mot register TMC2209 qua UART.
 * @param[in] hmotor: Con tro den handle TMC2209.
 * @param[in] reg_addr: Dia chi register can doc.
 * @param[out] value_out: Noi luu gia tri 32-bit doc duoc.
 * @return Trang thai giao tiep UART hoac CRC.
 *
 * @note TMC2209 echo lai 4 byte request truoc khi tra ve reply 8 byte.
 */
TMC2209_StatusTypeDef TMC2209_ReadReg(TMC2209_HandleTypeDef *hmotor,
                                      uint8_t reg_addr,
                                      uint32_t *value_out)
{
    uint8_t req[TMC2209_READ_REQ_LEN];
    uint8_t echo[TMC2209_READ_REQ_LEN];
    uint8_t reply[TMC2209_READ_REPLY_LEN];

    req[0] = TMC2209_SYNC_BYTE;
    req[1] = (uint8_t)hmotor->slave_address;
    req[2] = reg_addr & 0x7FU;  /* Bit write = 0. */
    req[3] = _TMC2209_CalcCRC(req, 3);

    HAL_StatusTypeDef ret;

    ret = HAL_UART_Transmit(TMC2209_UART, req, TMC2209_READ_REQ_LEN, TMC2209_UART_TIMEOUT);
    if (ret == HAL_TIMEOUT) return TMC2209_TIMEOUT;
    if (ret != HAL_OK)      return TMC2209_ERROR;

    ret = HAL_UART_Receive(TMC2209_UART, echo, TMC2209_READ_REQ_LEN, TMC2209_UART_TIMEOUT);
    if (ret == HAL_TIMEOUT) return TMC2209_TIMEOUT;
    if (ret != HAL_OK)      return TMC2209_ERROR;

    ret = HAL_UART_Receive(TMC2209_UART, reply, TMC2209_READ_REPLY_LEN, TMC2209_UART_TIMEOUT);
    if (ret == HAL_TIMEOUT) return TMC2209_TIMEOUT;
    if (ret != HAL_OK)      return TMC2209_ERROR;

    uint8_t crc_calc = _TMC2209_CalcCRC(reply, 7);
    if (crc_calc != reply[7]) return TMC2209_CRC_ERROR;

    *value_out = ((uint32_t)reply[3] << 24)
               | ((uint32_t)reply[4] << 16)
               | ((uint32_t)reply[5] <<  8)
               | ((uint32_t)reply[6]);

    return TMC2209_OK;
}

/* Private functions: PWM and register conversion -------------------------- */

/**
 * @brief  Dat tan so PWM STEP cho mot driver TMC2209.
 * @param[in,out] hmotor: Con tro den handle TMC2209.
 * @param[in] freq_hz: Tan so xung STEP mong muon, tinh bang Hz.
 * @return TMC2209_OK neu cap nhat timer thanh cong.
 *
 * @note Timer STEP dung duty 50%. ARR duoc tinh theo cong thuc:
 *       freq = timer_clock_hz / (prescaler + 1) / (ARR + 1).
 */
static TMC2209_StatusTypeDef _TMC2209_SetPWMFreq(TMC2209_HandleTypeDef *hmotor,
                                                  uint32_t freq_hz)
{
    if (freq_hz == 0) return TMC2209_ERROR;

    uint32_t timer_input_clk = hmotor->timer_clock_hz / (hmotor->prescaler + 1U);
    uint32_t arr = (timer_input_clk / freq_hz);
    if (arr < 2U) arr = 2U;   /* Dam bao duty 50% van co y nghia. */
    arr -= 1U;

    uint32_t ccr = (arr + 1U) / 2U;  /* Duty 50%. */

    __HAL_TIM_SET_AUTORELOAD(hmotor->htim, arr);
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->tim_channel, ccr);

    hmotor->speed_hz = freq_hz;
    return TMC2209_OK;
}

/**
 * @brief  Chuyen so microstep sang gia tri MRES trong CHOPCONF.
 * @param[in] ms: So microstep cau hinh cho driver.
 * @return Gia tri MRES[3:0] tuong ung.
 */
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
        default:                     return 4U; /* Mac dinh 16 microstep. */
    }
}

/* Public functions: initialization and configuration ----------------------- */

/**
 * @brief  Khoi tao driver TMC2209 voi cac register mac dinh.
 * @param[in,out] hmotor: Con tro den handle TMC2209 da duoc cau hinh chan.
 * @return TMC2209_OK neu khoi tao thanh cong, nguoc lai tra ve ma loi.
 */
TMC2209_StatusTypeDef TMC2209_Init(TMC2209_HandleTypeDef *hmotor)
{
    TMC2209_StatusTypeDef ret;

    /* Tat driver truoc khi ghi cau hinh de tranh motor quay ngoai y muon. */
    HAL_GPIO_WritePin(hmotor->en_port,  hmotor->en_pin,  GPIO_PIN_SET);   /* EN=HIGH -> disable. */
    HAL_GPIO_WritePin(hmotor->dir_port, hmotor->dir_pin, GPIO_PIN_RESET); /* DIR=0. */

    hmotor->enable_state = TMC2209_DISABLE;
    hmotor->direction    = TMC2209_DIR_CCW;
    hmotor->state        = TMC2209_STOPPED;
    hmotor->current_steps = 0;
    hmotor->current_angle = 0.0f;

    /* Cho driver on dinh sau khi cap nguon hoac reset. */
    HAL_Delay(50);

    /*
     * GCONF:
     *   pdn_disable      = 1: Tat PDN, dung UART.
     *   mstep_reg_select = 1: Chon microstep qua register.
     *   multistep_filt   = 1: Bat bo loc nhieu STEP.
     */
    hmotor->reg_gconf = (1U << 6) | (1U << 7) | (1U << 8);
    ret = TMC2209_WriteReg(hmotor, TMC2209_REG_GCONF, hmotor->reg_gconf);
    if (ret != TMC2209_OK) return ret;

    /*
     * CHOPCONF:
     *   MRES  [27:24] = microstep.
     *   tbl   [16:15] = 01, blank time 24 clock.
     *   hstrt [6:4]   = 4.
     *   toff  [3:0]   = 3, khac 0 de bat driver.
     */
    uint8_t mres = _TMC2209_MicrostepToMRES(hmotor->microstep);
    hmotor->reg_chopconf = ((uint32_t)mres << 24)
                         | (0x01U << 15)    /* tbl=01 */
                         | (4U    <<  4)    /* hstrt=4 */
                         | (3U         );   /* toff=3  */
    ret = TMC2209_WriteReg(hmotor, TMC2209_REG_CHOPCONF, hmotor->reg_chopconf);
    if (ret != TMC2209_OK) return ret;

    /*
     * IHOLD_IRUN mac dinh:
     *   IHOLD      = 8.
     *   IRUN       = 20.
     *   IHOLDDELAY = 6.
     */
    hmotor->reg_ihold_irun = (6U << 16) | (20U << 8) | (8U);
    ret = TMC2209_WriteReg(hmotor, TMC2209_REG_IHOLD_IRUN, hmotor->reg_ihold_irun);
    if (ret != TMC2209_OK) return ret;

    /* TPOWERDOWN quy dinh thoi gian cho truoc khi driver giam dong. */
    ret = TMC2209_WriteReg(hmotor, TMC2209_REG_TPOWERDOWN, 20U);
    if (ret != TMC2209_OK) return ret;

    /* Dat tan so STEP mac dinh de timer co cau hinh hop le truoc khi Start. */
    _TMC2209_SetPWMFreq(hmotor, 100U);
    hmotor->speed_hz = 100U;

    return TMC2209_OK;
}

/**
 * @brief  Cau hinh dong chay, dong giu va do tre giam dong.
 * @param[in,out] hmotor: Con tro den handle TMC2209.
 * @param[in] run_current: Dong chay, gia tri 0 den 31.
 * @param[in] hold_current: Dong giu, gia tri 0 den 31.
 * @param[in] hold_delay: Do tre giam dong, gia tri 0 den 15.
 * @return Trang thai ghi register IHOLD_IRUN.
 */
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

/**
 * @brief  Cau hinh so microstep trong CHOPCONF.
 * @param[in,out] hmotor: Con tro den handle TMC2209.
 * @param[in] microstep: So microstep can cau hinh.
 * @return Trang thai ghi register CHOPCONF.
 */
TMC2209_StatusTypeDef TMC2209_SetMicrostep(TMC2209_HandleTypeDef *hmotor,
                                           TMC2209_MicrostepTypeDef microstep)
{
    hmotor->microstep = microstep;
    uint8_t mres = _TMC2209_MicrostepToMRES(microstep);

    /* Chi cap nhat MRES de giu nguyen cac bit chopper khac trong cache. */
    hmotor->reg_chopconf &= ~(0x0FUL << 24);                   /* Xoa MRES cu. */
    hmotor->reg_chopconf |=  ((uint32_t)mres << 24);           /* Ghi MRES moi. */

    return TMC2209_WriteReg(hmotor, TMC2209_REG_CHOPCONF, hmotor->reg_chopconf);
}

/**
 * @brief  Chon che do StealthChop hoac SpreadCycle.
 * @param[in,out] hmotor: Con tro den handle TMC2209.
 * @param[in] enable: true de dung StealthChop, false de dung SpreadCycle.
 * @return Trang thai ghi register GCONF.
 */
TMC2209_StatusTypeDef TMC2209_SetStealthChop(TMC2209_HandleTypeDef *hmotor, bool enable)
{
    if (enable)
        hmotor->reg_gconf &= ~(1U << 2); /* en_spreadCycle=0 -> StealthChop. */
    else
        hmotor->reg_gconf |=  (1U << 2); /* en_spreadCycle=1 -> SpreadCycle. */

    return TMC2209_WriteReg(hmotor, TMC2209_REG_GCONF, hmotor->reg_gconf);
}

/* Public functions: motor GPIO and speed ---------------------------------- */

/**
 * @brief  Bat driver TMC2209 bang chan EN active-low.
 * @param[in,out] hmotor: Con tro den handle TMC2209.
 */
void TMC2209_Enable(TMC2209_HandleTypeDef *hmotor)
{
    HAL_GPIO_WritePin(hmotor->en_port, hmotor->en_pin, GPIO_PIN_RESET); /* EN=LOW. */
    hmotor->enable_state = TMC2209_ENABLE;
}

/**
 * @brief  Tat driver TMC2209 bang chan EN active-low.
 * @param[in,out] hmotor: Con tro den handle TMC2209.
 */
void TMC2209_Disable(TMC2209_HandleTypeDef *hmotor)
{
    HAL_GPIO_WritePin(hmotor->en_port, hmotor->en_pin, GPIO_PIN_SET);   /* EN=HIGH. */
    hmotor->enable_state = TMC2209_DISABLE;
}

/**
 * @brief  Dat chieu quay bang chan DIR.
 * @param[in,out] hmotor: Con tro den handle TMC2209.
 * @param[in] dir: Chieu quay can dat.
 */
void TMC2209_SetDirection(TMC2209_HandleTypeDef *hmotor,
                          TMC2209_DirectionTypeDef dir)
{
    hmotor->direction = dir;
    HAL_GPIO_WritePin(hmotor->dir_port, hmotor->dir_pin,
                      (dir == TMC2209_DIR_CW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  Dat toc do bang tan so xung STEP.
 * @param[in,out] hmotor: Con tro den handle TMC2209.
 * @param[in] freq_hz: Tan so STEP tinh bang Hz.
 * @return TMC2209_OK neu timer duoc cap nhat thanh cong.
 */
TMC2209_StatusTypeDef TMC2209_SetSpeedHz(TMC2209_HandleTypeDef *hmotor,
                                         uint32_t freq_hz)
{
    return _TMC2209_SetPWMFreq(hmotor, freq_hz);
}

/**
 * @brief  Dat toc do bang RPM.
 * @param[in,out] hmotor: Con tro den handle TMC2209.
 * @param[in] rpm: Toc do mong muon, tinh bang vong/phut.
 * @return TMC2209_OK neu timer duoc cap nhat thanh cong.
 */
TMC2209_StatusTypeDef TMC2209_SetSpeedRPM(TMC2209_HandleTypeDef *hmotor,
                                          float rpm)
{
    if (rpm <= 0.0f) return TMC2209_ERROR;
    uint32_t hz = TMC2209_RPM_to_Hz(hmotor, rpm);
    if (hz == 0U) hz = 1U;
    return _TMC2209_SetPWMFreq(hmotor, hz);
}

/* Public functions: motion control ---------------------------------------- */

/**
 * @brief  Bat PWM STEP de motor bat dau chay.
 * @param[in,out] hmotor: Con tro den handle TMC2209.
 * @return TMC2209_OK neu timer PWM start thanh cong.
 */
TMC2209_StatusTypeDef TMC2209_Start(TMC2209_HandleTypeDef *hmotor)
{
    if (hmotor->enable_state == TMC2209_DISABLE)
        TMC2209_Enable(hmotor);

    HAL_StatusTypeDef ret = HAL_TIM_PWM_Start_IT(hmotor->htim, hmotor->tim_channel);
    if (ret != HAL_OK) return TMC2209_ERROR;

    hmotor->state = TMC2209_RUNNING;
    return TMC2209_OK;
}

/**
 * @brief  Dung PWM STEP va dua trang thai motor ve STOPPED.
 * @param[in,out] hmotor: Con tro den handle TMC2209.
 */
void TMC2209_Stop(TMC2209_HandleTypeDef *hmotor)
{
    (void)HAL_TIM_PWM_Stop_IT(hmotor->htim, hmotor->tim_channel);
    hmotor->state = TMC2209_STOPPED;
}

/**
 * @brief  Chay motor them mot so buoc theo chieu va toc do chi dinh.
 * @param[in,out] hmotor: Con tro den handle TMC2209.
 * @param[in] steps: So xung STEP can phat.
 * @param[in] dir: Chieu quay.
 * @param[in] speed_rpm: Toc do chay, tinh bang RPM.
 * @return TMC2209_OK neu lenh chay duoc start thanh cong.
 */
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

/**
 * @brief  Chay motor den goc tuyet doi trong cache handle.
 * @param[in,out] hmotor: Con tro den handle TMC2209.
 * @param[in] angle_deg: Goc muc tieu, tinh bang do.
 * @param[in] speed_rpm: Toc do chay, tinh bang RPM.
 * @return TMC2209_OK neu lenh chay duoc chap nhan.
 */
TMC2209_StatusTypeDef TMC2209_MoveToAngle(TMC2209_HandleTypeDef *hmotor,
                                          float angle_deg,
                                          float speed_rpm)
{
    /* Lay delta de suy ra chieu quay ngan theo cache hien tai. */
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

    /* Doi delta goc sang so microstep va lam tron ve buoc gan nhat. */
    float steps_f = (delta / 360.0f) * (float)hmotor->steps_per_rev * (float)hmotor->microstep;
    uint32_t steps = (uint32_t)(steps_f + 0.5f);

    if (steps == 0U) return TMC2209_OK;  /* Da o dung vi tri theo do phan giai buoc. */

    hmotor->target_angle = angle_deg;

    return TMC2209_MoveSteps(hmotor, steps, dir, speed_rpm);
}

/**
 * @brief  Cap nhat bo dem buoc, goi tu callback ngat timer.
 * @param[in,out] hmotor: Con tro den handle TMC2209.
 * @return true neu motor vua hoan tat du so buoc muc tieu.
 *
 * @note Ham nay nen duoc goi tu HAL_TIM_PWM_PulseFinishedCallback() hoac
 *       callback timer tuong duong cho kenh STEP cua motor.
 */
bool TMC2209_UpdateSteps(TMC2209_HandleTypeDef *hmotor)
{
    if (hmotor->state != TMC2209_RUNNING) return false;
    if (hmotor->target_steps == 0U)       return false; /* Che do chay lien tuc. */

    hmotor->current_steps++;

    if (hmotor->current_steps >= hmotor->target_steps)
    {
        TMC2209_Stop(hmotor);

        /* Cap nhat cache goc dua tren so buoc vua phat ra. */
        float delta_deg = ((float)hmotor->target_steps /
                           ((float)hmotor->steps_per_rev * (float)hmotor->microstep)) * 360.0f;

        if (hmotor->direction == TMC2209_DIR_CW)
            hmotor->current_angle += delta_deg;
        else
            hmotor->current_angle -= delta_deg;

        /* Giu cache goc trong mien [0, 360). */
        while (hmotor->current_angle <   0.0f)   hmotor->current_angle += 360.0f;
        while (hmotor->current_angle >= 360.0f)  hmotor->current_angle -= 360.0f;

        hmotor->target_steps  = 0U;
        hmotor->current_steps = 0U;
        return true;
    }
    return false;
}

/* Public functions: driver status ----------------------------------------- */

/**
 * @brief  Doc register DRV_STATUS cua TMC2209.
 * @param[in] hmotor: Con tro den handle TMC2209.
 * @param[out] status_out: Noi luu gia tri DRV_STATUS.
 * @return Trang thai doc register.
 */
TMC2209_StatusTypeDef TMC2209_ReadDrvStatus(TMC2209_HandleTypeDef *hmotor,
                                            uint32_t *status_out)
{
    return TMC2209_ReadReg(hmotor, TMC2209_REG_DRV_STATUS, status_out);
}

/**
 * @brief  Doc gia tri StallGuard SG_RESULT.
 * @param[in] hmotor: Con tro den handle TMC2209.
 * @param[out] sg_out: Noi luu SG_RESULT 9-bit.
 * @return Trang thai doc register.
 */
TMC2209_StatusTypeDef TMC2209_ReadSGResult(TMC2209_HandleTypeDef *hmotor,
                                           uint16_t *sg_out)
{
    uint32_t val = 0;
    TMC2209_StatusTypeDef ret = TMC2209_ReadReg(hmotor, TMC2209_REG_SG_RESULT, &val);
    if (ret == TMC2209_OK)
        *sg_out = (uint16_t)(val & 0x1FFU);  /* SG_RESULT nam o bit [8:0]. */
    return ret;
}

/**
 * @brief  Kiem tra stall bang cach so sanh SG_RESULT voi nguong.
 * @param[in] hmotor: Con tro den handle TMC2209.
 * @param[in] threshold: Nguong SG_RESULT, gia tri cang lon cang nhay.
 * @return true neu SG_RESULT nho hon nguong.
 */
bool TMC2209_IsStalled(TMC2209_HandleTypeDef *hmotor, uint8_t threshold)
{
    uint16_t sg = 0;
    if (TMC2209_ReadSGResult(hmotor, &sg) != TMC2209_OK) return false;
    return (sg < (uint16_t)threshold);
}

/**
 * @brief  Kiem tra cac co loi nghiem trong trong DRV_STATUS.
 * @param[in] hmotor: Con tro den handle TMC2209.
 * @return true neu driver bao overtemperature hoac short to GND.
 */
bool TMC2209_HasFault(TMC2209_HandleTypeDef *hmotor)
{
    uint32_t status = 0;
    if (TMC2209_ReadDrvStatus(hmotor, &status) != TMC2209_OK) return false;

    /*
     * DRV_STATUS:
     *   bit 29 = ot, overtemperature shutdown.
     *   bit 25 = s2gb, short to GND phase B.
     *   bit 24 = s2ga, short to GND phase A.
     */
    uint32_t fault_mask = (1U << 29) | (1U << 25) | (1U << 24);
    return ((status & fault_mask) != 0U);
}
