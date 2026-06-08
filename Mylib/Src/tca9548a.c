/**
 * @file    tca9548a.c
 * @brief   Trien khai driver TCA9548A va quan ly AS5600 tren cac kenh mux.
 * @author  Lap4all
 * @date    2026-05-21
 */

#include "tca9548a.h"
#include <string.h>     /* memset, strncpy */

/* Private functions -------------------------------------------------------- */

/* Kiem tra handle va cac callback I2C bat buoc truoc khi truy cap mux. */
static TCA9548A_Status_t _check_hal(TCA9548A_Handle_t *mux)
{
    if (mux == NULL)                return TCA9548A_ERR_NULL_PTR;
    if (mux->i2c_write == NULL)     return TCA9548A_ERR_NULL_PTR;
    if (mux->i2c_read  == NULL)     return TCA9548A_ERR_NULL_PTR;
    return TCA9548A_OK;
}

/*
 * TCA9548A chi co mot thanh ghi control 1 byte.
 * Moi bit trong mask tuong ung voi mot kenh mux.
 */
static TCA9548A_Status_t _write_control(TCA9548A_Handle_t *mux, uint8_t mask)
{
    int8_t ret = mux->i2c_write(mux->i2c_addr, TCA9548A_CTRL_REG, &mask, 1);
    if (ret != 0) return TCA9548A_ERR_I2C;
    mux->active_mask = mask;
    return TCA9548A_OK;
}

/* Doc bitmask kenh hien tai tu thanh ghi control cua TCA9548A. */
static TCA9548A_Status_t _read_control(TCA9548A_Handle_t *mux, uint8_t *mask)
{
    int8_t ret = mux->i2c_read(mux->i2c_addr, TCA9548A_CTRL_REG, mask, 1);
    return (ret == 0) ? TCA9548A_OK : TCA9548A_ERR_I2C;
}

/* Tim slot sensor da dang ky tren kenh chi dinh. */
static TCA9548A_SensorSlot_t *_find_slot(TCA9548A_Handle_t *mux, TCA9548A_Channel_t ch)
{
    for (uint8_t i = 0; i < TCA9548A_MAX_CHANNELS; i++) {
        if (mux->sensors[i].enabled && mux->sensors[i].channel == ch) {
            return &mux->sensors[i];
        }
    }
    return NULL;
}

/* Tim slot chua dang ky sensor. */
static TCA9548A_SensorSlot_t *_find_empty_slot(TCA9548A_Handle_t *mux)
{
    for (uint8_t i = 0; i < TCA9548A_MAX_CHANNELS; i++) {
        if (!mux->sensors[i].enabled) {
            return &mux->sensors[i];
        }
    }
    return NULL;
}

/* Public functions: mux control ------------------------------------------- */

/**
 * @brief  Khoi tao TCA9548A va tat tat ca kenh.
 * @param[in,out] mux: Con tro den handle TCA9548A.
 * @param[in] addr: Dia chi I2C cua TCA9548A.
 * @return TCA9548A_OK neu khoi tao thanh cong, nguoc lai tra ve ma loi.
 */
TCA9548A_Status_t TCA9548A_Init(TCA9548A_Handle_t *mux, TCA9548A_I2CAddr_t addr)
{
    TCA9548A_Status_t ret;

    ret = _check_hal(mux);
    if (ret != TCA9548A_OK) return ret;

    /* Khoi tao trang thai handle truoc khi cham vao phan cung. */
    mux->i2c_addr      = (uint8_t)addr;
    mux->mode          = TCA9548A_MODE_SINGLE;
    mux->active_mask   = 0x00;
    mux->active_channel= TCA9548A_CH_NONE;
    mux->sensor_count  = 0;

    /* Xoa cac slot sensor de tranh dung lai du lieu cu trong RAM. */
    memset(mux->sensors, 0, sizeof(mux->sensors));

    /* Doc control register de xac nhan mux phan hoi tren bus I2C. */
    ret = TCA9548A_IsConnected(mux);
    if (ret != TCA9548A_OK) return ret;

    return TCA9548A_Reset(mux);
}

/**
 * @brief  Reset TCA9548A ve trang thai tat tat ca kenh.
 * @param[in,out] mux: Con tro den handle TCA9548A.
 * @return TCA9548A_OK neu reset thanh cong, nguoc lai tra ve ma loi.
 */
TCA9548A_Status_t TCA9548A_Reset(TCA9548A_Handle_t *mux)
{
    TCA9548A_Status_t ret;

    ret = _check_hal(mux);
    if (ret != TCA9548A_OK) return ret;

    ret = _write_control(mux, TCA9548A_ALL_CHANNELS_OFF);
    if (ret == TCA9548A_OK) {
        mux->active_channel = TCA9548A_CH_NONE;
    }
    return ret;
}

/**
 * @brief  Chon mot kenh duy nhat tren TCA9548A.
 * @param[in,out] mux: Con tro den handle TCA9548A.
 * @param[in] ch: Kenh can chon.
 * @return TCA9548A_OK neu chon kenh thanh cong, nguoc lai tra ve ma loi.
 */
TCA9548A_Status_t TCA9548A_SelectChannel(TCA9548A_Handle_t *mux, TCA9548A_Channel_t ch)
{
    TCA9548A_Status_t ret;

    ret = _check_hal(mux);
    if (ret != TCA9548A_OK) return ret;

    if (ch >= TCA9548A_MAX_CHANNELS) return TCA9548A_ERR_INVALID_CH;

    /* Single-channel mode chi bat bit cua kenh duoc chon. */
    uint8_t mask = (uint8_t)(1 << ch);
    ret = _write_control(mux, mask);
    if (ret == TCA9548A_OK) {
        mux->active_channel = ch;
    }
    return ret;
}

/**
 * @brief  Ghi bitmask kenh truc tiep vao TCA9548A.
 * @param[in,out] mux: Con tro den handle TCA9548A.
 * @param[in] mask: Bitmask kenh, bit0 tuong ung CH0.
 * @return TCA9548A_OK neu ghi thanh cong, nguoc lai tra ve ma loi.
 */
TCA9548A_Status_t TCA9548A_SetChannelMask(TCA9548A_Handle_t *mux, uint8_t mask)
{
    TCA9548A_Status_t ret;

    ret = _check_hal(mux);
    if (ret != TCA9548A_OK) return ret;

    ret = _write_control(mux, mask);
    if (ret == TCA9548A_OK) {
        /* active_channel luu kenh thap nhat dang bat de code cu van doc duoc. */
        mux->active_channel = TCA9548A_CH_NONE;
        for (uint8_t i = 0; i < TCA9548A_MAX_CHANNELS; i++) {
            if (mask & (1 << i)) {
                mux->active_channel = (TCA9548A_Channel_t)i;
                break;
            }
        }
    }
    return ret;
}

/**
 * @brief  Tat tat ca kenh tren TCA9548A.
 * @param[in,out] mux: Con tro den handle TCA9548A.
 * @return TCA9548A_OK neu tat kenh thanh cong, nguoc lai tra ve ma loi.
 */
TCA9548A_Status_t TCA9548A_DisableAll(TCA9548A_Handle_t *mux)
{
    TCA9548A_Status_t ret;

    ret = _check_hal(mux);
    if (ret != TCA9548A_OK) return ret;

    ret = _write_control(mux, TCA9548A_ALL_CHANNELS_OFF);
    if (ret == TCA9548A_OK) {
        mux->active_channel = TCA9548A_CH_NONE;
    }
    return ret;
}

/**
 * @brief  Doc bitmask kenh hien tai tu TCA9548A.
 * @param[in,out] mux: Con tro den handle TCA9548A.
 * @param[out] mask: Noi luu bitmask kenh doc duoc.
 * @return TCA9548A_OK neu doc thanh cong, nguoc lai tra ve ma loi.
 */
TCA9548A_Status_t TCA9548A_GetChannelMask(TCA9548A_Handle_t *mux, uint8_t *mask)
{
    TCA9548A_Status_t ret;

    if (mux == NULL || mask == NULL) return TCA9548A_ERR_NULL_PTR;

    ret = _read_control(mux, mask);
    if (ret == TCA9548A_OK) {
        mux->active_mask = *mask;
    }
    return ret;
}

/**
 * @brief  Kiem tra trang thai bat/tat cua mot kenh trong cache handle.
 * @param[in] mux: Con tro den handle TCA9548A.
 * @param[in] ch: Kenh can kiem tra.
 * @param[out] state: Noi luu trang thai kenh.
 * @return TCA9548A_OK neu tham so hop le, nguoc lai tra ve ma loi.
 */
TCA9548A_Status_t TCA9548A_GetChannelState(TCA9548A_Handle_t *mux,
                                             TCA9548A_Channel_t ch,
                                             TCA9548A_ChannelState_t *state)
{
    if (mux == NULL || state == NULL) return TCA9548A_ERR_NULL_PTR;
    if (ch >= TCA9548A_MAX_CHANNELS)  return TCA9548A_ERR_INVALID_CH;

    *state = (mux->active_mask & (1 << ch)) ?
             TCA9548A_CH_ENABLED : TCA9548A_CH_DISABLED;
    return TCA9548A_OK;
}

/**
 * @brief  Kiem tra TCA9548A co phan hoi tren bus I2C hay khong.
 * @param[in] mux: Con tro den handle TCA9548A.
 * @return TCA9548A_OK neu doc duoc control register, nguoc lai tra ve ma loi.
 */
TCA9548A_Status_t TCA9548A_IsConnected(TCA9548A_Handle_t *mux)
{
    uint8_t dummy;
    TCA9548A_Status_t ret;

    ret = _check_hal(mux);
    if (ret != TCA9548A_OK) return ret;

    ret = _read_control(mux, &dummy);
    return ret;
}

/* Public functions: AS5600 management ------------------------------------- */

/**
 * @brief  Dang ky mot AS5600 tren kenh mux chi dinh.
 * @param[in,out] mux: Con tro den handle TCA9548A.
 * @param[in] ch: Kenh mux noi voi AS5600.
 * @param[in] label: Nhan tuy chon, co the NULL.
 * @return TCA9548A_OK neu dang ky thanh cong, nguoc lai tra ve ma loi.
 */
TCA9548A_Status_t TCA9548A_RegisterSensor(TCA9548A_Handle_t *mux,
                                            TCA9548A_Channel_t ch,
                                            const char *label)
{
    if (mux == NULL) return TCA9548A_ERR_NULL_PTR;
    if (ch >= TCA9548A_MAX_CHANNELS) return TCA9548A_ERR_INVALID_CH;

    /* Dang ky lap lai cung kenh duoc xem la thanh cong de ham co tinh idempotent. */
    if (_find_slot(mux, ch) != NULL) {
        return TCA9548A_OK;
    }

    TCA9548A_SensorSlot_t *slot = _find_empty_slot(mux);
    if (slot == NULL) return TCA9548A_ERR_CHANNEL_BUSY;

    /* Xoa slot truoc khi gan de label va handle AS5600 co gia tri nen ro rang. */
    memset(slot, 0, sizeof(TCA9548A_SensorSlot_t));
    slot->channel = ch;
    slot->enabled = true;

    if (label != NULL) {
        strncpy(slot->label, label, sizeof(slot->label) - 1);
        slot->label[sizeof(slot->label) - 1] = '\0';
    }

    /* AS5600 dung chung callback I2C voi mux, kenh duoc chon truoc moi lan doc. */
    slot->sensor.i2c_addr  = AS5600_I2C_ADDR;
    slot->sensor.i2c_write = mux->i2c_write;
    slot->sensor.i2c_read  = mux->i2c_read;
    slot->sensor.delay_ms  = mux->delay_ms;

    mux->sensor_count++;
    return TCA9548A_OK;
}

/**
 * @brief  Khoi tao tat ca AS5600 da dang ky tren cac kenh mux.
 * @param[in,out] mux: Con tro den handle TCA9548A.
 * @return TCA9548A_OK neu tat ca sensor khoi tao thanh cong.
 */
TCA9548A_Status_t TCA9548A_InitAllSensors(TCA9548A_Handle_t *mux)
{
    TCA9548A_Status_t ret;
    AS5600_Status_t   as_ret;

    if (mux == NULL) return TCA9548A_ERR_NULL_PTR;

    for (uint8_t i = 0; i < TCA9548A_MAX_CHANNELS; i++) {
        TCA9548A_SensorSlot_t *slot = &mux->sensors[i];
        if (!slot->enabled) continue;

        /* Chon dung nhanh I2C truoc khi goi driver AS5600. */
        ret = TCA9548A_SelectChannel(mux, slot->channel);
        if (ret != TCA9548A_OK) return ret;

        /* Delay ngan giup kenh mux on dinh sau khi doi control register. */
        if (mux->delay_ms) mux->delay_ms(2);

        as_ret = AS5600_Init(&slot->sensor);
        if (as_ret != AS5600_OK) {
            return TCA9548A_ERR_SENSOR_FAIL;
        }
    }

    /* Dua mux ve trang thai tat kenh sau khi hoan tat init. */
    return TCA9548A_DisableAll(mux);
}

/**
 * @brief  Doc toan bo du lieu tu AS5600 tren mot kenh mux.
 * @param[in,out] mux: Con tro den handle TCA9548A.
 * @param[in] ch: Kenh can doc.
 * @param[out] data: Noi luu du lieu AS5600.
 * @return TCA9548A_OK neu doc thanh cong, nguoc lai tra ve ma loi.
 */
TCA9548A_Status_t TCA9548A_ReadSensor(TCA9548A_Handle_t *mux,
                                        TCA9548A_Channel_t ch,
                                        AS5600_Data_t *data)
{
    TCA9548A_Status_t ret;
    AS5600_Status_t   as_ret;

    if (mux == NULL || data == NULL) return TCA9548A_ERR_NULL_PTR;
    if (ch >= TCA9548A_MAX_CHANNELS) return TCA9548A_ERR_INVALID_CH;

    TCA9548A_SensorSlot_t *slot = _find_slot(mux, ch);
    if (slot == NULL) return TCA9548A_ERR_NOT_INIT;

    ret = TCA9548A_SelectChannel(mux, ch);
    if (ret != TCA9548A_OK) return ret;

    /* Delay ngan giup AS5600 thay duong I2C da duoc noi qua mux. */
    if (mux->delay_ms) mux->delay_ms(1);

    as_ret = AS5600_ReadAll(&slot->sensor, data);
    if (as_ret != AS5600_OK) {
        TCA9548A_DisableAll(mux);
        return TCA9548A_ERR_SENSOR_FAIL;
    }

    /* Mode SINGLE tranh de mot nhanh I2C van mo sau khi doc xong. */
    if (mux->mode == TCA9548A_MODE_SINGLE) {
        TCA9548A_DisableAll(mux);
    }

    return TCA9548A_OK;
}

/**
 * @brief  Doc goc theo do tu AS5600 tren mot kenh mux.
 * @param[in,out] mux: Con tro den handle TCA9548A.
 * @param[in] ch: Kenh can doc.
 * @param[out] degrees: Noi luu goc theo do.
 * @return TCA9548A_OK neu doc thanh cong, nguoc lai tra ve ma loi.
 */
#if (MY_APP_AS5600_FLOAT_UNITS == MY_APP_MODULE_ENABLED)
TCA9548A_Status_t TCA9548A_ReadAngleDeg(TCA9548A_Handle_t *mux,
                                          TCA9548A_Channel_t ch,
                                          float *degrees)
{
    TCA9548A_Status_t ret;
    AS5600_Status_t   as_ret;

    if (mux == NULL || degrees == NULL) return TCA9548A_ERR_NULL_PTR;
    if (ch >= TCA9548A_MAX_CHANNELS)    return TCA9548A_ERR_INVALID_CH;

    TCA9548A_SensorSlot_t *slot = _find_slot(mux, ch);
    if (slot == NULL) return TCA9548A_ERR_NOT_INIT;

    ret = TCA9548A_SelectChannel(mux, ch);
    if (ret != TCA9548A_OK) return ret;

    if (mux->delay_ms) mux->delay_ms(1);

    as_ret = AS5600_ReadAngleDeg(&slot->sensor, degrees);

    TCA9548A_DisableAll(mux);

    return (as_ret == AS5600_OK) ? TCA9548A_OK : TCA9548A_ERR_SENSOR_FAIL;
}
#elif (MY_APP_AS5600_FLOAT_UNITS == MY_APP_MODULE_DISABLED)
#else
#error "Invalid MY_APP_AS5600_FLOAT_UNITS setting"
#endif

/**
 * @brief  Quet va doc tat ca AS5600 da dang ky.
 * @param[in,out] mux: Con tro den handle TCA9548A.
 * @param[out] result: Noi luu ket qua doc tung sensor.
 * @return TCA9548A_OK khi hoan tat qua trinh quet.
 */
TCA9548A_Status_t TCA9548A_ScanAllSensors(TCA9548A_Handle_t *mux,
                                            TCA9548A_ScanResult_t *result)
{
    TCA9548A_Status_t ret;

    if (mux == NULL || result == NULL) return TCA9548A_ERR_NULL_PTR;

    memset(result, 0, sizeof(TCA9548A_ScanResult_t));
    result->count = 0;

    for (uint8_t i = 0; i < TCA9548A_MAX_CHANNELS; i++) {
        TCA9548A_SensorSlot_t *slot = &mux->sensors[i];
        if (!slot->enabled) continue;

        uint8_t idx = result->count;
        result->channels[idx] = slot->channel;

        ret = TCA9548A_SelectChannel(mux, slot->channel);
        if (ret != TCA9548A_OK) {
            result->status[idx] = ret;
            result->count++;
            continue;
        }

        if (mux->delay_ms) mux->delay_ms(1);

        AS5600_Status_t as_ret = AS5600_ReadAll(&slot->sensor, &result->data[idx]);
        result->status[idx] = (as_ret == AS5600_OK) ?
                               TCA9548A_OK : TCA9548A_ERR_SENSOR_FAIL;

        result->count++;
    }

    /* Tat tat ca kenh de lan doc tiep theo bat dau tu trang thai xac dinh. */
    TCA9548A_DisableAll(mux);

    return TCA9548A_OK;
}

/**
 * @brief  Lay handle AS5600 da gan voi mot kenh mux.
 * @param[in] mux: Con tro den handle TCA9548A.
 * @param[in] ch: Kenh can lay handle AS5600.
 * @return Con tro AS5600_Handle_t, hoac NULL neu kenh khong hop le/chua dang ky.
 */
AS5600_Handle_t *TCA9548A_GetSensorHandle(TCA9548A_Handle_t *mux,
                                            TCA9548A_Channel_t ch)
{
    if (mux == NULL || ch >= TCA9548A_MAX_CHANNELS) return NULL;

    TCA9548A_SensorSlot_t *slot = _find_slot(mux, ch);
    if (slot == NULL) return NULL;

    return &slot->sensor;
}
