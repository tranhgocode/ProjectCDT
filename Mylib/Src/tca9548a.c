/*
 * tca9548a.c
 *
 *  Created on: May 21, 2026
 *      Author: Lap4all
 */


#include "tca9548a.h"
#include <string.h>     /* memset, strncpy */
 
/* ============================================================
 *  HÀM NỘI BỘ (PRIVATE)
 * ============================================================ */
 
/**
 * @brief Kiểm tra tính hợp lệ của handle và HAL
 */
static TCA9548A_Status_t _check_hal(TCA9548A_Handle_t *mux)
{
    if (mux == NULL)                return TCA9548A_ERR_NULL_PTR;
    if (mux->i2c_write == NULL)     return TCA9548A_ERR_NULL_PTR;
    if (mux->i2c_read  == NULL)     return TCA9548A_ERR_NULL_PTR;
    return TCA9548A_OK;
}
 
/**
 * @brief Ghi bitmask kênh vào thanh ghi điều khiển TCA9548A
 * @note  TCA9548A chỉ có 1 thanh ghi điều khiển — ghi trực tiếp 1 byte
 */
static TCA9548A_Status_t _write_control(TCA9548A_Handle_t *mux, uint8_t mask)
{
    int8_t ret = mux->i2c_write(mux->i2c_addr, TCA9548A_CTRL_REG, &mask, 1);
    if (ret != 0) return TCA9548A_ERR_I2C;
    mux->active_mask = mask;
    return TCA9548A_OK;
}
 
/**
 * @brief Đọc bitmask kênh từ thanh ghi điều khiển TCA9548A
 */
static TCA9548A_Status_t _read_control(TCA9548A_Handle_t *mux, uint8_t *mask)
{
    int8_t ret = mux->i2c_read(mux->i2c_addr, TCA9548A_CTRL_REG, mask, 1);
    return (ret == 0) ? TCA9548A_OK : TCA9548A_ERR_I2C;
}
 
/**
 * @brief Tìm slot sensor theo kênh
 * @return Con trỏ đến slot hoặc NULL nếu không tìm thấy
 */
static TCA9548A_SensorSlot_t *_find_slot(TCA9548A_Handle_t *mux, TCA9548A_Channel_t ch)
{
    for (uint8_t i = 0; i < TCA9548A_MAX_CHANNELS; i++) {
        if (mux->sensors[i].enabled && mux->sensors[i].channel == ch) {
            return &mux->sensors[i];
        }
    }
    return NULL;
}
 
/**
 * @brief Tìm slot trống (chưa được đăng ký)
 * @return Con trỏ đến slot trống hoặc NULL nếu đầy
 */
static TCA9548A_SensorSlot_t *_find_empty_slot(TCA9548A_Handle_t *mux)
{
    for (uint8_t i = 0; i < TCA9548A_MAX_CHANNELS; i++) {
        if (!mux->sensors[i].enabled) {
            return &mux->sensors[i];
        }
    }
    return NULL;
}
 
/* ============================================================
 *  TRIỂN KHAI HÀM PUBLIC — QUẢN LÝ MUX
 * ============================================================ */
 
TCA9548A_Status_t TCA9548A_Init(TCA9548A_Handle_t *mux, TCA9548A_I2CAddr_t addr)
{
    TCA9548A_Status_t ret;
 
    ret = _check_hal(mux);
    if (ret != TCA9548A_OK) return ret;
 
    /* Khởi tạo các trường trong handle */
    mux->i2c_addr      = (uint8_t)addr;
    mux->mode          = TCA9548A_MODE_SINGLE;
    mux->active_mask   = 0x00;
    mux->active_channel= TCA9548A_CH_NONE;
    mux->sensor_count  = 0;
 
    /* Xóa toàn bộ slot sensor */
    memset(mux->sensors, 0, sizeof(mux->sensors));
 
    /* Kiểm tra kết nối và tắt tất cả kênh */
    ret = TCA9548A_IsConnected(mux);
    if (ret != TCA9548A_OK) return ret;
 
    return TCA9548A_Reset(mux);
}
 
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
 
TCA9548A_Status_t TCA9548A_SelectChannel(TCA9548A_Handle_t *mux, TCA9548A_Channel_t ch)
{
    TCA9548A_Status_t ret;
 
    ret = _check_hal(mux);
    if (ret != TCA9548A_OK) return ret;
 
    if (ch >= TCA9548A_MAX_CHANNELS) return TCA9548A_ERR_INVALID_CH;
 
    /* Chỉ bật 1 kênh duy nhất */
    uint8_t mask = (uint8_t)(1 << ch);
    ret = _write_control(mux, mask);
    if (ret == TCA9548A_OK) {
        mux->active_channel = ch;
    }
    return ret;
}
 
TCA9548A_Status_t TCA9548A_SetChannelMask(TCA9548A_Handle_t *mux, uint8_t mask)
{
    TCA9548A_Status_t ret;
 
    ret = _check_hal(mux);
    if (ret != TCA9548A_OK) return ret;
 
    ret = _write_control(mux, mask);
    if (ret == TCA9548A_OK) {
        /* Cập nhật active_channel (kênh thấp nhất đang bật) */
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
 
TCA9548A_Status_t TCA9548A_IsConnected(TCA9548A_Handle_t *mux)
{
    uint8_t dummy;
    TCA9548A_Status_t ret;
 
    ret = _check_hal(mux);
    if (ret != TCA9548A_OK) return ret;
 
    ret = _read_control(mux, &dummy);
    return ret;
}
 
/* ============================================================
 *  TRIỂN KHAI HÀM PUBLIC — QUẢN LÝ AS5600 TRÊN MUX
 * ============================================================ */
 
TCA9548A_Status_t TCA9548A_RegisterSensor(TCA9548A_Handle_t *mux,
                                            TCA9548A_Channel_t ch,
                                            const char *label)
{
    if (mux == NULL) return TCA9548A_ERR_NULL_PTR;
    if (ch >= TCA9548A_MAX_CHANNELS) return TCA9548A_ERR_INVALID_CH;
 
    /* Kiểm tra xem kênh đã được đăng ký chưa */
    if (_find_slot(mux, ch) != NULL) {
        /* Kênh đã có sensor, coi như thành công */
        return TCA9548A_OK;
    }
 
    /* Tìm slot trống */
    TCA9548A_SensorSlot_t *slot = _find_empty_slot(mux);
    if (slot == NULL) return TCA9548A_ERR_CHANNEL_BUSY;
 
    /* Điền thông tin sensor */
    memset(slot, 0, sizeof(TCA9548A_SensorSlot_t));
    slot->channel = ch;
    slot->enabled = true;
 
    if (label != NULL) {
        strncpy(slot->label, label, sizeof(slot->label) - 1);
        slot->label[sizeof(slot->label) - 1] = '\0';
    }
 
    /* Cài đặt HAL cho AS5600 — dùng chung HAL của MUX */
    slot->sensor.i2c_addr  = AS5600_I2C_ADDR;
    slot->sensor.i2c_write = mux->i2c_write;
    slot->sensor.i2c_read  = mux->i2c_read;
    slot->sensor.delay_ms  = mux->delay_ms;
 
    mux->sensor_count++;
    return TCA9548A_OK;
}
 
TCA9548A_Status_t TCA9548A_InitAllSensors(TCA9548A_Handle_t *mux)
{
    TCA9548A_Status_t ret;
    AS5600_Status_t   as_ret;
 
    if (mux == NULL) return TCA9548A_ERR_NULL_PTR;
 
    for (uint8_t i = 0; i < TCA9548A_MAX_CHANNELS; i++) {
        TCA9548A_SensorSlot_t *slot = &mux->sensors[i];
        if (!slot->enabled) continue;
 
        /* Chọn kênh MUX tương ứng */
        ret = TCA9548A_SelectChannel(mux, slot->channel);
        if (ret != TCA9548A_OK) return ret;
 
        /* Delay nhỏ để kênh ổn định */
        if (mux->delay_ms) mux->delay_ms(2);
 
        /* Khởi tạo AS5600 trên kênh này */
        as_ret = AS5600_Init(&slot->sensor);
        if (as_ret != AS5600_OK) {
            return TCA9548A_ERR_SENSOR_FAIL;
        }
    }
 
    /* Tắt tất cả kênh sau khi khởi tạo xong */
    return TCA9548A_DisableAll(mux);
}
 
TCA9548A_Status_t TCA9548A_ReadSensor(TCA9548A_Handle_t *mux,
                                        TCA9548A_Channel_t ch,
                                        AS5600_Data_t *data)
{
    TCA9548A_Status_t ret;
    AS5600_Status_t   as_ret;
 
    if (mux == NULL || data == NULL) return TCA9548A_ERR_NULL_PTR;
    if (ch >= TCA9548A_MAX_CHANNELS) return TCA9548A_ERR_INVALID_CH;
 
    /* Tìm slot sensor */
    TCA9548A_SensorSlot_t *slot = _find_slot(mux, ch);
    if (slot == NULL) return TCA9548A_ERR_NOT_INIT;
 
    /* Chọn kênh MUX */
    ret = TCA9548A_SelectChannel(mux, ch);
    if (ret != TCA9548A_OK) return ret;
 
    /* Delay nhỏ để kênh ổn định */
    if (mux->delay_ms) mux->delay_ms(1);
 
    /* Đọc dữ liệu AS5600 */
    as_ret = AS5600_ReadAll(&slot->sensor, data);
    if (as_ret != AS5600_OK) {
        TCA9548A_DisableAll(mux); /* Tắt kênh trước khi trả lỗi */
        return TCA9548A_ERR_SENSOR_FAIL;
    }
 
    /* Tắt kênh sau khi đọc xong (mode SINGLE) */
    if (mux->mode == TCA9548A_MODE_SINGLE) {
        TCA9548A_DisableAll(mux);
    }
 
    return TCA9548A_OK;
}
 
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
 
        /* Chọn kênh */
        ret = TCA9548A_SelectChannel(mux, slot->channel);
        if (ret != TCA9548A_OK) {
            result->status[idx] = ret;
            result->count++;
            continue;
        }
 
        if (mux->delay_ms) mux->delay_ms(1);
 
        /* Đọc dữ liệu */
        AS5600_Status_t as_ret = AS5600_ReadAll(&slot->sensor, &result->data[idx]);
        result->status[idx] = (as_ret == AS5600_OK) ?
                               TCA9548A_OK : TCA9548A_ERR_SENSOR_FAIL;
 
        result->count++;
    }
 
    /* Tắt tất cả kênh sau khi quét xong */
    TCA9548A_DisableAll(mux);
 
    return TCA9548A_OK;
}
 
AS5600_Handle_t *TCA9548A_GetSensorHandle(TCA9548A_Handle_t *mux,
                                            TCA9548A_Channel_t ch)
{
    if (mux == NULL || ch >= TCA9548A_MAX_CHANNELS) return NULL;
 
    TCA9548A_SensorSlot_t *slot = _find_slot(mux, ch);
    if (slot == NULL) return NULL;
 
    return &slot->sensor;
}