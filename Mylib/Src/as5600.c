/*
 * as5600.c
 *
 *  Created on: May 21, 2026
 *      Author: Lap4all
 */


#include "as5600.h"
#include <math.h>   /* fabsf, M_PI */
 
/* ============================================================
 *  HẰNG SỐ NỘI BỘ
 * ============================================================ */
#define AS5600_RAW_MAX          4096.0f         /**< 2^12 */
#define AS5600_DEG_PER_RAW      (360.0f / AS5600_RAW_MAX)
#define AS5600_RAD_PER_RAW      (2.0f * 3.14159265f / AS5600_RAW_MAX)
 
#define AS5600_BURN_ANGLE_CMD   0x80
#define AS5600_BURN_SETTING_CMD 0x40
 
/* ============================================================
 *  HÀM NỘI BỘ (PRIVATE)
 * ============================================================ */
 
/**
 * @brief  Kiểm tra con trỏ hàm HAL đã được cài đặt chưa
 */
static AS5600_Status_t _check_hal(AS5600_Handle_t *dev)
{
    if (dev == NULL)                return AS5600_ERR_NULL_PTR;
    if (dev->i2c_write == NULL)     return AS5600_ERR_NULL_PTR;
    if (dev->i2c_read  == NULL)     return AS5600_ERR_NULL_PTR;
    return AS5600_OK;
}
 
/**
 * @brief  Ghi 1 byte vào thanh ghi
 */
static AS5600_Status_t _write_reg(AS5600_Handle_t *dev, uint8_t reg, uint8_t val)
{
    int8_t ret = dev->i2c_write(dev->i2c_addr, reg, &val, 1);
    return (ret == 0) ? AS5600_OK : AS5600_ERR_I2C;
}
 
/**
 * @brief  Đọc 1 byte từ thanh ghi
 */
static AS5600_Status_t _read_reg(AS5600_Handle_t *dev, uint8_t reg, uint8_t *val)
{
    int8_t ret = dev->i2c_read(dev->i2c_addr, reg, val, 1);
    return (ret == 0) ? AS5600_OK : AS5600_ERR_I2C;
}
 
/**
 * @brief  Đọc 2 byte liên tiếp và ghép thành uint16_t (big-endian)
 */
static AS5600_Status_t _read_reg16(AS5600_Handle_t *dev, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    int8_t ret = dev->i2c_read(dev->i2c_addr, reg, buf, 2);
    if (ret != 0) return AS5600_ERR_I2C;
    *val = ((uint16_t)buf[0] << 8) | buf[1];
    return AS5600_OK;
}
 
/**
 * @brief  Ghi 2 byte (big-endian) vào thanh ghi liên tiếp
 */
static AS5600_Status_t _write_reg16(AS5600_Handle_t *dev, uint8_t reg, uint16_t val)
{
    uint8_t buf[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    int8_t ret = dev->i2c_write(dev->i2c_addr, reg, buf, 2);
    return (ret == 0) ? AS5600_OK : AS5600_ERR_I2C;
}
 
/* ============================================================
 *  TRIỂN KHAI HÀM PUBLIC
 * ============================================================ */
 
AS5600_Status_t AS5600_Init(AS5600_Handle_t *dev)
{
    AS5600_Status_t ret;
 
    ret = _check_hal(dev);
    if (ret != AS5600_OK) return ret;
 
    /* Cài địa chỉ mặc định nếu bằng 0 */
    if (dev->i2c_addr == 0) {
        dev->i2c_addr = AS5600_I2C_ADDR;
    }
 
    /* Kiểm tra kết nối */
    ret = AS5600_IsConnected(dev);
    if (ret != AS5600_OK) return ret;
 
    /* Cài cấu hình mặc định: Normal, no hysteresis, full analog, 16x filter */
    dev->config.power_mode   = AS5600_PM_NOM;
    dev->config.hysteresis   = AS5600_HYST_OFF;
    dev->config.output_stage = AS5600_OUT_FULL;
    dev->config.pwm_freq     = AS5600_PWM_115HZ;
    dev->config.slow_filter  = AS5600_SF_16X;
    dev->config.fast_filter  = AS5600_FTH_SLOW_ONLY;
    dev->config.watchdog     = false;
 
    return AS5600_SetConfig(dev, &dev->config);
}
 
AS5600_Status_t AS5600_SetConfig(AS5600_Handle_t *dev, AS5600_Config_t *cfg)
{
    AS5600_Status_t ret;
    if (dev == NULL || cfg == NULL) return AS5600_ERR_NULL_PTR;
 
    /*
     * Cấu trúc thanh ghi CONF (2 byte):
     *  Byte H (0x07): [15:14]=PM, [13:12]=HYST, [11:10]=OUTS, [9:8]=PWMF
     *  Byte L (0x08): [7]=WD, [6:5]=FTH, [4:3]=SF (bit 2:0 không dùng)
     */
    uint16_t conf_val = 0;
 
    conf_val |= ((uint16_t)(cfg->power_mode   & 0x03) << 0);   /* bit 1:0 */
    conf_val |= ((uint16_t)(cfg->hysteresis   & 0x03) << 2);   /* bit 3:2 */
    conf_val |= ((uint16_t)(cfg->output_stage & 0x03) << 4);   /* bit 5:4 */
    conf_val |= ((uint16_t)(cfg->pwm_freq     & 0x03) << 6);   /* bit 7:6 */
    conf_val |= ((uint16_t)(cfg->slow_filter  & 0x03) << 8);   /* bit 9:8 */
    conf_val |= ((uint16_t)(cfg->fast_filter  & 0x07) << 10);  /* bit 12:10 */
    conf_val |= ((uint16_t)(cfg->watchdog ? 1 : 0)    << 13);  /* bit 13 */
 
    ret = _write_reg16(dev, AS5600_REG_CONF_H, conf_val);
    if (ret == AS5600_OK) {
        dev->config = *cfg;
    }
    return ret;
}
 
AS5600_Status_t AS5600_GetConfig(AS5600_Handle_t *dev, AS5600_Config_t *cfg)
{
    AS5600_Status_t ret;
    uint16_t conf_val;
 
    if (dev == NULL || cfg == NULL) return AS5600_ERR_NULL_PTR;
 
    ret = _read_reg16(dev, AS5600_REG_CONF_H, &conf_val);
    if (ret != AS5600_OK) return ret;
 
    cfg->power_mode   = (AS5600_PowerMode_t)         ((conf_val >> 0)  & 0x03);
    cfg->hysteresis   = (AS5600_Hysteresis_t)         ((conf_val >> 2)  & 0x03);
    cfg->output_stage = (AS5600_OutputStage_t)        ((conf_val >> 4)  & 0x03);
    cfg->pwm_freq     = (AS5600_PWMFreq_t)            ((conf_val >> 6)  & 0x03);
    cfg->slow_filter  = (AS5600_SlowFilter_t)         ((conf_val >> 8)  & 0x03);
    cfg->fast_filter  = (AS5600_FastFilterThreshold_t)((conf_val >> 10) & 0x07);
    cfg->watchdog     = (bool)                        ((conf_val >> 13) & 0x01);
 
    return AS5600_OK;
}
 
AS5600_Status_t AS5600_SetZone(AS5600_Handle_t *dev, AS5600_Zone_t *zone)
{
    AS5600_Status_t ret;
    if (dev == NULL || zone == NULL) return AS5600_ERR_NULL_PTR;
 
    ret = _write_reg16(dev, AS5600_REG_ZPOS_H, zone->start_pos & 0x0FFF);
    if (ret != AS5600_OK) return ret;
 
    ret = _write_reg16(dev, AS5600_REG_MPOS_H, zone->stop_pos & 0x0FFF);
    if (ret != AS5600_OK) return ret;
 
    ret = _write_reg16(dev, AS5600_REG_MANG_H, zone->max_angle & 0x0FFF);
    if (ret == AS5600_OK) {
        dev->zone = *zone;
    }
    return ret;
}
 
AS5600_Status_t AS5600_ReadAll(AS5600_Handle_t *dev, AS5600_Data_t *data)
{
    AS5600_Status_t ret;
    uint8_t  status_byte;
    uint16_t raw, angle, magnitude;
    uint8_t  agc;
 
    if (dev == NULL || data == NULL) return AS5600_ERR_NULL_PTR;
 
    /* Đọc status */
    ret = _read_reg(dev, AS5600_REG_STATUS, &status_byte);
    if (ret != AS5600_OK) return ret;
 
    /* Xác định trạng thái nam châm */
    if (!(status_byte & AS5600_STATUS_MD)) {
        data->magnet = AS5600_MAG_NOT_DETECTED;
    } else if (status_byte & AS5600_STATUS_MH) {
        data->magnet = AS5600_MAG_TOO_STRONG;
    } else if (status_byte & AS5600_STATUS_ML) {
        data->magnet = AS5600_MAG_TOO_WEAK;
    } else {
        data->magnet = AS5600_MAG_DETECTED;
    }
 
    /* Đọc góc thô */
    ret = _read_reg16(dev, AS5600_REG_RAW_ANGLE_H, &raw);
    if (ret != AS5600_OK) return ret;
    data->raw_angle = raw & 0x0FFF;
 
    /* Đọc góc đã lọc */
    ret = _read_reg16(dev, AS5600_REG_ANGLE_H, &angle);
    if (ret != AS5600_OK) return ret;
    data->angle = angle & 0x0FFF;
 
    /* Tính góc theo độ và radian */
    data->angle_deg = AS5600_RawToDegrees(data->angle);
    data->angle_rad = AS5600_RawToRadians(data->angle);
 
    /* Đọc AGC */
    ret = _read_reg(dev, AS5600_REG_AGC, &agc);
    if (ret != AS5600_OK) return ret;
    data->agc = agc;
 
    /* Đọc magnitude */
    ret = _read_reg16(dev, AS5600_REG_MAGNITUDE_H, &magnitude);
    if (ret != AS5600_OK) return ret;
    data->magnitude = magnitude & 0x0FFF;
 
    /* Cập nhật cache trong handle */
    dev->data = *data;
 
    return AS5600_OK;
}
 
AS5600_Status_t AS5600_ReadAngle(AS5600_Handle_t *dev, uint16_t *angle)
{
    AS5600_Status_t ret;
    uint16_t raw;
 
    if (dev == NULL || angle == NULL) return AS5600_ERR_NULL_PTR;
 
    ret = _read_reg16(dev, AS5600_REG_ANGLE_H, &raw);
    if (ret != AS5600_OK) return ret;
 
    *angle = raw & 0x0FFF;
    dev->data.angle = *angle;
    return AS5600_OK;
}
 
AS5600_Status_t AS5600_ReadRawAngle(AS5600_Handle_t *dev, uint16_t *raw_angle)
{
    AS5600_Status_t ret;
    uint16_t raw;
 
    if (dev == NULL || raw_angle == NULL) return AS5600_ERR_NULL_PTR;
 
    ret = _read_reg16(dev, AS5600_REG_RAW_ANGLE_H, &raw);
    if (ret != AS5600_OK) return ret;
 
    *raw_angle = raw & 0x0FFF;
    dev->data.raw_angle = *raw_angle;
    return AS5600_OK;
}
 
AS5600_Status_t AS5600_ReadAngleDeg(AS5600_Handle_t *dev, float *degrees)
{
    AS5600_Status_t ret;
    uint16_t angle;
 
    if (dev == NULL || degrees == NULL) return AS5600_ERR_NULL_PTR;
 
    ret = AS5600_ReadAngle(dev, &angle);
    if (ret != AS5600_OK) return ret;
 
    *degrees = AS5600_RawToDegrees(angle);
    dev->data.angle_deg = *degrees;
    return AS5600_OK;
}
 
AS5600_Status_t AS5600_GetMagnetStatus(AS5600_Handle_t *dev, AS5600_MagnetStatus_t *status)
{
    AS5600_Status_t ret;
    uint8_t byte;
 
    if (dev == NULL || status == NULL) return AS5600_ERR_NULL_PTR;
 
    ret = _read_reg(dev, AS5600_REG_STATUS, &byte);
    if (ret != AS5600_OK) return ret;
 
    if (!(byte & AS5600_STATUS_MD)) {
        *status = AS5600_MAG_NOT_DETECTED;
    } else if (byte & AS5600_STATUS_MH) {
        *status = AS5600_MAG_TOO_STRONG;
    } else if (byte & AS5600_STATUS_ML) {
        *status = AS5600_MAG_TOO_WEAK;
    } else {
        *status = AS5600_MAG_DETECTED;
    }
 
    dev->data.magnet = *status;
    return AS5600_OK;
}
 
AS5600_Status_t AS5600_ReadAGC(AS5600_Handle_t *dev, uint8_t *agc)
{
    if (dev == NULL || agc == NULL) return AS5600_ERR_NULL_PTR;
    AS5600_Status_t ret = _read_reg(dev, AS5600_REG_AGC, agc);
    if (ret == AS5600_OK) dev->data.agc = *agc;
    return ret;
}
 
AS5600_Status_t AS5600_ReadMagnitude(AS5600_Handle_t *dev, uint16_t *magnitude)
{
    AS5600_Status_t ret;
    uint16_t raw;
 
    if (dev == NULL || magnitude == NULL) return AS5600_ERR_NULL_PTR;
 
    ret = _read_reg16(dev, AS5600_REG_MAGNITUDE_H, &raw);
    if (ret != AS5600_OK) return ret;
 
    *magnitude = raw & 0x0FFF;
    dev->data.magnitude = *magnitude;
    return AS5600_OK;
}
 
AS5600_Status_t AS5600_BurnSettings(AS5600_Handle_t *dev)
{
    AS5600_Status_t ret;
    uint8_t zmco;
 
    if (dev == NULL) return AS5600_ERR_NULL_PTR;
 
    /* Kiểm tra số lần burn còn lại */
    ret = _read_reg(dev, AS5600_REG_ZMCO, &zmco);
    if (ret != AS5600_OK) return ret;
 
    if (zmco >= 3) return AS5600_ERR_BURN_FAIL;
 
    return _write_reg(dev, AS5600_REG_BURN, AS5600_BURN_SETTING_CMD);
}
 
AS5600_Status_t AS5600_BurnAngle(AS5600_Handle_t *dev)
{
    if (dev == NULL) return AS5600_ERR_NULL_PTR;
    return _write_reg(dev, AS5600_REG_BURN, AS5600_BURN_ANGLE_CMD);
}
 
AS5600_Status_t AS5600_IsConnected(AS5600_Handle_t *dev)
{
    uint8_t dummy;
    AS5600_Status_t ret;
 
    ret = _check_hal(dev);
    if (ret != AS5600_OK) return ret;
 
    ret = _read_reg(dev, AS5600_REG_STATUS, &dummy);
    return ret;
}
 
float AS5600_RawToDegrees(uint16_t raw)
{
    return (float)(raw & 0x0FFF) * AS5600_DEG_PER_RAW;
}
 
float AS5600_RawToRadians(uint16_t raw)
{
    return (float)(raw & 0x0FFF) * AS5600_RAD_PER_RAW;
}