/**
 * @file    as5600.c
 * @brief   Trien khai driver doc cam bien goc tu AS5600 qua I2C.
 * @author  Lap4all
 * @date    2026-05-21
 */

#include "as5600.h"
#include "my_app.h"
#include <stddef.h>

/* Private defines ---------------------------------------------------------- */

#define AS5600_RAW_MAX          4096.0f         /**< So muc raw cua ADC 12-bit. */
#define AS5600_DEG_PER_RAW      (360.0f / AS5600_RAW_MAX)              /**< Do tren moi raw count. */
#define AS5600_RAD_PER_RAW      (2.0f * 3.14159265f / AS5600_RAW_MAX)  /**< Radian tren moi raw count. */

#define AS5600_BURN_ANGLE_CMD   0x80    /**< Lenh burn goc zero vao OTP. */
#define AS5600_BURN_SETTING_CMD 0x40    /**< Lenh burn cau hinh vao OTP. */

/* Private functions -------------------------------------------------------- */

/* Kiem tra cac callback I2C bat buoc truoc khi truy cap cam bien. */
static AS5600_Status_t _check_hal(AS5600_Handle_t *dev)
{
    if (dev == NULL)                return AS5600_ERR_NULL_PTR;
    if (dev->i2c_write == NULL)     return AS5600_ERR_NULL_PTR;
    if (dev->i2c_read  == NULL)     return AS5600_ERR_NULL_PTR;
    return AS5600_OK;
}

/* Ghi mot byte vao thanh ghi AS5600. */
static AS5600_Status_t _write_reg(AS5600_Handle_t *dev, uint8_t reg, uint8_t val)
{
    int8_t ret = dev->i2c_write(dev->i2c_addr, reg, &val, 1);
    return (ret == 0) ? AS5600_OK : AS5600_ERR_I2C;
}

/* Doc mot byte tu thanh ghi AS5600. */
static AS5600_Status_t _read_reg(AS5600_Handle_t *dev, uint8_t reg, uint8_t *val)
{
    int8_t ret = dev->i2c_read(dev->i2c_addr, reg, val, 1);
    return (ret == 0) ? AS5600_OK : AS5600_ERR_I2C;
}

/* Doc hai byte lien tiep va ghep theo thu tu big-endian cua AS5600. */
static AS5600_Status_t _read_reg16(AS5600_Handle_t *dev, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    int8_t ret = dev->i2c_read(dev->i2c_addr, reg, buf, 2);
    if (ret != 0) return AS5600_ERR_I2C;
    *val = ((uint16_t)buf[0] << 8) | buf[1];
    return AS5600_OK;
}

/* Ghi hai byte lien tiep theo thu tu big-endian cua AS5600. */
static AS5600_Status_t _write_reg16(AS5600_Handle_t *dev, uint8_t reg, uint16_t val)
{
    uint8_t buf[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    int8_t ret = dev->i2c_write(dev->i2c_addr, reg, buf, 2);
    return (ret == 0) ? AS5600_OK : AS5600_ERR_I2C;
}

/* Function definitions ----------------------------------------------------- */

/**
 * @brief  Khoi tao AS5600 voi cau hinh mac dinh.
 * @param[in,out] dev: Con tro den handle AS5600.
 * @return AS5600_OK neu khoi tao thanh cong, nguoc lai tra ve ma loi.
 */
AS5600_Status_t AS5600_Init(AS5600_Handle_t *dev)
{
    AS5600_Status_t ret;

    ret = _check_hal(dev);
    if (ret != AS5600_OK) return ret;

    /* Gan dia chi I2C mac dinh khi nguoi dung chua cau hinh. */
    if (dev->i2c_addr == 0) {
        dev->i2c_addr = AS5600_I2C_ADDR;
    }

    /* Doc thanh ghi STATUS de xac nhan cam bien phan hoi tren bus. */
    ret = AS5600_IsConnected(dev);
    if (ret != AS5600_OK) return ret;

    /* Cau hinh mac dinh: normal mode, full analog output, slow filter 16x. */
    dev->config.power_mode   = AS5600_PM_NOM;
    dev->config.hysteresis   = AS5600_HYST_OFF;
    dev->config.output_stage = AS5600_OUT_FULL;
    dev->config.pwm_freq     = AS5600_PWM_115HZ;
    dev->config.slow_filter  = AS5600_SF_16X;
    dev->config.fast_filter  = AS5600_FTH_SLOW_ONLY;
    dev->config.watchdog     = false;

    return AS5600_SetConfig(dev, &dev->config);
}

/**
 * @brief  Ghi cau hinh vao thanh ghi CONF cua AS5600.
 * @param[in,out] dev: Con tro den handle AS5600.
 * @param[in] cfg: Cau hinh can ghi.
 * @return AS5600_OK neu ghi thanh cong, nguoc lai tra ve ma loi.
 */
AS5600_Status_t AS5600_SetConfig(AS5600_Handle_t *dev, AS5600_Config_t *cfg)
{
    AS5600_Status_t ret;
    if (dev == NULL || cfg == NULL) return AS5600_ERR_NULL_PTR;

    /*
     * Thanh ghi CONF duoc ghi qua hai byte lien tiep.
     * PM/HYST/OUTS/PWMF nam o byte thap, WD/FTH/SF nam o byte cao.
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

/**
 * @brief  Doc cau hinh hien tai tu thanh ghi CONF.
 * @param[in] dev: Con tro den handle AS5600.
 * @param[out] cfg: Noi luu cau hinh doc duoc.
 * @return AS5600_OK neu doc thanh cong, nguoc lai tra ve ma loi.
 */
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

/**
 * @brief  Cai dat vung goc lam viec ZPOS, MPOS va MANG.
 * @param[in,out] dev: Con tro den handle AS5600.
 * @param[in] zone: Cau hinh vung goc can ghi.
 * @return AS5600_OK neu ghi thanh cong, nguoc lai tra ve ma loi.
 */
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

/**
 * @brief  Doc toan bo du lieu do va trang thai nam cham cua AS5600.
 * @param[in,out] dev: Con tro den handle AS5600.
 * @param[out] data: Noi luu du lieu doc duoc.
 * @return AS5600_OK neu doc thanh cong, nguoc lai tra ve ma loi.
 */
AS5600_Status_t AS5600_ReadAll(AS5600_Handle_t *dev, AS5600_Data_t *data)
{
    AS5600_Status_t ret;
    uint8_t  status_byte;
    uint16_t raw, angle, magnitude;
    uint8_t  agc;

    if (dev == NULL || data == NULL) return AS5600_ERR_NULL_PTR;

    /* Doc STATUS truoc de phat hien loi dat nam cham. */
    ret = _read_reg(dev, AS5600_REG_STATUS, &status_byte);
    if (ret != AS5600_OK) return ret;

    /* Thu tu uu tien: chua co nam cham, qua manh, qua yeu, hop le. */
    if (!(status_byte & AS5600_STATUS_MD)) {
        data->magnet = AS5600_MAG_NOT_DETECTED;
    } else if (status_byte & AS5600_STATUS_MH) {
        data->magnet = AS5600_MAG_TOO_STRONG;
    } else if (status_byte & AS5600_STATUS_ML) {
        data->magnet = AS5600_MAG_TOO_WEAK;
    } else {
        data->magnet = AS5600_MAG_DETECTED;
    }

    /* Raw angle la goc chua qua bo loc noi bo cua AS5600. */
    ret = _read_reg16(dev, AS5600_REG_RAW_ANGLE_H, &raw);
    if (ret != AS5600_OK) return ret;
    data->raw_angle = raw & 0x0FFF;

    /* ANGLE la gia tri da qua bo loc noi bo cua AS5600. */
    ret = _read_reg16(dev, AS5600_REG_ANGLE_H, &angle);
    if (ret != AS5600_OK) return ret;
    data->angle = angle & 0x0FFF;

#if (MY_APP_AS5600_FLOAT_UNITS == MY_APP_MODULE_ENABLED)
    /* Luu san don vi thong dung de tang tinh tien dung cho lop ung dung. */
    data->angle_deg = AS5600_RawToDegrees(data->angle);
    data->angle_rad = AS5600_RawToRadians(data->angle);
#elif (MY_APP_AS5600_FLOAT_UNITS == MY_APP_MODULE_DISABLED)
    data->angle_deg = 0.0f;
    data->angle_rad = 0.0f;
#else
#error "Invalid MY_APP_AS5600_FLOAT_UNITS setting"
#endif

    ret = _read_reg(dev, AS5600_REG_AGC, &agc);
    if (ret != AS5600_OK) return ret;
    data->agc = agc;

    ret = _read_reg16(dev, AS5600_REG_MAGNITUDE_H, &magnitude);
    if (ret != AS5600_OK) return ret;
    data->magnitude = magnitude & 0x0FFF;

    /* Cache giu ban sao moi nhat de cac ham khac co the tham chieu nhanh. */
    dev->data = *data;

    return AS5600_OK;
}

/**
 * @brief  Doc goc da loc dang raw 12-bit.
 * @param[in,out] dev: Con tro den handle AS5600.
 * @param[out] angle: Noi luu goc raw 12-bit, gia tri 0 den 4095.
 * @return AS5600_OK neu doc thanh cong, nguoc lai tra ve ma loi.
 */
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

/**
 * @brief  Doc goc tho chua qua bo loc noi bo cua AS5600.
 * @param[in,out] dev: Con tro den handle AS5600.
 * @param[out] raw_angle: Noi luu goc raw 12-bit, gia tri 0 den 4095.
 * @return AS5600_OK neu doc thanh cong, nguoc lai tra ve ma loi.
 */
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

/**
 * @brief  Doc goc da loc va chuyen sang don vi do.
 * @param[in,out] dev: Con tro den handle AS5600.
 * @param[out] degrees: Noi luu goc theo do, xap xi 0.0 den 360.0.
 * @return AS5600_OK neu doc thanh cong, nguoc lai tra ve ma loi.
 */
#if (MY_APP_AS5600_FLOAT_UNITS == MY_APP_MODULE_ENABLED)
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
#elif (MY_APP_AS5600_FLOAT_UNITS == MY_APP_MODULE_DISABLED)
#else
#error "Invalid MY_APP_AS5600_FLOAT_UNITS setting"
#endif

/**
 * @brief  Doc va phan loai trang thai nam cham.
 * @param[in,out] dev: Con tro den handle AS5600.
 * @param[out] status: Noi luu trang thai nam cham.
 * @return AS5600_OK neu doc thanh cong, nguoc lai tra ve ma loi.
 */
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

/**
 * @brief  Doc gia tri AGC cua AS5600.
 * @param[in,out] dev: Con tro den handle AS5600.
 * @param[out] agc: Noi luu gia tri automatic gain control.
 * @return AS5600_OK neu doc thanh cong, nguoc lai tra ve ma loi.
 */
AS5600_Status_t AS5600_ReadAGC(AS5600_Handle_t *dev, uint8_t *agc)
{
    if (dev == NULL || agc == NULL) return AS5600_ERR_NULL_PTR;
    AS5600_Status_t ret = _read_reg(dev, AS5600_REG_AGC, agc);
    if (ret == AS5600_OK) dev->data.agc = *agc;
    return ret;
}

/**
 * @brief  Doc do lon tu truong ma AS5600 dang do duoc.
 * @param[in,out] dev: Con tro den handle AS5600.
 * @param[out] magnitude: Noi luu magnitude 12-bit, gia tri 0 den 4095.
 * @return AS5600_OK neu doc thanh cong, nguoc lai tra ve ma loi.
 */
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

/**
 * @brief  Ghi cau hinh vao OTP cua AS5600.
 * @param[in] dev: Con tro den handle AS5600.
 * @return AS5600_OK neu burn thanh cong, nguoc lai tra ve ma loi.
 * @warning Lenh burn OTP khong the hoan tac va chi co so lan ghi gioi han.
 */
AS5600_Status_t AS5600_BurnSettings(AS5600_Handle_t *dev)
{
    AS5600_Status_t ret;
    uint8_t zmco;

    if (dev == NULL) return AS5600_ERR_NULL_PTR;

    /* ZMCO cho biet so lan burn ZPOS/MPOS da su dung. */
    ret = _read_reg(dev, AS5600_REG_ZMCO, &zmco);
    if (ret != AS5600_OK) return ret;

    if (zmco >= 3) return AS5600_ERR_BURN_FAIL;

    return _write_reg(dev, AS5600_REG_BURN, AS5600_BURN_SETTING_CMD);
}

/**
 * @brief  Ghi goc zero hien tai vao OTP cua AS5600.
 * @param[in] dev: Con tro den handle AS5600.
 * @return AS5600_OK neu burn thanh cong, nguoc lai tra ve ma loi.
 * @warning Lenh burn OTP khong the hoan tac.
 */
AS5600_Status_t AS5600_BurnAngle(AS5600_Handle_t *dev)
{
    if (dev == NULL) return AS5600_ERR_NULL_PTR;
    return _write_reg(dev, AS5600_REG_BURN, AS5600_BURN_ANGLE_CMD);
}

/**
 * @brief  Kiem tra AS5600 co phan hoi tren bus I2C hay khong.
 * @param[in] dev: Con tro den handle AS5600.
 * @return AS5600_OK neu doc duoc thanh ghi STATUS, nguoc lai tra ve ma loi.
 */
AS5600_Status_t AS5600_IsConnected(AS5600_Handle_t *dev)
{
    uint8_t dummy;
    AS5600_Status_t ret;

    ret = _check_hal(dev);
    if (ret != AS5600_OK) return ret;

    ret = _read_reg(dev, AS5600_REG_STATUS, &dummy);
    return ret;
}

/**
 * @brief  Chuyen gia tri raw 12-bit sang don vi do.
 * @param[in] raw: Gia tri raw 12-bit cua AS5600.
 * @return Goc theo do, xap xi 0.0 den 360.0.
 */
#if (MY_APP_AS5600_FLOAT_UNITS == MY_APP_MODULE_ENABLED)
float AS5600_RawToDegrees(uint16_t raw)
{
    return (float)(raw & 0x0FFF) * AS5600_DEG_PER_RAW;
}

/**
 * @brief  Chuyen gia tri raw 12-bit sang don vi radian.
 * @param[in] raw: Gia tri raw 12-bit cua AS5600.
 * @return Goc theo radian, xap xi 0.0 den 2*pi.
 */
float AS5600_RawToRadians(uint16_t raw)
{
    return (float)(raw & 0x0FFF) * AS5600_RAD_PER_RAW;
}
#elif (MY_APP_AS5600_FLOAT_UNITS == MY_APP_MODULE_DISABLED)
#else
#error "Invalid MY_APP_AS5600_FLOAT_UNITS setting"
#endif
