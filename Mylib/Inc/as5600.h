/*
 * as5600.h
 *
 *  Created on: May 21, 2026
 *      Author: Lap4all
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include <stdint.h>
#include <stdbool.h>
 
#ifdef __cplusplus
extern "C" {
#endif
 
/* ============================================================
 *  ĐỊA CHỈ I2C VÀ THANH GHI (REGISTER MAP)
 * ============================================================ */
 
#define AS5600_I2C_ADDR             0x36    /**< Địa chỉ I2C cố định */
 
/* -- Nhóm cấu hình (Configuration) -- */
#define AS5600_REG_ZMCO             0x00    /**< Số lần burn ZPOS/MPOS */
#define AS5600_REG_ZPOS_H           0x01    /**< Vị trí 0 (High byte) */
#define AS5600_REG_ZPOS_L           0x02    /**< Vị trí 0 (Low byte) */
#define AS5600_REG_MPOS_H           0x03    /**< Vị trí max (High byte) */
#define AS5600_REG_MPOS_L           0x04    /**< Vị trí max (Low byte) */
#define AS5600_REG_MANG_H           0x05    /**< Góc max (High byte) */
#define AS5600_REG_MANG_L           0x06    /**< Góc max (Low byte) */
#define AS5600_REG_CONF_H           0x07    /**< Config (High byte) */
#define AS5600_REG_CONF_L           0x08    /**< Config (Low byte) */
 
/* -- Nhóm đầu ra (Output) -- */
#define AS5600_REG_RAW_ANGLE_H      0x0C    /**< Góc thô (High byte) */
#define AS5600_REG_RAW_ANGLE_L      0x0D    /**< Góc thô (Low byte) */
#define AS5600_REG_ANGLE_H          0x0E    /**< Góc đã lọc (High byte) */
#define AS5600_REG_ANGLE_L          0x0F    /**< Góc đã lọc (Low byte) */
 
/* -- Nhóm trạng thái (Status) -- */
#define AS5600_REG_STATUS           0x0B    /**< Trạng thái cảm biến */
#define AS5600_REG_AGC              0x1A    /**< Automatic Gain Control */
#define AS5600_REG_MAGNITUDE_H      0x1B    /**< Độ lớn từ trường (High) */
#define AS5600_REG_MAGNITUDE_L      0x1C    /**< Độ lớn từ trường (Low) */
 
/* -- Thanh ghi Burn -- */
#define AS5600_REG_BURN             0xFF    /**< Burn command register */
 
/* ============================================================
 *  BIT MASK CHO THANH GHI STATUS
 * ============================================================ */
#define AS5600_STATUS_MH            (1 << 3)    /**< Magnet too strong */
#define AS5600_STATUS_ML            (1 << 4)    /**< Magnet too weak */
#define AS5600_STATUS_MD            (1 << 5)    /**< Magnet detected */
 
/* ============================================================
 *  ENUM ĐỊNH NGHĨA
 * ============================================================ */
 
/**
 * @brief Mã lỗi trả về của thư viện AS5600
 */
typedef enum {
    AS5600_OK               = 0,    /**< Thành công */
    AS5600_ERR_I2C          = -1,   /**< Lỗi giao tiếp I2C */
    AS5600_ERR_NO_MAGNET    = -2,   /**< Không phát hiện nam châm */
    AS5600_ERR_MAGNET_WEAK  = -3,   /**< Từ trường quá yếu */
    AS5600_ERR_MAGNET_STRONG= -4,   /**< Từ trường quá mạnh */
    AS5600_ERR_NULL_PTR     = -5,   /**< Con trỏ null */
    AS5600_ERR_BURN_FAIL    = -6,   /**< Burn OTP thất bại */
} AS5600_Status_t;
 
/**
 * @brief Chế độ nguồn (Power Mode)
 */
typedef enum {
    AS5600_PM_NOM   = 0b00,     /**< Normal mode (không tắt) */
    AS5600_PM_LPM1  = 0b01,     /**< Low power mode 1 (5ms polling) */
    AS5600_PM_LPM2  = 0b10,     /**< Low power mode 2 (20ms polling) */
    AS5600_PM_LPM3  = 0b11,     /**< Low power mode 3 (100ms polling) */
} AS5600_PowerMode_t;
 
/**
 * @brief Chế độ Hysteresis (chống nhiễu biên)
 */
typedef enum {
    AS5600_HYST_OFF = 0b00,     /**< Tắt hysteresis */
    AS5600_HYST_1LSB= 0b01,     /**< 1 LSB */
    AS5600_HYST_2LSB= 0b10,     /**< 2 LSB */
    AS5600_HYST_3LSB= 0b11,     /**< 3 LSB */
} AS5600_Hysteresis_t;
 
/**
 * @brief Loại tín hiệu đầu ra (Output Stage)
 */
typedef enum {
    AS5600_OUT_FULL     = 0b00, /**< Analog: 0% - 100% VCC */
    AS5600_OUT_REDUCED  = 0b01, /**< Analog: 10% - 90% VCC */
    AS5600_OUT_PWM      = 0b10, /**< PWM output */
} AS5600_OutputStage_t;
 
/**
 * @brief Tần số PWM (khi dùng chế độ PWM)
 */
typedef enum {
    AS5600_PWM_115HZ    = 0b00, /**< 115 Hz */
    AS5600_PWM_230HZ    = 0b01, /**< 230 Hz */
    AS5600_PWM_460HZ    = 0b10, /**< 460 Hz */
    AS5600_PWM_920HZ    = 0b11, /**< 920 Hz */
} AS5600_PWMFreq_t;
 
/**
 * @brief Bộ lọc chậm (Slow Filter)
 */
typedef enum {
    AS5600_SF_16X   = 0b00,     /**< 16x (mặc định) */
    AS5600_SF_8X    = 0b01,     /**< 8x */
    AS5600_SF_4X    = 0b10,     /**< 4x */
    AS5600_SF_2X    = 0b11,     /**< 2x */
} AS5600_SlowFilter_t;
 
/**
 * @brief Ngưỡng bộ lọc nhanh (Fast Filter Threshold)
 */
typedef enum {
    AS5600_FTH_SLOW_ONLY    = 0b000,   /**< Chỉ dùng slow filter */
    AS5600_FTH_6LSB         = 0b001,   /**< 6 LSB */
    AS5600_FTH_7LSB         = 0b010,   /**< 7 LSB */
    AS5600_FTH_9LSB         = 0b011,   /**< 9 LSB */
    AS5600_FTH_18LSB        = 0b100,   /**< 18 LSB */
    AS5600_FTH_21LSB        = 0b101,   /**< 21 LSB */
    AS5600_FTH_24LSB        = 0b110,   /**< 24 LSB */
    AS5600_FTH_10LSB        = 0b111,   /**< 10 LSB */
} AS5600_FastFilterThreshold_t;
 
/**
 * @brief Trạng thái nam châm
 */
typedef enum {
    AS5600_MAG_NOT_DETECTED = 0,    /**< Không phát hiện nam châm */
    AS5600_MAG_DETECTED     = 1,    /**< Nam châm OK */
    AS5600_MAG_TOO_WEAK     = 2,    /**< Từ trường quá yếu */
    AS5600_MAG_TOO_STRONG   = 3,    /**< Từ trường quá mạnh */
} AS5600_MagnetStatus_t;
 
/* ============================================================
 *  STRUCT ĐỊNH NGHĨA
 * ============================================================ */
 
/**
 * @brief Cấu hình hoạt động của AS5600
 */
typedef struct {
    AS5600_PowerMode_t          power_mode;     /**< Chế độ nguồn */
    AS5600_Hysteresis_t         hysteresis;     /**< Hysteresis */
    AS5600_OutputStage_t        output_stage;   /**< Loại đầu ra */
    AS5600_PWMFreq_t            pwm_freq;       /**< Tần số PWM */
    AS5600_SlowFilter_t         slow_filter;    /**< Bộ lọc chậm */
    AS5600_FastFilterThreshold_t fast_filter;   /**< Bộ lọc nhanh */
    bool                        watchdog;       /**< Bật watchdog */
} AS5600_Config_t;
 
/**
 * @brief Dữ liệu đọc từ AS5600
 */
typedef struct {
    uint16_t    raw_angle;          /**< Góc thô 12-bit (0 - 4095) */
    uint16_t    angle;              /**< Góc đã lọc 12-bit (0 - 4095) */
    float       angle_deg;          /**< Góc tính theo độ (0.0 - 360.0) */
    float       angle_rad;          /**< Góc tính theo radian (0.0 - 2π) */
    uint16_t    magnitude;          /**< Độ lớn từ trường (0 - 4095) */
    uint8_t     agc;                /**< Giá trị AGC (0 - 255) */
    AS5600_MagnetStatus_t magnet;   /**< Trạng thái nam châm */
} AS5600_Data_t;
 
/**
 * @brief Cấu hình vùng góc (Zone Configuration)
 */
typedef struct {
    uint16_t    start_pos;  /**< Vị trí bắt đầu ZPOS (0 - 4095) */
    uint16_t    stop_pos;   /**< Vị trí kết thúc MPOS (0 - 4095) */
    uint16_t    max_angle;  /**< Góc tối đa MANG (0 - 4095) */
} AS5600_Zone_t;
 
/**
 * @brief Handle (đối tượng) chính của AS5600
 */
typedef struct {
    uint8_t         i2c_addr;       /**< Địa chỉ I2C (mặc định 0x36) */
    AS5600_Config_t config;         /**< Cấu hình hoạt động */
    AS5600_Data_t   data;           /**< Dữ liệu đo được mới nhất */
    AS5600_Zone_t   zone;           /**< Cấu hình vùng góc */
 
    /* Con trỏ hàm HAL - người dùng phải cài đặt trước khi dùng */
    int8_t (*i2c_write)(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len);
    int8_t (*i2c_read) (uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len);
    void   (*delay_ms) (uint32_t ms);
} AS5600_Handle_t;
 
/* ============================================================
 *  API FUNCTIONS
 * ============================================================ */
 
/**
 * @brief Khởi tạo AS5600 với cấu hình mặc định
 * @param dev   Con trỏ đến handle AS5600
 * @return AS5600_OK nếu thành công
 */
AS5600_Status_t AS5600_Init(AS5600_Handle_t *dev);
 
/**
 * @brief Ghi cấu hình vào thanh ghi CONF
 * @param dev   Con trỏ đến handle AS5600
 * @param cfg   Cấu hình cần ghi
 * @return AS5600_OK nếu thành công
 */
AS5600_Status_t AS5600_SetConfig(AS5600_Handle_t *dev, AS5600_Config_t *cfg);
 
/**
 * @brief Đọc lại cấu hình từ thanh ghi CONF
 * @param dev   Con trỏ đến handle AS5600
 * @param cfg   Nơi lưu cấu hình đọc về
 * @return AS5600_OK nếu thành công
 */
AS5600_Status_t AS5600_GetConfig(AS5600_Handle_t *dev, AS5600_Config_t *cfg);
 
/**
 * @brief Cài đặt vùng góc hoạt động (ZPOS, MPOS, MANG)
 * @param dev   Con trỏ đến handle AS5600
 * @param zone  Cấu hình vùng
 * @return AS5600_OK nếu thành công
 */
AS5600_Status_t AS5600_SetZone(AS5600_Handle_t *dev, AS5600_Zone_t *zone);
 
/**
 * @brief Đọc toàn bộ dữ liệu (góc, AGC, magnitude, status)
 * @param dev   Con trỏ đến handle AS5600
 * @param data  Nơi lưu dữ liệu đọc về
 * @return AS5600_OK nếu thành công
 */
AS5600_Status_t AS5600_ReadAll(AS5600_Handle_t *dev, AS5600_Data_t *data);
 
/**
 * @brief Chỉ đọc góc đã lọc (nhanh nhất)
 * @param dev       Con trỏ đến handle AS5600
 * @param angle     Giá trị góc raw 12-bit (0-4095)
 * @return AS5600_OK nếu thành công
 */
AS5600_Status_t AS5600_ReadAngle(AS5600_Handle_t *dev, uint16_t *angle);
 
/**
 * @brief Đọc góc thô (chưa lọc)
 * @param dev       Con trỏ đến handle AS5600
 * @param raw_angle Giá trị góc thô 12-bit (0-4095)
 * @return AS5600_OK nếu thành công
 */
AS5600_Status_t AS5600_ReadRawAngle(AS5600_Handle_t *dev, uint16_t *raw_angle);
 
/**
 * @brief Đọc góc tính ra độ
 * @param dev       Con trỏ đến handle AS5600
 * @param degrees   Giá trị góc (0.0 - 360.0)
 * @return AS5600_OK nếu thành công
 */
AS5600_Status_t AS5600_ReadAngleDeg(AS5600_Handle_t *dev, float *degrees);
 
/**
 * @brief Kiểm tra trạng thái nam châm
 * @param dev       Con trỏ đến handle AS5600
 * @param status    Trạng thái nam châm
 * @return AS5600_OK nếu thành công
 */
AS5600_Status_t AS5600_GetMagnetStatus(AS5600_Handle_t *dev, AS5600_MagnetStatus_t *status);
 
/**
 * @brief Đọc giá trị AGC
 * @param dev   Con trỏ đến handle AS5600
 * @param agc   Giá trị AGC (0-255)
 * @return AS5600_OK nếu thành công
 */
AS5600_Status_t AS5600_ReadAGC(AS5600_Handle_t *dev, uint8_t *agc);
 
/**
 * @brief Đọc độ lớn từ trường
 * @param dev       Con trỏ đến handle AS5600
 * @param magnitude Giá trị magnitude (0-4095)
 * @return AS5600_OK nếu thành công
 */
AS5600_Status_t AS5600_ReadMagnitude(AS5600_Handle_t *dev, uint16_t *magnitude);
 
/**
 * @brief Burn cấu hình vào OTP (không thể hoàn tác!)
 * @param dev   Con trỏ đến handle AS5600
 * @return AS5600_OK nếu thành công
 * @note  Chỉ burn tối đa 3 lần
 */
AS5600_Status_t AS5600_BurnSettings(AS5600_Handle_t *dev);
 
/**
 * @brief Burn góc zero vào OTP (không thể hoàn tác!)
 * @param dev   Con trỏ đến handle AS5600
 * @return AS5600_OK nếu thành công
 */
AS5600_Status_t AS5600_BurnAngle(AS5600_Handle_t *dev);
 
/**
 * @brief Kiểm tra kết nối với cảm biến
 * @param dev   Con trỏ đến handle AS5600
 * @return AS5600_OK nếu cảm biến phản hồi
 */
AS5600_Status_t AS5600_IsConnected(AS5600_Handle_t *dev);
 
/**
 * @brief Chuyển đổi giá trị 12-bit sang độ
 * @param raw   Giá trị 12-bit (0-4095)
 * @return Góc tính theo độ (0.0 - 360.0)
 */
float AS5600_RawToDegrees(uint16_t raw);
 
/**
 * @brief Chuyển đổi giá trị 12-bit sang radian
 * @param raw   Giá trị 12-bit (0-4095)
 * @return Góc tính theo radian (0 - 2π)
 */
float AS5600_RawToRadians(uint16_t raw);
 
#ifdef __cplusplus
}
#endif


#endif /* INC_AS5600_H_ */
