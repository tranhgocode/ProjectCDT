/*
 * tca9548a.h
 *
 *  Created on: May 21, 2026
 *      Author: Lap4all
 */

#ifndef INC_TCA9548A_H_
#define INC_TCA9548A_H_

#include <stdint.h>
#include <stdbool.h>
#include "as5600.h"
 
#ifdef __cplusplus
extern "C" {
#endif
 
/* ============================================================
 *  ĐỊA CHỈ VÀ HẰNG SỐ
 * ============================================================ */
 
#define TCA9548A_BASE_ADDR      0x70    /**< Địa chỉ cơ sở (A2=A1=A0=0) */
#define TCA9548A_ADDR_MAX       0x77    /**< Địa chỉ cao nhất */
#define TCA9548A_MAX_CHANNELS   8       /**< Số kênh tối đa */
#define TCA9548A_MAX_SENSORS    8       /**< Số AS5600 tối đa trên 1 mux */
 
/** Thanh ghi điều khiển kênh (1 byte duy nhất) */
#define TCA9548A_CTRL_REG       0x00
#define TCA9548A_ALL_CHANNELS_OFF   0x00    /**< Tắt tất cả kênh */
 
/* ============================================================
 *  ENUM ĐỊNH NGHĨA
 * ============================================================ */
 
/**
 * @brief Mã lỗi trả về của thư viện TCA9548A
 */
typedef enum {
    TCA9548A_OK                 =  0,   /**< Thành công */
    TCA9548A_ERR_I2C            = -1,   /**< Lỗi giao tiếp I2C */
    TCA9548A_ERR_NULL_PTR       = -2,   /**< Con trỏ null */
    TCA9548A_ERR_INVALID_CH     = -3,   /**< Kênh không hợp lệ (>7) */
    TCA9548A_ERR_SENSOR_FAIL    = -4,   /**< Lỗi từ cảm biến AS5600 */
    TCA9548A_ERR_NOT_INIT       = -5,   /**< Chưa khởi tạo kênh/sensor */
    TCA9548A_ERR_CHANNEL_BUSY   = -6,   /**< Kênh đang bận */
} TCA9548A_Status_t;
 
/**
 * @brief Chỉ số kênh của TCA9548A
 */
typedef enum {
    TCA9548A_CH0 = 0,   /**< Kênh 0 - bit 0 của control register */
    TCA9548A_CH1 = 1,   /**< Kênh 1 */
    TCA9548A_CH2 = 2,   /**< Kênh 2 */
    TCA9548A_CH3 = 3,   /**< Kênh 3 */
    TCA9548A_CH4 = 4,   /**< Kênh 4 */
    TCA9548A_CH5 = 5,   /**< Kênh 5 */
    TCA9548A_CH6 = 6,   /**< Kênh 6 */
    TCA9548A_CH7 = 7,   /**< Kênh 7 */
    TCA9548A_CH_NONE = 0xFF,    /**< Không chọn kênh nào */
} TCA9548A_Channel_t;
 
/**
 * @brief Chế độ chọn kênh
 */
typedef enum {
    TCA9548A_MODE_SINGLE    = 0,    /**< Chỉ bật 1 kênh tại một thời điểm */
    TCA9548A_MODE_MULTI     = 1,    /**< Cho phép bật nhiều kênh cùng lúc */
} TCA9548A_ChannelMode_t;
 
/**
 * @brief Trạng thái của từng kênh
 */
typedef enum {
    TCA9548A_CH_DISABLED    = 0,    /**< Kênh bị tắt */
    TCA9548A_CH_ENABLED     = 1,    /**< Kênh đang bật */
} TCA9548A_ChannelState_t;
 
/**
 * @brief Địa chỉ I2C tương ứng với cấu hình chân A2:A1:A0
 */
typedef enum {
    TCA9548A_ADDR_A000 = 0x70,  /**< A2=0, A1=0, A0=0 */
    TCA9548A_ADDR_A001 = 0x71,  /**< A2=0, A1=0, A0=1 */
    TCA9548A_ADDR_A010 = 0x72,  /**< A2=0, A1=1, A0=0 */
    TCA9548A_ADDR_A011 = 0x73,  /**< A2=0, A1=1, A0=1 */
    TCA9548A_ADDR_A100 = 0x74,  /**< A2=1, A1=0, A0=0 */
    TCA9548A_ADDR_A101 = 0x75,  /**< A2=1, A1=0, A0=1 */
    TCA9548A_ADDR_A110 = 0x76,  /**< A2=1, A1=1, A0=0 */
    TCA9548A_ADDR_A111 = 0x77,  /**< A2=1, A1=1, A0=1 */
} TCA9548A_I2CAddr_t;
 
/* ============================================================
 *  STRUCT ĐỊNH NGHĨA
 * ============================================================ */
 
/**
 * @brief Thông tin của một AS5600 gắn trên kênh TCA9548A
 */
typedef struct {
    TCA9548A_Channel_t  channel;        /**< Kênh mux mà sensor nằm trên */
    bool                enabled;        /**< Kênh này có sensor không? */
    AS5600_Handle_t     sensor;         /**< Handle AS5600 của kênh này */
    char                label[16];      /**< Nhãn tùy chọn (vd: "MOTOR_L") */
} TCA9548A_SensorSlot_t;
 
/**
 * @brief Kết quả đọc dữ liệu từ tất cả cảm biến
 */
typedef struct {
    uint8_t             count;                          /**< Số cảm biến hợp lệ */
    AS5600_Data_t       data[TCA9548A_MAX_SENSORS];     /**< Mảng dữ liệu */
    TCA9548A_Channel_t  channels[TCA9548A_MAX_SENSORS]; /**< Kênh tương ứng */
    TCA9548A_Status_t   status[TCA9548A_MAX_SENSORS];   /**< Trạng thái đọc */
} TCA9548A_ScanResult_t;
 
/**
 * @brief Handle (đối tượng) chính của TCA9548A
 */
typedef struct {
    uint8_t                 i2c_addr;       /**< Địa chỉ I2C của MUX */
    TCA9548A_ChannelMode_t  mode;           /**< Chế độ chọn kênh */
    uint8_t                 active_mask;    /**< Bitmask kênh đang bật */
    TCA9548A_Channel_t      active_channel; /**< Kênh đang chọn (mode single) */
 
    TCA9548A_SensorSlot_t   sensors[TCA9548A_MAX_CHANNELS]; /**< Danh sách sensor */
    uint8_t                 sensor_count;   /**< Số sensor được đăng ký */
 
    /* Con trỏ hàm HAL - người dùng phải cài đặt */
    int8_t (*i2c_write)(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len);
    int8_t (*i2c_read) (uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len);
    void   (*delay_ms) (uint32_t ms);
} TCA9548A_Handle_t;
 
/* ============================================================
 *  API FUNCTIONS — QUẢN LÝ MUX
 * ============================================================ */
 
/**
 * @brief Khởi tạo TCA9548A
 * @param mux   Con trỏ đến handle TCA9548A
 * @param addr  Địa chỉ I2C (TCA9548A_ADDR_A000 ... TCA9548A_ADDR_A111)
 * @return TCA9548A_OK nếu thành công
 */
TCA9548A_Status_t TCA9548A_Init(TCA9548A_Handle_t *mux, TCA9548A_I2CAddr_t addr);
 
/**
 * @brief Reset TCA9548A (tắt tất cả kênh)
 * @param mux   Con trỏ đến handle TCA9548A
 * @return TCA9548A_OK nếu thành công
 */
TCA9548A_Status_t TCA9548A_Reset(TCA9548A_Handle_t *mux);
 
/**
 * @brief Chọn một kênh duy nhất (các kênh còn lại tự động tắt)
 * @param mux   Con trỏ đến handle TCA9548A
 * @param ch    Kênh cần chọn (TCA9548A_CH0 ... TCA9548A_CH7)
 * @return TCA9548A_OK nếu thành công
 */
TCA9548A_Status_t TCA9548A_SelectChannel(TCA9548A_Handle_t *mux, TCA9548A_Channel_t ch);
 
/**
 * @brief Bật/tắt kênh theo bitmask (nhiều kênh cùng lúc)
 * @param mux   Con trỏ đến handle TCA9548A
 * @param mask  Bitmask kênh (bit0=CH0, bit7=CH7); 0x00 = tắt tất cả
 * @return TCA9548A_OK nếu thành công
 */
TCA9548A_Status_t TCA9548A_SetChannelMask(TCA9548A_Handle_t *mux, uint8_t mask);
 
/**
 * @brief Tắt tất cả kênh
 * @param mux   Con trỏ đến handle TCA9548A
 * @return TCA9548A_OK nếu thành công
 */
TCA9548A_Status_t TCA9548A_DisableAll(TCA9548A_Handle_t *mux);
 
/**
 * @brief Đọc trạng thái kênh hiện tại từ chip
 * @param mux   Con trỏ đến handle TCA9548A
 * @param mask  Bitmask trạng thái kênh hiện tại
 * @return TCA9548A_OK nếu thành công
 */
TCA9548A_Status_t TCA9548A_GetChannelMask(TCA9548A_Handle_t *mux, uint8_t *mask);
 
/**
 * @brief Kiểm tra xem kênh có đang bật không
 * @param mux   Con trỏ đến handle TCA9548A
 * @param ch    Kênh cần kiểm tra
 * @param state Trạng thái kênh
 * @return TCA9548A_OK nếu thành công
 */
TCA9548A_Status_t TCA9548A_GetChannelState(TCA9548A_Handle_t *mux,
                                            TCA9548A_Channel_t ch,
                                            TCA9548A_ChannelState_t *state);
 
/* ============================================================
 *  API FUNCTIONS — QUẢN LÝ AS5600 TRÊN MUX
 * ============================================================ */
 
/**
 * @brief Đăng ký một AS5600 trên kênh cụ thể
 * @param mux       Con trỏ đến handle TCA9548A
 * @param ch        Kênh mux mà AS5600 nằm trên
 * @param label     Nhãn tùy chọn (có thể NULL)
 * @return TCA9548A_OK nếu thành công
 *
 * @example:
 *   TCA9548A_RegisterSensor(&mux, TCA9548A_CH0, "LEFT_MOTOR");
 */
TCA9548A_Status_t TCA9548A_RegisterSensor(TCA9548A_Handle_t *mux,
                                           TCA9548A_Channel_t ch,
                                           const char *label);
 
/**
 * @brief Khởi tạo tất cả AS5600 đã đăng ký
 * @param mux   Con trỏ đến handle TCA9548A
 * @return TCA9548A_OK nếu tất cả sensor khởi tạo thành công
 */
TCA9548A_Status_t TCA9548A_InitAllSensors(TCA9548A_Handle_t *mux);
 
/**
 * @brief Đọc dữ liệu từ một AS5600 trên kênh chỉ định
 * @param mux   Con trỏ đến handle TCA9548A
 * @param ch    Kênh cần đọc
 * @param data  Nơi lưu dữ liệu
 * @return TCA9548A_OK nếu thành công
 */
TCA9548A_Status_t TCA9548A_ReadSensor(TCA9548A_Handle_t *mux,
                                       TCA9548A_Channel_t ch,
                                       AS5600_Data_t *data);
 
/**
 * @brief Đọc góc (độ) từ một AS5600 trên kênh chỉ định
 * @param mux       Con trỏ đến handle TCA9548A
 * @param ch        Kênh cần đọc
 * @param degrees   Giá trị góc (0.0 - 360.0)
 * @return TCA9548A_OK nếu thành công
 */
TCA9548A_Status_t TCA9548A_ReadAngleDeg(TCA9548A_Handle_t *mux,
                                          TCA9548A_Channel_t ch,
                                          float *degrees);
 
/**
 * @brief Quét và đọc tất cả AS5600 đã đăng ký
 * @param mux       Con trỏ đến handle TCA9548A
 * @param result    Kết quả đọc từ tất cả sensor
 * @return TCA9548A_OK nếu thành công (ngay cả khi 1 số sensor lỗi)
 */
TCA9548A_Status_t TCA9548A_ScanAllSensors(TCA9548A_Handle_t *mux,
                                            TCA9548A_ScanResult_t *result);
 
/**
 * @brief Lấy con trỏ đến handle AS5600 trên kênh chỉ định
 * @param mux   Con trỏ đến handle TCA9548A
 * @param ch    Kênh cần lấy
 * @return Con trỏ AS5600_Handle_t hoặc NULL nếu không hợp lệ
 */
AS5600_Handle_t *TCA9548A_GetSensorHandle(TCA9548A_Handle_t *mux,
                                            TCA9548A_Channel_t ch);
 
/**
 * @brief Kiểm tra kết nối với chip TCA9548A
 * @param mux   Con trỏ đến handle TCA9548A
 * @return TCA9548A_OK nếu chip phản hồi
 */
TCA9548A_Status_t TCA9548A_IsConnected(TCA9548A_Handle_t *mux);
 
#ifdef __cplusplus
}
#endif


#endif /* INC_TCA9548A_H_ */
