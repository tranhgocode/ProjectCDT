/*
 * my_app.c
 *
 *  Created on: May 14, 2026
 *      Author: Lap4all
 */

#include "my_app.h"
#include "my_config.h"
#include "as5600.h"
#include "i2c.h"
#include "tca9548a.h"
#include "tim.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

TMC2209_HandleTypeDef motor1;
TMC2209_HandleTypeDef motor2;
TMC2209_HandleTypeDef motor3;

#define MYAPP_SENSOR_CHANNEL       TCA9548A_CH0
#define MYAPP_USB_SEND_PERIOD_MS   200U
#define MYAPP_SENSOR_START_DELAY_MS 1500U
#define MYAPP_I2C_TIMEOUT_MS       20U
#define MYAPP_USB_TX_BUF_SIZE      128U

typedef enum {
    MYAPP_SENSOR_USB_WAIT = 0,
    MYAPP_SENSOR_USB_RUN,
    MYAPP_SENSOR_USB_ERROR
} MyApp_SensorUsbState_t;

extern USBD_HandleTypeDef hUsbDeviceFS;

static TCA9548A_Handle_t g_mux;
static AS5600_Data_t g_as5600_data;
static MyApp_SensorUsbState_t g_sensor_usb_state = MYAPP_SENSOR_USB_WAIT;
static uint32_t g_sensor_usb_tick = 0U;
static uint32_t g_sensor_usb_start_tick = 0U;
static char g_usb_tx_buf[MYAPP_USB_TX_BUF_SIZE];

static uint8_t MyApp_IsTCA9548AAddress(uint8_t dev_addr)
{
    return ((dev_addr >= TCA9548A_BASE_ADDR) && (dev_addr <= TCA9548A_ADDR_MAX)) ? 1U : 0U;
}

static int8_t MyApp_I2C_Write(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef ret;
    (void)reg;

    if (MyApp_IsTCA9548AAddress(dev_addr) != 0U) {
        ret = HAL_I2C_Master_Transmit(&hi2c1,
                                      (uint16_t)(dev_addr << 1),
                                      buf,
                                      len,
                                      MYAPP_I2C_TIMEOUT_MS);
    } else {
        ret = HAL_I2C_Mem_Write(&hi2c1,
                                (uint16_t)(dev_addr << 1),
                                reg,
                                I2C_MEMADD_SIZE_8BIT,
                                buf,
                                len,
                                MYAPP_I2C_TIMEOUT_MS);
    }

    return (ret == HAL_OK) ? 0 : -1;
}

static int8_t MyApp_I2C_Read(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef ret;

    if (MyApp_IsTCA9548AAddress(dev_addr) != 0U) {
        (void)reg;
        ret = HAL_I2C_Master_Receive(&hi2c1,
                                     (uint16_t)(dev_addr << 1),
                                     buf,
                                     len,
                                     MYAPP_I2C_TIMEOUT_MS);
    } else {
        ret = HAL_I2C_Mem_Read(&hi2c1,
                               (uint16_t)(dev_addr << 1),
                               reg,
                               I2C_MEMADD_SIZE_8BIT,
                               buf,
                               len,
                               MYAPP_I2C_TIMEOUT_MS);
    }

    return (ret == HAL_OK) ? 0 : -1;
}

static void MyApp_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

static uint8_t MyApp_USB_IsReady(void)
{
    USBD_CDC_HandleTypeDef *hcdc;

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        return 0U;
    }

    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if ((hcdc == NULL) || (hcdc->TxState != 0U)) {
        return 0U;
    }

    return 1U;
}

static uint8_t MyApp_USB_SendLine(const char *text)
{
    size_t len;

    if (MyApp_USB_IsReady() == 0U) {
        return USBD_BUSY;
    }

    len = strlen(text);
    if (len >= sizeof(g_usb_tx_buf)) {
        len = sizeof(g_usb_tx_buf) - 1U;
    }

    (void)memcpy(g_usb_tx_buf, text, len);
    g_usb_tx_buf[len] = '\0';

    return CDC_Transmit_FS((uint8_t *)g_usb_tx_buf, (uint16_t)len);
}

static const char *MyApp_AS5600_MagnetText(AS5600_MagnetStatus_t magnet)
{
    switch (magnet) {
    case AS5600_MAG_DETECTED:
        return "OK";
    case AS5600_MAG_TOO_WEAK:
        return "WEAK";
    case AS5600_MAG_TOO_STRONG:
        return "STRONG";
    case AS5600_MAG_NOT_DETECTED:
    default:
        return "NO_MAGNET";
    }
}

static void MyApp_USB_SendAS5600Data(const AS5600_Data_t *data)
{
    uint32_t angle_cdeg = ((uint32_t)data->angle * 36000UL) / 4096UL;

    if (MyApp_USB_IsReady() == 0U) {
        return;
    }

    int len = snprintf(g_usb_tx_buf,
                       sizeof(g_usb_tx_buf),
                       "CH0 raw_angle=%u angle_raw=%u angle=%lu.%02lu deg agc=%u mag=%u magnet=%s\r\n",
                       data->raw_angle,
                       data->angle,
                       angle_cdeg / 100UL,
                       angle_cdeg % 100UL,
                       data->agc,
                       data->magnitude,
                       MyApp_AS5600_MagnetText(data->magnet));

    if ((len > 0) && (len < (int)sizeof(g_usb_tx_buf))) {
        (void)CDC_Transmit_FS((uint8_t *)g_usb_tx_buf, (uint16_t)len);
    }
}

static uint8_t MyApp_SensorUsb_StartSensor(void)
{
    if (TCA9548A_Init(&g_mux, TCA9548A_ADDR_A000) != TCA9548A_OK) {
        (void)MyApp_USB_SendLine("TCA9548A init failed\r\n");
        return 0U;
    }

    if (TCA9548A_RegisterSensor(&g_mux, MYAPP_SENSOR_CHANNEL, "AS5600_CH0") != TCA9548A_OK) {
        (void)MyApp_USB_SendLine("AS5600 CH0 register failed\r\n");
        return 0U;
    }

    if (TCA9548A_InitAllSensors(&g_mux) != TCA9548A_OK) {
        (void)MyApp_USB_SendLine("AS5600 CH0 init failed\r\n");
        return 0U;
    }

    (void)MyApp_USB_SendLine("AS5600 CH0 USB CDC started\r\n");
    return 1U;
}

void Motors_Config(void)
{
    motor1.id             = 0U;
    motor1.htim           = &htim2;
    motor1.tim_channel    = TIM_CHANNEL_1;
    motor1.dir_port       = DIR_1_GPIO_Port;
    motor1.dir_pin        = DIR_1_Pin;
    motor1.en_port        = EN_1_GPIO_Port;
    motor1.en_pin         = EN_1_Pin;
    motor1.timer_clock_hz = TMC2209_TIMER_CLOCK_HZ;
    motor1.prescaler      = TMC2209_TIMER_PRESCALER;
    motor1.steps_per_rev  = TMC2209_MOTOR_STEPS_REV;
    motor1.microstep      = TMC2209_MICROSTEP_16;
    motor1.slave_address  = TMC2209_SLAVE_ADD1;

    motor2.id             = 1U;
    motor2.htim           = &htim3;
    motor2.tim_channel    = TIM_CHANNEL_1;
    motor2.dir_port       = DIR_2_GPIO_Port;
    motor2.dir_pin        = DIR_2_Pin;
    motor2.en_port        = EN_2_GPIO_Port;
    motor2.en_pin         = EN_2_Pin;
    motor2.timer_clock_hz = TMC2209_TIMER_CLOCK_HZ;
    motor2.prescaler      = TMC2209_TIMER_PRESCALER;
    motor2.steps_per_rev  = TMC2209_MOTOR_STEPS_REV;
    motor2.microstep      = TMC2209_MICROSTEP_16;
    motor2.slave_address  = TMC2209_SLAVE_ADD2;

    motor3.id             = 2U;
    motor3.htim           = &htim4;
    motor3.tim_channel    = TIM_CHANNEL_1;
    motor3.dir_port       = DIR_3_GPIO_Port;
    motor3.dir_pin        = DIR_3_Pin;
    motor3.en_port        = EN_3_GPIO_Port;
    motor3.en_pin         = EN_3_Pin;
    motor3.timer_clock_hz = TMC2209_TIMER_CLOCK_HZ;
    motor3.prescaler      = TMC2209_TIMER_PRESCALER;
    motor3.steps_per_rev  = TMC2209_MOTOR_STEPS_REV;
    motor3.microstep      = TMC2209_MICROSTEP_16;
    motor3.slave_address  = TMC2209_SLAVE_ADD3;
}

void Motors_Init(void)
{
    Motors_Config();

    (void)TMC2209_Init(&motor1);
    (void)TMC2209_Init(&motor2);
    (void)TMC2209_Init(&motor3);

    (void)TMC2209_SetStealthChop(&motor1, true);
    (void)TMC2209_SetStealthChop(&motor2, true);
    (void)TMC2209_SetStealthChop(&motor3, true);
}

void MyApp_TestStep1(void)
{
    Motors_Config();

    (void)TMC2209_Init(&motor1);
    (void)TMC2209_SetStealthChop(&motor1, true);

    TMC2209_Enable(&motor1);
    TMC2209_SetDirection(&motor1, TMC2209_DIR_CW);
    (void)TMC2209_SetSpeedRPM(&motor1, 60.0f);
    (void)TMC2209_Start(&motor1);

    HAL_Delay(2000U);

    TMC2209_Stop(&motor1);
    HAL_Delay(500U);

    TMC2209_SetDirection(&motor1, TMC2209_DIR_CCW);
    (void)TMC2209_SetSpeedRPM(&motor1, 60.0f);
    (void)TMC2209_Start(&motor1);

    HAL_Delay(2000U);

    TMC2209_Stop(&motor1);
    TMC2209_Disable(&motor1);
}

void MyApp_SensorUsb_Init(void)
{
    g_mux.i2c_write = MyApp_I2C_Write;
    g_mux.i2c_read = MyApp_I2C_Read;
    g_mux.delay_ms = MyApp_DelayMs;

    g_sensor_usb_state = MYAPP_SENSOR_USB_WAIT;
    g_sensor_usb_tick = HAL_GetTick();
    g_sensor_usb_start_tick = g_sensor_usb_tick;
}

void MyApp_SensorUsb_Task(void)
{
    const uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - g_sensor_usb_tick) < MYAPP_USB_SEND_PERIOD_MS) {
        return;
    }
    g_sensor_usb_tick = now;

    if (MyApp_USB_IsReady() == 0U) {
        return;
    }

    if (g_sensor_usb_state == MYAPP_SENSOR_USB_WAIT) {
        if ((uint32_t)(now - g_sensor_usb_start_tick) < MYAPP_SENSOR_START_DELAY_MS) {
            return;
        }

        if (MyApp_SensorUsb_StartSensor() != 0U) {
            g_sensor_usb_state = MYAPP_SENSOR_USB_RUN;
        } else {
            g_sensor_usb_state = MYAPP_SENSOR_USB_ERROR;
        }
        return;
    }

    if (g_sensor_usb_state == MYAPP_SENSOR_USB_ERROR) {
        return;
    }

    if (TCA9548A_ReadSensor(&g_mux, MYAPP_SENSOR_CHANNEL, &g_as5600_data) != TCA9548A_OK) {
        (void)MyApp_USB_SendLine("Read AS5600 CH0 failed\r\n");
        return;
    }

    MyApp_USB_SendAS5600Data(&g_as5600_data);
}
