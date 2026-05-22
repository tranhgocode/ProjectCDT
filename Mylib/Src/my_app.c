/*
 * my_app.c
 *
 *  Created on: May 14, 2026
 *      Author: Lap4all
 */

#include "my_app.h"
#include "my_config.h"
#include "i2c.h"
#include "tca9548a.h"
#include "tim.h"

TMC2209_HandleTypeDef motor1;
TMC2209_HandleTypeDef motor2;
TMC2209_HandleTypeDef motor3;

#define MYAPP_I2C_TIMEOUT_MS       20U

static TCA9548A_Handle_t g_mux;

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

void MyApp_SensorUsb_Init(void)
{
    g_mux.i2c_write = MyApp_I2C_Write;
    g_mux.i2c_read = MyApp_I2C_Read;
    g_mux.delay_ms = MyApp_DelayMs;
}
