/*
 * my_app.c
 *
 *  Created on: May 14, 2026
 *      Author: Lap4all
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "as5600.h"
#include "i2c.h"
#include "my_app.h"
#include "my_config.h"
#include "tca9548a.h"
#include "tim.h"
#include "usbd_cdc_if.h"

TMC2209_HandleTypeDef motor1;
TMC2209_HandleTypeDef motor2;
TMC2209_HandleTypeDef motor3;

#define MYAPP_SENSOR_CHANNEL             TCA9548A_CH0
#define MYAPP_I2C_TIMEOUT_MS             20u
#define MYAPP_USB_TX_BUFFER_SIZE         192u
#define MYAPP_USB_RX_BUFFER_SIZE         64u
#define MYAPP_ANGLE_TEXT_BUFFER_SIZE     16u
#define MYAPP_AS5600_RAW_STEPS           4096u
#define MYAPP_ANGLE_SCALE_STEPS          3600u
#define MYAPP_FULL_TURN_CDEG             36000l
#define MYAPP_ANGLE_SCALE_CDEG           9u
#define MYAPP_MOTOR_SPEED_RPM            60.0f
#define MYAPP_INPUT_MAX_ABS_CDEG         36000l
#define MYAPP_STEP_SCALE_NUMERATOR       9u
#define MYAPP_STEP_SCALE_DENOMINATOR     2u

typedef enum {
    MYAPP_STATE_WAIT_COMMAND = 0,
    MYAPP_STATE_WAIT_MOTOR,
} MyApp_State_e;

typedef struct {
    int32_t input_angle_cdeg;
    int32_t start_angle_cdeg;
    uint32_t target_steps;
    TMC2209_DirectionTypeDef direction;
} MyApp_MoveContext_t;

extern USBD_HandleTypeDef hUsbDeviceFS;

static TCA9548A_Handle_t s_mux;
static AS5600_Data_t s_as5600_data;
static MyApp_State_e s_app_state = MYAPP_STATE_WAIT_COMMAND;
static MyApp_MoveContext_t s_move_context;
static bool s_is_sensor_ready = false;
static bool s_has_init_error_been_reported = false;
static char s_usb_tx_buffer[MYAPP_USB_TX_BUFFER_SIZE];

static bool MyApp_IsTCA9548AAddress(uint8_t device_address);
static int8_t MyApp_I2C_Write(uint8_t device_address, uint8_t reg, uint8_t *buffer, uint16_t length);
static int8_t MyApp_I2C_Read(uint8_t device_address, uint8_t reg, uint8_t *buffer, uint16_t length);
static void MyApp_DelayMs(uint32_t delay_ms);
static bool MyApp_USB_IsReady(void);
static void MyApp_USB_SendText(const char *text);
static bool MyApp_ParseAngleCdeg(const uint8_t *buffer, uint16_t length, int32_t *angle_cdeg);
static bool MyApp_ReadSensorAngleCdeg(int32_t *angle_cdeg);
static void MyApp_FormatAngleDeg(int32_t angle_cdeg, char *buffer, uint16_t buffer_size);
static int32_t MyApp_CalculateMeasuredDeltaCdeg(int32_t start_angle_cdeg,
                                                 int32_t end_angle_cdeg,
                                                 int32_t input_angle_cdeg);
static TMC2209_DirectionTypeDef MyApp_GetDirectionFromAngle(int32_t angle_cdeg);
static bool MyApp_StartRelativeMove(int32_t input_angle_cdeg);
static void MyApp_ProcessUsbCommand(void);
static void MyApp_ProcessMotorDone(void);

static bool MyApp_IsTCA9548AAddress(uint8_t device_address)
{
    return ((device_address >= TCA9548A_BASE_ADDR) &&
            (device_address <= TCA9548A_ADDR_MAX));
}

static int8_t MyApp_I2C_Write(uint8_t device_address, uint8_t reg, uint8_t *buffer, uint16_t length)
{
    HAL_StatusTypeDef hal_status;

    if (MyApp_IsTCA9548AAddress(device_address) == true) {
        (void)reg;
        hal_status = HAL_I2C_Master_Transmit(&hi2c1,
                                             (uint16_t)(device_address << 1),
                                             buffer,
                                             length,
                                             MYAPP_I2C_TIMEOUT_MS);
    } else {
        hal_status = HAL_I2C_Mem_Write(&hi2c1,
                                       (uint16_t)(device_address << 1),
                                       reg,
                                       I2C_MEMADD_SIZE_8BIT,
                                       buffer,
                                       length,
                                       MYAPP_I2C_TIMEOUT_MS);
    }

    return (hal_status == HAL_OK) ? 0 : -1;
}

static int8_t MyApp_I2C_Read(uint8_t device_address, uint8_t reg, uint8_t *buffer, uint16_t length)
{
    HAL_StatusTypeDef hal_status;

    if (MyApp_IsTCA9548AAddress(device_address) == true) {
        (void)reg;
        hal_status = HAL_I2C_Master_Receive(&hi2c1,
                                            (uint16_t)(device_address << 1),
                                            buffer,
                                            length,
                                            MYAPP_I2C_TIMEOUT_MS);
    } else {
        hal_status = HAL_I2C_Mem_Read(&hi2c1,
                                      (uint16_t)(device_address << 1),
                                      reg,
                                      I2C_MEMADD_SIZE_8BIT,
                                      buffer,
                                      length,
                                      MYAPP_I2C_TIMEOUT_MS);
    }

    return (hal_status == HAL_OK) ? 0 : -1;
}

static void MyApp_DelayMs(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

static bool MyApp_USB_IsReady(void)
{
    USBD_CDC_HandleTypeDef *cdc_handle;

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        return false;
    }

    cdc_handle = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if ((cdc_handle == NULL) || (cdc_handle->TxState != 0u)) {
        return false;
    }

    return true;
}

static void MyApp_USB_SendText(const char *text)
{
    uint16_t text_length = 0u;

    if ((text == NULL) || (MyApp_USB_IsReady() == false)) {
        return;
    }

    while ((text[text_length] != '\0') && (text_length < (MYAPP_USB_TX_BUFFER_SIZE - 1u))) {
        text_length++;
    }

    if (text_length > 0u) {
        (void)CDC_Transmit_FS((uint8_t *)text, text_length);
    }
}

static bool MyApp_ParseAngleCdeg(const uint8_t *buffer, uint16_t length, int32_t *angle_cdeg)
{
    uint16_t index = 0u;
    int32_t sign = 1;
    int32_t whole_deg = 0;
    int32_t frac_cdeg = 0;
    uint8_t frac_digits = 0u;
    bool has_digit = false;

    if ((buffer == NULL) || (angle_cdeg == NULL) || (length == 0u)) {
        return false;
    }

    while ((index < length) && ((buffer[index] == ' ') || (buffer[index] == '\t'))) {
        index++;
    }

    if ((index < length) && ((buffer[index] == '-') || (buffer[index] == '+'))) {
        if (buffer[index] == '-') {
            sign = -1;
        }
        index++;
    }

    while ((index < length) && (buffer[index] >= '0') && (buffer[index] <= '9')) {
        has_digit = true;
        whole_deg = (whole_deg * 10) + (int32_t)(buffer[index] - '0');
        index++;
    }

    if ((index < length) && (buffer[index] == '.')) {
        index++;
        while ((index < length) &&
               (buffer[index] >= '0') &&
               (buffer[index] <= '9') &&
               (frac_digits < 2u)) {
            has_digit = true;
            frac_cdeg = (frac_cdeg * 10) + (int32_t)(buffer[index] - '0');
            frac_digits++;
            index++;
        }
    }

    if (frac_digits == 1u) {
        frac_cdeg *= 10;
    }

    if (has_digit == false) {
        return false;
    }

    *angle_cdeg = sign * ((whole_deg * 100) + frac_cdeg);
    return true;
}

static bool MyApp_ReadSensorAngleCdeg(int32_t *angle_cdeg)
{
    uint16_t angle_4000;

    if (angle_cdeg == NULL) {
        return false;
    }

    if (TCA9548A_ReadSensor(&s_mux, MYAPP_SENSOR_CHANNEL, &s_as5600_data) != TCA9548A_OK) {
        return false;
    }

    angle_4000 = MyApp_ScaleAngleTo4000(s_as5600_data.angle);
    *angle_cdeg = (int32_t)angle_4000 * (int32_t)MYAPP_ANGLE_SCALE_CDEG;
    return true;
}

static void MyApp_FormatAngleDeg(int32_t angle_cdeg, char *buffer, uint16_t buffer_size)
{
    uint32_t angle_abs_cdeg;
    uint32_t angle_whole_deg;
    uint32_t angle_frac_cdeg;

    if ((buffer == NULL) || (buffer_size == 0u)) {
        return;
    }

    angle_abs_cdeg = (angle_cdeg < 0) ? (uint32_t)(-angle_cdeg) : (uint32_t)angle_cdeg;
    angle_whole_deg = angle_abs_cdeg / 100u;
    angle_frac_cdeg = angle_abs_cdeg % 100u;

    if (angle_cdeg < 0) {
        (void)snprintf(buffer,
                       buffer_size,
                       "-%lu.%02lu",
                       (unsigned long)angle_whole_deg,
                       (unsigned long)angle_frac_cdeg);
    } else {
        (void)snprintf(buffer,
                       buffer_size,
                       "%lu.%02lu",
                       (unsigned long)angle_whole_deg,
                       (unsigned long)angle_frac_cdeg);
    }
}

static int32_t MyApp_CalculateMeasuredDeltaCdeg(int32_t start_angle_cdeg,
                                                 int32_t end_angle_cdeg,
                                                 int32_t input_angle_cdeg)
{
    int32_t measured_delta_cdeg = end_angle_cdeg - start_angle_cdeg;

    if ((input_angle_cdeg >= 0) && (measured_delta_cdeg < 0)) {
        measured_delta_cdeg += MYAPP_FULL_TURN_CDEG;
    }

    if ((input_angle_cdeg < 0) && (measured_delta_cdeg > 0)) {
        measured_delta_cdeg -= MYAPP_FULL_TURN_CDEG;
    }

    return measured_delta_cdeg;
}

static TMC2209_DirectionTypeDef MyApp_GetDirectionFromAngle(int32_t angle_cdeg)
{
    return (angle_cdeg >= 0) ? TMC2209_DIR_CW : TMC2209_DIR_CCW;
}

static bool MyApp_StartRelativeMove(int32_t input_angle_cdeg)
{
    TMC2209_StatusTypeDef motor_status;
    uint32_t move_steps;

    if (MyApp_ReadSensorAngleCdeg(&s_move_context.start_angle_cdeg) == false) {
        MyApp_USB_SendText("ERR: cannot read start angle\r\n");
        return false;
    }

    move_steps = MyApp_CalculateMotorStepsFromAngle(input_angle_cdeg);
    s_move_context.input_angle_cdeg = input_angle_cdeg;
    s_move_context.target_steps = move_steps;
    s_move_context.direction = MyApp_GetDirectionFromAngle(input_angle_cdeg);

    if (move_steps == 0u) {
        s_app_state = MYAPP_STATE_WAIT_MOTOR;
        return true;
    }

    motor_status = TMC2209_MoveSteps(&motor1,
                                     move_steps,
                                     s_move_context.direction,
                                     MYAPP_MOTOR_SPEED_RPM);
    if (motor_status != TMC2209_OK) {
        MyApp_USB_SendText("ERR: motor start failed\r\n");
        return false;
    }

    s_app_state = MYAPP_STATE_WAIT_MOTOR;
    return true;
}

static void MyApp_ProcessUsbCommand(void)
{
    uint8_t command_buffer[MYAPP_USB_RX_BUFFER_SIZE];
    uint16_t command_length = 0u;
    int32_t input_angle_cdeg = 0;

    if (CDC_ReadCommand(command_buffer, sizeof(command_buffer), &command_length) == 0u) {
        return;
    }

    if (MyApp_ParseAngleCdeg(command_buffer, command_length, &input_angle_cdeg) == false) {
        MyApp_USB_SendText("ERR: input must be angle in degree, example: 90 or -45.50\r\n");
        return;
    }

    if ((input_angle_cdeg > MYAPP_INPUT_MAX_ABS_CDEG) ||
        (input_angle_cdeg < -MYAPP_INPUT_MAX_ABS_CDEG)) {
        MyApp_USB_SendText("ERR: input range is -360.00..360.00 deg\r\n");
        return;
    }

    (void)MyApp_StartRelativeMove(input_angle_cdeg);
}

static void MyApp_ProcessMotorDone(void)
{
    int32_t end_angle_cdeg = 0;
    int32_t measured_delta_cdeg;
    int32_t error_cdeg;
    int32_t report_length;
    char input_angle_deg_text[MYAPP_ANGLE_TEXT_BUFFER_SIZE];
    char start_angle_deg_text[MYAPP_ANGLE_TEXT_BUFFER_SIZE];
    char end_angle_deg_text[MYAPP_ANGLE_TEXT_BUFFER_SIZE];
    char measured_delta_deg_text[MYAPP_ANGLE_TEXT_BUFFER_SIZE];
    char error_deg_text[MYAPP_ANGLE_TEXT_BUFFER_SIZE];

    if (motor1.state == TMC2209_RUNNING) {
        return;
    }

    if (MyApp_ReadSensorAngleCdeg(&end_angle_cdeg) == false) {
        MyApp_USB_SendText("ERR: cannot read end angle\r\n");
        s_app_state = MYAPP_STATE_WAIT_COMMAND;
        return;
    }

    measured_delta_cdeg = MyApp_CalculateMeasuredDeltaCdeg(s_move_context.start_angle_cdeg,
                                                           end_angle_cdeg,
                                                           s_move_context.input_angle_cdeg);
    error_cdeg = s_move_context.input_angle_cdeg - measured_delta_cdeg;

    MyApp_FormatAngleDeg(s_move_context.input_angle_cdeg,
                         input_angle_deg_text,
                         sizeof(input_angle_deg_text));
    MyApp_FormatAngleDeg(s_move_context.start_angle_cdeg,
                         start_angle_deg_text,
                         sizeof(start_angle_deg_text));
    MyApp_FormatAngleDeg(end_angle_cdeg,
                         end_angle_deg_text,
                         sizeof(end_angle_deg_text));
    MyApp_FormatAngleDeg(measured_delta_cdeg,
                         measured_delta_deg_text,
                         sizeof(measured_delta_deg_text));
    MyApp_FormatAngleDeg(error_cdeg,
                         error_deg_text,
                         sizeof(error_deg_text));

    report_length = snprintf(s_usb_tx_buffer,
                             sizeof(s_usb_tx_buffer),
                             "input_deg=%s,start_deg=%s,end_deg=%s,"
                             "delta_deg=%s,error_deg=%s,steps=%lu\r\n",
                             input_angle_deg_text,
                             start_angle_deg_text,
                             end_angle_deg_text,
                             measured_delta_deg_text,
                             error_deg_text,
                             (unsigned long)s_move_context.target_steps);

    if ((report_length > 0) && (report_length < (int32_t)sizeof(s_usb_tx_buffer))) {
        (void)CDC_Transmit_FS((uint8_t *)s_usb_tx_buffer, (uint16_t)report_length);
    }

    s_app_state = MYAPP_STATE_WAIT_COMMAND;
}

void Motors_Config(void)
{
    motor1.id             = 0u;
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

    motor2.id             = 1u;
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

    motor3.id             = 2u;
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
    Motors_Init();

    s_mux.i2c_write = MyApp_I2C_Write;
    s_mux.i2c_read = MyApp_I2C_Read;
    s_mux.delay_ms = MyApp_DelayMs;
    s_is_sensor_ready = false;
    s_has_init_error_been_reported = false;
    s_app_state = MYAPP_STATE_WAIT_COMMAND;

    if (TCA9548A_Init(&s_mux, TCA9548A_ADDR_A000) != TCA9548A_OK) {
        return;
    }

    if (TCA9548A_RegisterSensor(&s_mux, MYAPP_SENSOR_CHANNEL, "AS5600_CH0") != TCA9548A_OK) {
        return;
    }

    if (TCA9548A_InitAllSensors(&s_mux) != TCA9548A_OK) {
        return;
    }

    s_is_sensor_ready = true;
}

uint16_t MyApp_ScaleAngleTo4000(uint16_t angle_raw)
{
    return (uint16_t)((((uint32_t)angle_raw & 0x0fffu) * MYAPP_ANGLE_SCALE_STEPS) /
                      MYAPP_AS5600_RAW_STEPS);
}

uint32_t MyApp_CalculateMotorStepsFromAngle(int32_t angle_cdeg)
{
    uint32_t angle_abs_cdeg;
    uint32_t steps_per_turn;

    angle_abs_cdeg = (angle_cdeg < 0) ? (uint32_t)(-angle_cdeg) : (uint32_t)angle_cdeg;
    steps_per_turn = (uint32_t)(((uint64_t)motor1.steps_per_rev *
                                 (uint64_t)motor1.microstep *
                                 (uint64_t)MYAPP_STEP_SCALE_NUMERATOR) /
                                (uint64_t)MYAPP_STEP_SCALE_DENOMINATOR);

    return (uint32_t)(((uint64_t)angle_abs_cdeg * steps_per_turn +
                       ((uint64_t)MYAPP_FULL_TURN_CDEG / 2u)) /
                      (uint64_t)MYAPP_FULL_TURN_CDEG);
}

void My_app(void)
{
    if (MyApp_USB_IsReady() == false) {
        return;
    }

    if (s_is_sensor_ready == false) {
        if (s_has_init_error_been_reported == false) {
            MyApp_USB_SendText("ERR: AS5600 init failed\r\n");
            s_has_init_error_been_reported = true;
        }
        return;
    }

    if (s_app_state == MYAPP_STATE_WAIT_COMMAND) {
        MyApp_ProcessUsbCommand();
        return;
    }

    MyApp_ProcessMotorDone();
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == motor1.htim) {
        (void)TMC2209_UpdateSteps(&motor1);
    }
}
