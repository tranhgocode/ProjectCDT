/*
 * my_app.c
 *
 *  Created on: May 14, 2026
 *      Author: Lap4all
 */

#include "my_app.h"
#include "as5600.h"
#include "i2c.h"
#include "my_config.h"
#include "tca9548a.h"
#include "tim.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

TMC2209_HandleTypeDef motor1;
TMC2209_HandleTypeDef motor2;
TMC2209_HandleTypeDef motor3;

#define MY_APP_SENSOR_CHANNEL              TCA9548A_CH0
#define MY_APP_I2C_TIMEOUT_MS              20u
#define MY_APP_USB_TX_BUFFER_SIZE          192u
#define MY_APP_USB_RX_BUFFER_SIZE          64u
#define MY_APP_ANGLE_TEXT_BUFFER_SIZE      16u
#define MY_APP_AS5600_RAW_STEPS            4096u
#define MY_APP_FULL_TURN_CDEG              36000l
#define MY_APP_MOTOR_SPEED_RPM             60.0f
#define MY_APP_STEP_SCALE_NUMERATOR        9u
#define MY_APP_STEP_SCALE_DENOMINATOR      2u

typedef enum {
    MY_APP_STATE_WAIT_COMMAND = 0,
    MY_APP_STATE_WAIT_MOTOR,
} my_app_state_t;

typedef struct {
    int32_t target_yaw_cdeg;
    int32_t start_yaw_cdeg;
    int32_t move_delta_cdeg;
    int32_t start_sensor_yaw_cdeg;
    uint32_t target_steps;
} my_app_move_context_t;

extern USBD_HandleTypeDef hUsbDeviceFS;

static TCA9548A_Handle_t s_mux;
static AS5600_Data_t s_as5600_data;
static my_app_state_t s_app_state = MY_APP_STATE_WAIT_COMMAND;
static my_app_move_context_t s_move_context;
static bool s_is_sensor_ready = false;
static bool s_has_init_error_been_reported = false;
static int32_t s_sensor_zero_cdeg = 0;
static int32_t s_current_yaw_cdeg = 0;
static char s_usb_tx_buffer[MY_APP_USB_TX_BUFFER_SIZE];

static bool my_app_is_tca9548a_address(uint8_t device_address);
static int8_t my_app_i2c_write(uint8_t device_address,
                               uint8_t reg,
                               uint8_t *buffer,
                               uint16_t length);
static int8_t my_app_i2c_read(uint8_t device_address,
                              uint8_t reg,
                              uint8_t *buffer,
                              uint16_t length);
static void my_app_delay_ms(uint32_t delay_ms);
static bool my_app_usb_is_ready(void);
static void my_app_usb_send_text(const char *text);
static bool my_app_is_zero_command(const uint8_t *buffer, uint16_t length);
static bool my_app_parse_target_angle_cdeg(const uint8_t *buffer,
                                           uint16_t length,
                                           int32_t *target_angle_cdeg);
static int32_t my_app_normalize_sensor_yaw_cdeg(int32_t sensor_angle_cdeg);
static bool my_app_read_sensor_absolute_cdeg(int32_t *sensor_angle_cdeg);
static bool my_app_read_sensor_yaw_cdeg(int32_t *sensor_yaw_cdeg);
static void my_app_format_angle_deg(int32_t angle_cdeg,
                                    char *buffer,
                                    uint16_t buffer_size);
static TMC2209_DirectionTypeDef my_app_get_direction_from_delta(int32_t delta_cdeg);
static int32_t my_app_calculate_sensor_delta_cdeg(int32_t start_cdeg,
                                                  int32_t end_cdeg,
                                                  int32_t expected_delta_cdeg);
static bool my_app_set_zero_from_sensor(void);
static bool my_app_start_target_move(int32_t target_yaw_cdeg);
static void my_app_process_usb_command(void);
static void my_app_process_motor_done(void);

static bool my_app_is_tca9548a_address(uint8_t device_address)
{
    return ((device_address >= TCA9548A_BASE_ADDR) &&
            (device_address <= TCA9548A_ADDR_MAX));
}

static int8_t my_app_i2c_write(uint8_t device_address,
                               uint8_t reg,
                               uint8_t *buffer,
                               uint16_t length)
{
    HAL_StatusTypeDef hal_status;

    if (my_app_is_tca9548a_address(device_address) == true) {
        (void)reg;
        hal_status = HAL_I2C_Master_Transmit(&hi2c1,
                                             (uint16_t)(device_address << 1),
                                             buffer,
                                             length,
                                             MY_APP_I2C_TIMEOUT_MS);
    } else {
        hal_status = HAL_I2C_Mem_Write(&hi2c1,
                                       (uint16_t)(device_address << 1),
                                       reg,
                                       I2C_MEMADD_SIZE_8BIT,
                                       buffer,
                                       length,
                                       MY_APP_I2C_TIMEOUT_MS);
    }

    return (hal_status == HAL_OK) ? 0 : -1;
}

static int8_t my_app_i2c_read(uint8_t device_address,
                              uint8_t reg,
                              uint8_t *buffer,
                              uint16_t length)
{
    HAL_StatusTypeDef hal_status;

    if (my_app_is_tca9548a_address(device_address) == true) {
        (void)reg;
        hal_status = HAL_I2C_Master_Receive(&hi2c1,
                                            (uint16_t)(device_address << 1),
                                            buffer,
                                            length,
                                            MY_APP_I2C_TIMEOUT_MS);
    } else {
        hal_status = HAL_I2C_Mem_Read(&hi2c1,
                                      (uint16_t)(device_address << 1),
                                      reg,
                                      I2C_MEMADD_SIZE_8BIT,
                                      buffer,
                                      length,
                                      MY_APP_I2C_TIMEOUT_MS);
    }

    return (hal_status == HAL_OK) ? 0 : -1;
}

static void my_app_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

static bool my_app_usb_is_ready(void)
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

static void my_app_usb_send_text(const char *text)
{
    uint16_t text_length = 0u;

    if ((text == NULL) || (my_app_usb_is_ready() == false)) {
        return;
    }

    while ((text[text_length] != '\0') &&
           (text_length < (MY_APP_USB_TX_BUFFER_SIZE - 1u))) {
        text_length++;
    }

    if (text_length > 0u) {
        (void)CDC_Transmit_FS((uint8_t *)text, text_length);
    }
}

static bool my_app_is_zero_command(const uint8_t *buffer, uint16_t length)
{
    uint16_t start_index = 0u;
    uint16_t end_index = length;

    while ((start_index < length) &&
           ((buffer[start_index] == ' ') ||
            (buffer[start_index] == '\t') ||
            (buffer[start_index] == '\r') ||
            (buffer[start_index] == '\n'))) {
        start_index++;
    }

    while ((end_index > start_index) &&
           ((buffer[end_index - 1u] == ' ') ||
            (buffer[end_index - 1u] == '\t') ||
            (buffer[end_index - 1u] == '\r') ||
            (buffer[end_index - 1u] == '\n'))) {
        end_index--;
    }

    if ((end_index - start_index) != 4u) {
        return false;
    }

    return (((buffer[start_index] == 'z') || (buffer[start_index] == 'Z')) &&
            ((buffer[start_index + 1u] == 'e') || (buffer[start_index + 1u] == 'E')) &&
            ((buffer[start_index + 2u] == 'r') || (buffer[start_index + 2u] == 'R')) &&
            ((buffer[start_index + 3u] == 'o') || (buffer[start_index + 3u] == 'O')));
}

static bool my_app_parse_target_angle_cdeg(const uint8_t *buffer,
                                           uint16_t length,
                                           int32_t *target_angle_cdeg)
{
    uint16_t index = 0u;
    int32_t whole_deg = 0;
    int32_t frac_cdeg = 0;
    uint8_t frac_digits = 0u;
    bool has_digit = false;

    if ((buffer == NULL) || (target_angle_cdeg == NULL) || (length == 0u)) {
        return false;
    }

    while ((index < length) && ((buffer[index] == ' ') || (buffer[index] == '\t'))) {
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

    while ((index < length) &&
           ((buffer[index] == ' ') ||
            (buffer[index] == '\t') ||
            (buffer[index] == '\r') ||
            (buffer[index] == '\n'))) {
        index++;
    }

    if ((has_digit == false) || (index != length)) {
        return false;
    }

    *target_angle_cdeg = (whole_deg * 100) + frac_cdeg;
    return true;
}

static int32_t my_app_normalize_sensor_yaw_cdeg(int32_t sensor_angle_cdeg)
{
    int32_t yaw_cdeg = sensor_angle_cdeg - s_sensor_zero_cdeg;

    while (yaw_cdeg < 0) {
        yaw_cdeg += MY_APP_FULL_TURN_CDEG;
    }

    while (yaw_cdeg >= MY_APP_FULL_TURN_CDEG) {
        yaw_cdeg -= MY_APP_FULL_TURN_CDEG;
    }

    return yaw_cdeg;
}

static bool my_app_read_sensor_absolute_cdeg(int32_t *sensor_angle_cdeg)
{
    if (sensor_angle_cdeg == NULL) {
        return false;
    }

    if (TCA9548A_ReadSensor(&s_mux, MY_APP_SENSOR_CHANNEL, &s_as5600_data) != TCA9548A_OK) {
        return false;
    }

    *sensor_angle_cdeg = (int32_t)(((uint64_t)s_as5600_data.angle *
                                    (uint64_t)MY_APP_FULL_TURN_CDEG) /
                                   (uint64_t)MY_APP_AS5600_RAW_STEPS);
    return true;
}

static bool my_app_read_sensor_yaw_cdeg(int32_t *sensor_yaw_cdeg)
{
    int32_t sensor_angle_cdeg = 0;

    if (sensor_yaw_cdeg == NULL) {
        return false;
    }

    if (my_app_read_sensor_absolute_cdeg(&sensor_angle_cdeg) == false) {
        return false;
    }

    *sensor_yaw_cdeg = my_app_normalize_sensor_yaw_cdeg(sensor_angle_cdeg);
    return true;
}

static void my_app_format_angle_deg(int32_t angle_cdeg,
                                    char *buffer,
                                    uint16_t buffer_size)
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

static TMC2209_DirectionTypeDef my_app_get_direction_from_delta(int32_t delta_cdeg)
{
    return (delta_cdeg >= 0) ? TMC2209_DIR_CW : TMC2209_DIR_CCW;
}

static int32_t my_app_calculate_sensor_delta_cdeg(int32_t start_cdeg,
                                                  int32_t end_cdeg,
                                                  int32_t expected_delta_cdeg)
{
    int32_t sensor_delta_cdeg = end_cdeg - start_cdeg;

    if ((expected_delta_cdeg >= 0) && (sensor_delta_cdeg < 0)) {
        sensor_delta_cdeg += MY_APP_FULL_TURN_CDEG;
    }

    if ((expected_delta_cdeg < 0) && (sensor_delta_cdeg > 0)) {
        sensor_delta_cdeg -= MY_APP_FULL_TURN_CDEG;
    }

    return sensor_delta_cdeg;
}

static bool my_app_set_zero_from_sensor(void)
{
    int32_t sensor_angle_cdeg = 0;

    if (my_app_read_sensor_absolute_cdeg(&sensor_angle_cdeg) == false) {
        my_app_usb_send_text("ERR: cannot read sensor for zero\r\n");
        return false;
    }

    s_sensor_zero_cdeg = sensor_angle_cdeg;
    s_current_yaw_cdeg = 0;
    s_app_state = MY_APP_STATE_WAIT_COMMAND;
    my_app_usb_send_text("zero_ok yaw_deg=0.00\r\n");
    return true;
}

static bool my_app_start_target_move(int32_t target_yaw_cdeg)
{
    TMC2209_StatusTypeDef motor_status;
    int32_t move_delta_cdeg;
    uint32_t move_steps;

    if (my_app_read_sensor_yaw_cdeg(&s_move_context.start_sensor_yaw_cdeg) == false) {
        my_app_usb_send_text("ERR: cannot read start angle\r\n");
        return false;
    }

    move_delta_cdeg = target_yaw_cdeg - s_current_yaw_cdeg;
    move_steps = my_app_calculate_motor_steps_from_angle(move_delta_cdeg);

    s_move_context.target_yaw_cdeg = target_yaw_cdeg;
    s_move_context.start_yaw_cdeg = s_current_yaw_cdeg;
    s_move_context.move_delta_cdeg = move_delta_cdeg;
    s_move_context.target_steps = move_steps;

    if (move_steps == 0u) {
        s_app_state = MY_APP_STATE_WAIT_MOTOR;
        return true;
    }

    motor_status = TMC2209_MoveSteps(&motor1,
                                     move_steps,
                                     my_app_get_direction_from_delta(move_delta_cdeg),
                                     MY_APP_MOTOR_SPEED_RPM);
    if (motor_status != TMC2209_OK) {
        my_app_usb_send_text("ERR: motor start failed\r\n");
        return false;
    }

    s_app_state = MY_APP_STATE_WAIT_MOTOR;
    return true;
}

static void my_app_process_usb_command(void)
{
    uint8_t command_buffer[MY_APP_USB_RX_BUFFER_SIZE];
    uint16_t command_length = 0u;
    int32_t target_yaw_cdeg = 0;

    if (CDC_ReadCommand(command_buffer, sizeof(command_buffer), &command_length) == 0u) {
        return;
    }

    if (my_app_is_zero_command(command_buffer, command_length) == true) {
        (void)my_app_set_zero_from_sensor();
        return;
    }

    if (my_app_parse_target_angle_cdeg(command_buffer,
                                       command_length,
                                       &target_yaw_cdeg) == false) {
        my_app_usb_send_text("ERR: input must be zero or angle 0.00..360.00 deg\r\n");
        return;
    }

    if ((target_yaw_cdeg < 0) || (target_yaw_cdeg > MY_APP_FULL_TURN_CDEG)) {
        my_app_usb_send_text("ERR: input range is 0.00..360.00 deg\r\n");
        return;
    }

    (void)my_app_start_target_move(target_yaw_cdeg);
}

static void my_app_process_motor_done(void)
{
    int32_t end_sensor_yaw_cdeg = 0;
    int32_t sensor_delta_cdeg;
    int32_t error_cdeg;
    int32_t report_length;
    char input_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char start_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char end_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char delta_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];
    char error_angle_text[MY_APP_ANGLE_TEXT_BUFFER_SIZE];

    if (motor1.state == TMC2209_RUNNING) {
        return;
    }

    if (my_app_read_sensor_yaw_cdeg(&end_sensor_yaw_cdeg) == false) {
        my_app_usb_send_text("ERR: cannot read end angle\r\n");
        s_app_state = MY_APP_STATE_WAIT_COMMAND;
        return;
    }

    sensor_delta_cdeg = my_app_calculate_sensor_delta_cdeg(
        s_move_context.start_sensor_yaw_cdeg,
        end_sensor_yaw_cdeg,
        s_move_context.move_delta_cdeg);
    error_cdeg = s_move_context.move_delta_cdeg - sensor_delta_cdeg;
    s_current_yaw_cdeg = s_move_context.target_yaw_cdeg;

    my_app_format_angle_deg(s_move_context.target_yaw_cdeg,
                            input_angle_text,
                            sizeof(input_angle_text));
    my_app_format_angle_deg(s_move_context.start_sensor_yaw_cdeg,
                            start_angle_text,
                            sizeof(start_angle_text));
    my_app_format_angle_deg(end_sensor_yaw_cdeg,
                            end_angle_text,
                            sizeof(end_angle_text));
    my_app_format_angle_deg(sensor_delta_cdeg,
                            delta_angle_text,
                            sizeof(delta_angle_text));
    my_app_format_angle_deg(error_cdeg,
                            error_angle_text,
                            sizeof(error_angle_text));

    report_length = snprintf(s_usb_tx_buffer,
                             sizeof(s_usb_tx_buffer),
                             "input_deg=%s,start_deg=%s,end_deg=%s,"
                             "delta_deg=%s,error_deg=%s,steps=%lu\r\n",
                             input_angle_text,
                             start_angle_text,
                             end_angle_text,
                             delta_angle_text,
                             error_angle_text,
                             (unsigned long)s_move_context.target_steps);

    if ((report_length > 0) && (report_length < (int32_t)sizeof(s_usb_tx_buffer))) {
        (void)CDC_Transmit_FS((uint8_t *)s_usb_tx_buffer, (uint16_t)report_length);
    }

    s_app_state = MY_APP_STATE_WAIT_COMMAND;
}

void motors_config(void)
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

void motors_init(void)
{
    motors_config();

    (void)TMC2209_Init(&motor1);
    (void)TMC2209_Init(&motor2);
    (void)TMC2209_Init(&motor3);

    (void)TMC2209_SetStealthChop(&motor1, true);
    (void)TMC2209_SetStealthChop(&motor2, true);
    (void)TMC2209_SetStealthChop(&motor3, true);
}

void my_app_init(void)
{
    motors_init();

    s_mux.i2c_write = my_app_i2c_write;
    s_mux.i2c_read = my_app_i2c_read;
    s_mux.delay_ms = my_app_delay_ms;
    s_is_sensor_ready = false;
    s_has_init_error_been_reported = false;
    s_app_state = MY_APP_STATE_WAIT_COMMAND;
    s_sensor_zero_cdeg = 0;
    s_current_yaw_cdeg = 0;

    if (TCA9548A_Init(&s_mux, TCA9548A_ADDR_A000) != TCA9548A_OK) {
        return;
    }

    if (TCA9548A_RegisterSensor(&s_mux, MY_APP_SENSOR_CHANNEL, "AS5600_CH0") != TCA9548A_OK) {
        return;
    }

    if (TCA9548A_InitAllSensors(&s_mux) != TCA9548A_OK) {
        return;
    }

    s_is_sensor_ready = true;
    (void)my_app_set_zero_from_sensor();
}

uint32_t my_app_calculate_motor_steps_from_angle(int32_t angle_cdeg)
{
    uint32_t angle_abs_cdeg;
    uint32_t scaled_steps_per_turn;

    angle_abs_cdeg = (angle_cdeg < 0) ? (uint32_t)(-angle_cdeg) : (uint32_t)angle_cdeg;
    scaled_steps_per_turn = (uint32_t)((uint64_t)motor1.steps_per_rev * (uint64_t)motor1.microstep); // * (uint64_t)MY_APP_STEP_SCALE_NUMERATOR) / (uint64_t)MY_APP_STEP_SCALE_DENOMINATOR

    return (uint32_t)(((uint64_t)angle_abs_cdeg * scaled_steps_per_turn + ((uint64_t)MY_APP_FULL_TURN_CDEG / 2u)) / (uint64_t)MY_APP_FULL_TURN_CDEG);
}

void my_app_process(void)
{
    if (my_app_usb_is_ready() == false) {
        return;
    }

    if (s_is_sensor_ready == false) {
        if (s_has_init_error_been_reported == false) {
            my_app_usb_send_text("ERR: AS5600 init failed\r\n");
            s_has_init_error_been_reported = true;
        }
        return;
    }

    if (s_app_state == MY_APP_STATE_WAIT_COMMAND) {
        my_app_process_usb_command();
        return;
    }

    my_app_process_motor_done();
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == motor1.htim) {
        (void)TMC2209_UpdateSteps(&motor1);
    }
}
