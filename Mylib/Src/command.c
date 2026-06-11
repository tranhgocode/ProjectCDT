/**
 * @file    command.c
 * @brief   Phân tích cú pháp lệnh và giao tiếp USB CDC.
 * @author  Lap4all
 * @date    2026-06-10
 */

#include "command.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include <stdbool.h>
#include <stdint.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

/* ============================================================================
 * String builder helpers
 * ============================================================================ */

void Command_AppendChar(char *buffer, uint16_t buffer_size, uint16_t *index,
                        char value)
{
    if ((buffer == NULL) || (index == NULL) || (buffer_size == 0U))
    {
        return;
    }
    if (*index < (uint16_t)(buffer_size - 1U))
    {
        buffer[*index] = value;
        (*index)++;
        buffer[*index] = '\0';
    }
}

void Command_AppendText(char *buffer, uint16_t buffer_size, uint16_t *index,
                        const char *text)
{
    uint16_t i = 0U;
    if (text == NULL) { return; }
    while (text[i] != '\0')
    {
        Command_AppendChar(buffer, buffer_size, index, text[i]);
        i++;
    }
}

void Command_AppendUnsigned(char *buffer, uint16_t buffer_size, uint16_t *index,
                            uint32_t value)
{
    char    digits[10];
    uint8_t n = 0U;
    do
    {
        digits[n++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value > 0U) && (n < sizeof(digits)));
    while (n > 0U)
    {
        n--;
        Command_AppendChar(buffer, buffer_size, index, digits[n]);
    }
}

void Command_AppendSigned(char *buffer, uint16_t buffer_size, uint16_t *index,
                          int32_t value)
{
    uint32_t abs_val;
    if (value < 0)
    {
        Command_AppendChar(buffer, buffer_size, index, '-');
        abs_val = (uint32_t)(-value);
    }
    else
    {
        abs_val = (uint32_t)value;
    }
    Command_AppendUnsigned(buffer, buffer_size, index, abs_val);
}

void Command_AppendAngleDeg(int32_t angle_cdeg, char *buffer,
                            uint16_t buffer_size, uint16_t *index)
{
    uint32_t abs_cdeg;
    uint32_t whole;
    uint32_t frac;

    if ((buffer == NULL) || (index == NULL) || (buffer_size == 0U)) { return; }

    abs_cdeg = (angle_cdeg < 0) ? (uint32_t)(-angle_cdeg) : (uint32_t)angle_cdeg;
    whole    = abs_cdeg / 100U;
    frac     = abs_cdeg % 100U;

    if (angle_cdeg < 0) { Command_AppendChar(buffer, buffer_size, index, '-'); }
    Command_AppendUnsigned(buffer, buffer_size, index, whole);
    Command_AppendChar(buffer, buffer_size, index, '.');
    Command_AppendChar(buffer, buffer_size, index, (char)('0' + ((frac / 10U) % 10U)));
    Command_AppendChar(buffer, buffer_size, index, (char)('0' + (frac % 10U)));
}

/* ============================================================================
 * USB RX ring buffer
 * ============================================================================
 *
 * Single-producer (USB ISR via USB_RX_PushBytes) /
 * single-consumer (main loop via USB_RX_GetLine) lock-free ring buffer.
 * s_rx_tail is written only by the producer; s_rx_head only by the consumer.
 * volatile ensures the compiler does not cache either index across calls.
 */

static uint8_t           s_rx_ring[USB_RX_LINE_BUFFER_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

void USB_RX_PushBytes(uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint16_t next;

    if (data == NULL) { return; }

    for (i = 0U; i < len; i++)
    {
        next = (uint16_t)((s_rx_tail + 1U) % USB_RX_LINE_BUFFER_SIZE);
        if (next != s_rx_head)
        {
            s_rx_ring[s_rx_tail] = data[i];
            s_rx_tail = next;
        }
        /* else: buffer full — byte dropped */
    }
}

int USB_RX_GetLine(char *line, uint16_t max_len)
{
    uint16_t tail;
    uint16_t pos;
    uint16_t out;
    uint16_t i;

    if ((line == NULL) || (max_len < 2U)) { return 0; }

    tail = s_rx_tail; /* snapshot so the ISR can keep writing safely */

    /* Search for '\n' from head to tail. */
    pos = s_rx_head;
    while (pos != tail)
    {
        if (s_rx_ring[pos] == (uint8_t)'\n')
        {
            /* Copy bytes from head to pos, strip '\r'. */
            out = 0U;
            i   = s_rx_head;
            while (i != pos)
            {
                char c = (char)s_rx_ring[i];
                if ((c != '\r') && (out < (uint16_t)(max_len - 1U)))
                {
                    line[out] = c;
                    out++;
                }
                i = (uint16_t)((i + 1U) % USB_RX_LINE_BUFFER_SIZE);
            }
            line[out] = '\0';

            /* Advance head past the '\n'. */
            s_rx_head = (uint16_t)((pos + 1U) % USB_RX_LINE_BUFFER_SIZE);
            return 1;
        }
        pos = (uint16_t)((pos + 1U) % USB_RX_LINE_BUFFER_SIZE);
    }

    return 0;
}

/* ============================================================================
 * USB CDC wrappers
 * ============================================================================ */

bool Command_UsbIsReady(void)
{
    USBD_CDC_HandleTypeDef *hcdc;
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) { return false; }
    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if ((hcdc == NULL) || (hcdc->TxState != 0U)) { return false; }
    return true;
}

void Command_UsbSendText(const char *text)
{
    uint16_t len = 0U;
    if ((text == NULL) || (Command_UsbIsReady() == false)) { return; }
    while ((text[len] != '\0') && (len < (COMMAND_USB_TX_BUFFER_SIZE - 1U))) { len++; }
    if (len > 0U) { (void)CDC_Transmit_FS((uint8_t *)text, len); }
}

void Command_UsbSendBuffer(uint8_t *buf, uint16_t length)
{
    if ((buf == NULL) || (length == 0U) || (Command_UsbIsReady() == false)) { return; }
    (void)CDC_Transmit_FS(buf, length);
}

/* ============================================================================
 * Command recognition
 * ============================================================================ */

static void command_trim(const uint8_t *buffer, uint16_t length,
                         uint16_t *s, uint16_t *e)
{
    *s = 0U;
    *e = length;
    while ((*s < length) &&
           ((buffer[*s] == ' ') || (buffer[*s] == '\t') ||
            (buffer[*s] == '\r') || (buffer[*s] == '\n'))) { (*s)++; }
    while ((*e > *s) &&
           ((buffer[*e - 1U] == ' ') || (buffer[*e - 1U] == '\t') ||
            (buffer[*e - 1U] == '\r') || (buffer[*e - 1U] == '\n'))) { (*e)--; }
}

bool Command_IsZeroCommand(const uint8_t *buffer, uint16_t length)
{
    uint16_t s, e;
    if ((buffer == NULL) || (length == 0U)) { return false; }
    command_trim(buffer, length, &s, &e);
    if ((e - s) != 4U) { return false; }
    return (((buffer[s]      == 'z') || (buffer[s]      == 'Z')) &&
            ((buffer[s + 1U] == 'e') || (buffer[s + 1U] == 'E')) &&
            ((buffer[s + 2U] == 'r') || (buffer[s + 2U] == 'R')) &&
            ((buffer[s + 3U] == 'o') || (buffer[s + 3U] == 'O')));
}

bool Command_IsGoCommand(const uint8_t *buffer, uint16_t length)
{
    uint16_t s, e;
    if ((buffer == NULL) || (length == 0U)) { return false; }
    command_trim(buffer, length, &s, &e);
    if ((e - s) != 2U) { return false; }
    return (((buffer[s]      == 'g') || (buffer[s]      == 'G')) &&
            ((buffer[s + 1U] == 'o') || (buffer[s + 1U] == 'O')));
}

bool Command_IsStopCommand(const uint8_t *buffer, uint16_t length)
{
    uint16_t s, e;
    if ((buffer == NULL) || (length == 0U)) { return false; }
    command_trim(buffer, length, &s, &e);
    if ((e - s) != 4U) { return false; }
    return (((buffer[s]      == 's') || (buffer[s]      == 'S')) &&
            ((buffer[s + 1U] == 't') || (buffer[s + 1U] == 'T')) &&
            ((buffer[s + 2U] == 'o') || (buffer[s + 2U] == 'O')) &&
            ((buffer[s + 3U] == 'p') || (buffer[s + 3U] == 'P')));
}

/* ============================================================================
 * Three-motor angle command parser  ("10 20 30" / "10,20,30" / "10;20;30")
 * ============================================================================ */

static bool command_is_sep(uint8_t c)
{
    return ((c == ' ') || (c == '\t') || (c == ',') || (c == ';'));
}

static void command_skip_sep(const uint8_t *buf, uint16_t len, uint16_t *i)
{
    while ((*i < len) && command_is_sep(buf[*i])) { (*i)++; }
}

static bool command_parse_angle(const uint8_t *buf, uint16_t len,
                                uint16_t *i, int32_t *cdeg)
{
    int32_t whole = 0;
    int32_t frac  = 0;
    uint8_t fdig  = 0U;
    bool    got   = false;

    while ((*i < len) && (buf[*i] >= '0') && (buf[*i] <= '9'))
    {
        got    = true;
        whole  = (whole * 10) + (int32_t)(buf[*i] - '0');
        (*i)++;
    }
    if ((*i < len) && (buf[*i] == '.'))
    {
        (*i)++;
        while ((*i < len) && (buf[*i] >= '0') && (buf[*i] <= '9') && (fdig < 2U))
        {
            got   = true;
            frac  = (frac * 10) + (int32_t)(buf[*i] - '0');
            fdig++;
            (*i)++;
        }
        /* Nếu còn chữ số thập phân thứ 3 trở đi thì không hợp lệ. */
        if ((*i < len) && (buf[*i] >= '0') && (buf[*i] <= '9')) { return false; }
    }
    if (fdig == 1U) { frac *= 10; }
    if (!got)       { return false; }
    *cdeg = (whole * 100) + frac;
    return true;
}

bool Command_ParseThreeMotorCommand(const uint8_t *buffer, uint16_t length,
                                    MyController_ThreeMotorMoveCommand_t *command)
{
    uint16_t i = 0U;
    int32_t  angles[3] = {0, 0, 0};
    uint8_t  m;

    if ((buffer == NULL) || (command == NULL) || (length == 0U)) { return false; }

    for (m = 0U; m < 3U; m++)
    {
        command_skip_sep(buffer, length, &i);
        if (!command_parse_angle(buffer, length, &i, &angles[m])) { return false; }
        if (angles[m] > MY_CONTROLLER_FULL_TURN_CDEG)             { return false; }
    }

    /* Sau ba góc chỉ cho phép whitespace / separator / CR / LF. */
    while ((i < length) &&
           (command_is_sep(buffer[i]) || (buffer[i] == '\r') || (buffer[i] == '\n')))
    {
        i++;
    }
    if (i != length) { return false; }

    command->motor1_angle_cdeg = angles[0];
    command->motor2_angle_cdeg = angles[1];
    command->motor3_angle_cdeg = angles[2];
    return true;
}
