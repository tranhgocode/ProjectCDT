/**
 * @file    my_queue.c
 * @brief   Hàng đợi vòng và bộ parser byte cho frame quỹ đạo 36 byte.
 * @author  Lap4all
 * @date    2026-06-08
 */

#include "my_queue.h"
#include "my_app.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ============================================================================
 * Private define
 * ============================================================================ */

/** @brief Số byte payload sau khi bỏ header (MY_QUEUE_FRAME_SIZE - 2). */
#define MY_QUEUE_PAYLOAD_SIZE       34U

/** @brief Số byte dữ liệu đưa vào XOR checksum (timestep + 7 float = 32). */
#define MY_QUEUE_CHECKSUM_DATA_LEN  32U

/* ============================================================================
 * Private typedef
 * ============================================================================ */

typedef enum {
    MY_QUEUE_PARSE_SEEK_SYNC0 = 0,  /* Tìm byte 0xAA */
    MY_QUEUE_PARSE_SEEK_SYNC1,      /* Xác nhận byte 0x55 tiếp theo */
    MY_QUEUE_PARSE_COLLECT,         /* Thu thập 34 byte payload */
} my_queue_parse_state_t;

/* Union giúp đọc float/uint32 từ 4 byte mà không cần memcpy. */
typedef union {
    uint8_t  bytes[4];
    uint32_t u32;
    float    f;
} prv_word_t;

/* ============================================================================
 * Private variables
 * ============================================================================ */

/** @brief Mảng lưu các frame đã parse — circular buffer. */
static MyQueue_Frame_t s_frames[MY_QUEUE_CAPACITY];

/** @brief Chỉ số đọc tiếp theo. */
static uint16_t s_head;

/** @brief Chỉ số ghi tiếp theo. */
static uint16_t s_tail;

/** @brief Số frame hiện có trong hàng đợi. */
static uint16_t s_count;

/** @brief Trạng thái máy trạng thái parser. */
static my_queue_parse_state_t s_parse_state;

/** @brief Bộ đệm payload (34 byte sau header). */
static uint8_t s_payload[MY_QUEUE_PAYLOAD_SIZE];

/** @brief Số byte payload đã thu thập. */
static uint16_t s_payload_idx;

/** @brief Cờ báo có frame bị drop do queue đầy, tự reset khi MyQueue_WasDropped() đọc. */
static bool s_drop_flag;

/* ============================================================================
 * Private function prototypes
 * ============================================================================ */

static void prv_parser_reset(void);
static uint32_t prv_read_u32(uint16_t offset);
static float prv_read_float(uint16_t offset);
static bool prv_validate_and_push(void);
static void prv_parse_byte(uint8_t byte);

/* ============================================================================
 * Private function definitions
 * ============================================================================ */

/** @brief Đặt lại parser về trạng thái tìm byte header đầu tiên. */
static void prv_parser_reset(void)
{
    s_parse_state = MY_QUEUE_PARSE_SEEK_SYNC0;
    s_payload_idx = 0U;
}

/**
 * @brief  Đọc uint32_t little-endian từ buffer payload.
 * @param  offset: Vị trí byte đầu tiên trong s_payload.
 * @return Giá trị uint32_t tương ứng.
 */
static uint32_t prv_read_u32(uint16_t offset)
{
    prv_word_t w;

    w.bytes[0] = s_payload[offset];
    w.bytes[1] = s_payload[offset + 1U];
    w.bytes[2] = s_payload[offset + 2U];
    w.bytes[3] = s_payload[offset + 3U];
    return w.u32;
}

/**
 * @brief  Đọc float IEEE 754 little-endian từ buffer payload.
 * @param  offset: Vị trí byte đầu tiên trong s_payload.
 * @return Giá trị float tương ứng.
 */
static float prv_read_float(uint16_t offset)
{
    prv_word_t w;

    w.bytes[0] = s_payload[offset];
    w.bytes[1] = s_payload[offset + 1U];
    w.bytes[2] = s_payload[offset + 2U];
    w.bytes[3] = s_payload[offset + 3U];
    return w.f;
}

/**
 * @brief  Kiểm tra checksum và footer rồi push frame vào queue nếu hợp lệ.
 * @return true nếu frame hợp lệ và được thêm vào queue, ngược lại false.
 */
static bool prv_validate_and_push(void)
{
    uint8_t checksum = 0U;
    uint16_t i;
    MyQueue_Frame_t frame;

    /* Tính XOR của 32 byte dữ liệu (timestep + 7 float). */
    for (i = 0U; i < MY_QUEUE_CHECKSUM_DATA_LEN; i++)
    {
        checksum ^= s_payload[i];
    }

    if (checksum != s_payload[MY_QUEUE_CHECKSUM_DATA_LEN])
    {
        return false;
    }

    if (s_payload[MY_QUEUE_PAYLOAD_SIZE - 1U] != MY_QUEUE_FRAME_FOOTER)
    {
        return false;
    }

    if (s_count >= MY_QUEUE_CAPACITY)
    {
        s_drop_flag = true;
        return false;
    }

    /* Giải mã frame từ payload. */
    frame.timestep   = prv_read_u32(0U);
    frame.theta1_deg = prv_read_float(4U);
    frame.vel1_dps   = prv_read_float(8U);
    frame.theta2_deg = prv_read_float(12U);
    frame.vel2_dps   = prv_read_float(16U);
    frame.theta3_deg = prv_read_float(20U);
    frame.vel3_dps   = prv_read_float(24U);
    frame.theta4_deg = prv_read_float(28U);

    s_frames[s_tail] = frame;
    s_tail = (s_tail + 1U) % MY_QUEUE_CAPACITY;
    s_count++;

    return true;
}

/**
 * @brief  Xử lý một byte qua máy trạng thái parser.
 * @param  byte: Byte nhận được từ USB CDC.
 */
static void prv_parse_byte(uint8_t byte)
{
    switch (s_parse_state)
    {
    case MY_QUEUE_PARSE_SEEK_SYNC0:
        if (byte == MY_QUEUE_FRAME_HEADER_0)
        {
            s_parse_state = MY_QUEUE_PARSE_SEEK_SYNC1;
        }
        break;

    case MY_QUEUE_PARSE_SEEK_SYNC1:
        if (byte == MY_QUEUE_FRAME_HEADER_1)
        {
            s_payload_idx = 0U;
            s_parse_state = MY_QUEUE_PARSE_COLLECT;
        }
        else if (byte == MY_QUEUE_FRAME_HEADER_0)
        {
            /* 0xAA 0xAA: ở lại SEEK_SYNC1 để chờ 0x55. */
        }
        else
        {
            s_parse_state = MY_QUEUE_PARSE_SEEK_SYNC0;
        }
        break;

    case MY_QUEUE_PARSE_COLLECT:
        s_payload[s_payload_idx] = byte;
        s_payload_idx++;

        if (s_payload_idx >= MY_QUEUE_PAYLOAD_SIZE)
        {
            (void)prv_validate_and_push();
            prv_parser_reset();
        }
        break;

    default:
        prv_parser_reset();
        break;
    }
}

/* ============================================================================
 * Exported function definitions
 * ============================================================================ */

/**
 * @brief  Khởi tạo hàng đợi và bộ parser byte về trạng thái ban đầu.
 */
void MyQueue_Init(void)
{
    s_head = 0U;
    s_tail = 0U;
    s_count = 0U;
    s_drop_flag = false;
    prv_parser_reset();
}

/**
 * @brief  Nạp byte thô từ USB CDC và tự động parse thành frame khi đủ dữ liệu.
 * @param  data:   Con trỏ tới mảng byte cần xử lý.
 * @param  length: Số byte cần xử lý.
 */
void MyQueue_FeedBytes(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (data == NULL)
    {
        return;
    }

    for (i = 0U; i < length; i++)
    {
        prv_parse_byte(data[i]);
    }
}

/**
 * @brief  Lấy một frame từ đầu hàng đợi.
 * @param  frame: Nơi lưu frame lấy ra.
 * @return true nếu lấy thành công, false nếu hàng đợi rỗng.
 */
bool MyQueue_Pop(MyQueue_Frame_t *frame)
{
    if ((frame == NULL) || (s_count == 0U))
    {
        return false;
    }

    *frame = s_frames[s_head];
    s_head = (s_head + 1U) % MY_QUEUE_CAPACITY;
    s_count--;
    return true;
}

/**
 * @brief  Trả về số frame hiện có trong hàng đợi.
 * @return Số frame đang chờ thực thi.
 */
uint16_t MyQueue_Count(void)
{
    return s_count;
}

/**
 * @brief  Kiểm tra hàng đợi có rỗng không.
 * @return true nếu không còn frame nào.
 */
bool MyQueue_IsEmpty(void)
{
    return (s_count == 0U);
}

/**
 * @brief  Kiểm tra hàng đợi có đầy không.
 * @return true nếu không còn chỗ cho frame mới.
 */
bool MyQueue_IsFull(void)
{
    return (s_count >= MY_QUEUE_CAPACITY);
}

/**
 * @brief  Xóa toàn bộ frame và đặt lại parser về trạng thái tìm header.
 */
void MyQueue_Flush(void)
{
    s_head = 0U;
    s_tail = 0U;
    s_count = 0U;
    s_drop_flag = false;
    prv_parser_reset();
}

bool MyQueue_WasDropped(void)
{
    bool v = s_drop_flag;
    s_drop_flag = false;
    return v;
}
