# Robot Arm Serial Protocol

## 1. Mục tiêu

Tài liệu này định nghĩa giao thức truyền dữ liệu giữa **PC Simulation** và **STM32 Robot Arm Controller** thông qua cổng **COM / USB_CDC**.

Mục tiêu của phiên bản này là:

- Packet đơn giản, dễ đọc, dễ debug.
- Có thể test bằng Serial Monitor hoặc Python.
- STM32 dễ parse bằng chuỗi text.
- Hỗ trợ gửi 3 góc điều khiển cho 3 động cơ.
- Có phản hồi `ACK`, `DONE`, `ERROR` từ STM32 về PC.
- Có `seq` để PC biết phản hồi thuộc về command nào.
- Có thể mở rộng về sau nếu cần packet nhị phân hoặc CRC.

---

## 2. Nguyên tắc thiết kế

Protocol sử dụng dạng **ASCII text packet**, mỗi packet kết thúc bằng ký tự newline:

```text
\n
```

Một packet hoàn chỉnh có dạng:

```text
$<TYPE>,<FIELD_1>,<FIELD_2>,...\n
```

Ví dụ:

```text
$M,1,9000,4550,-3000\n
```

Trong đó:

- `$` là ký tự bắt đầu packet.
- `TYPE` là loại packet.
- Các field được phân tách bằng dấu phẩy `,`.
- `\n` là ký tự kết thúc packet.

---

## 3. Quy ước đơn vị góc

Không truyền số thực `float` trực tiếp.

Tất cả góc được truyền dưới dạng số nguyên `angle_x100`.

Công thức:

```text
angle_x100 = angle_degree * 100
```

Ví dụ:

| Góc thực tế | Giá trị truyền |
| ----------: | -------------: |
|    `90.00°` |         `9000` |
|    `45.50°` |         `4550` |
|   `-30.00°` |        `-3000` |
|     `0.00°` |            `0` |

Khi STM32 cần đổi ngược lại:

```text
angle_degree = angle_x100 / 100.0
```

---

## 4. Kiểu dữ liệu quy ước

| Field         | Kiểu dữ liệu         | Ghi chú              |
| ------------- | -------------------- | -------------------- |
| `seq`         | `uint8_t` hoặc `int` | Số thứ tự command    |
| `angle1_x100` | `int32_t`            | Góc motor 1 nhân 100 |
| `angle2_x100` | `int32_t`            | Góc motor 2 nhân 100 |
| `angle3_x100` | `int32_t`            | Góc motor 3 nhân 100 |
| `error_code`  | string               | Mã lỗi dạng text     |

---

## 5. Các loại packet

| TYPE | Chiều truyền | Ý nghĩa                                            |
| ---- | ------------ | -------------------------------------------------- |
| `M`  | PC → STM32   | Move command: gửi 3 góc mục tiêu                   |
| `A`  | STM32 → PC   | ACK: STM32 đã nhận command hợp lệ và đưa vào queue |
| `D`  | STM32 → PC   | DONE: STM32 đã chạy xong command                   |
| `E`  | STM32 → PC   | ERROR: có lỗi xảy ra                               |

---

# 6. Packet PC gửi xuống STM32

## 6.1. Move Command

### Format

```text
$M,<seq>,<angle1_x100>,<angle2_x100>,<angle3_x100>\n
```

### Ý nghĩa field

| Field         | Ý nghĩa                  |
| ------------- | ------------------------ |
| `$`           | Ký tự bắt đầu packet     |
| `M`           | Move command             |
| `seq`         | Số thứ tự command        |
| `angle1_x100` | Góc mục tiêu của motor 1 |
| `angle2_x100` | Góc mục tiêu của motor 2 |
| `angle3_x100` | Góc mục tiêu của motor 3 |
| `\n`          | Ký tự kết thúc packet    |

### Ví dụ

```text
$M,1,9000,4550,-3000\n
```

Ý nghĩa:

```text
seq = 1
angle1 = 90.00°
angle2 = 45.50°
angle3 = -30.00°
```

---

## 6.2. Quy tắc tạo `seq`

`seq` là số thứ tự command do PC tạo.

Khuyến nghị:

```text
seq chạy từ 0 đến 255
sau 255 quay lại 0
```

Ví dụ:

```text
0, 1, 2, 3, ..., 254, 255, 0, 1, ...
```

Mục đích của `seq`:

- PC biết `ACK` thuộc command nào.
- PC biết `DONE` thuộc command nào.
- Dễ debug khi gửi nhiều command liên tiếp.
- Dễ phát hiện mất phản hồi hoặc phản hồi sai.

---

# 7. Packet STM32 gửi về PC

## 7.1. ACK Packet

STM32 gửi `ACK` khi packet move hợp lệ và đã được đưa vào queue.

### Format

```text
$A,<seq>\n
```

### Ví dụ

```text
$A,1\n
```

Ý nghĩa:

```text
STM32 đã nhận command seq = 1
Command đã được đưa vào queue
```

Lưu ý:

`ACK` không có nghĩa là motor đã chạy xong. `ACK` chỉ có nghĩa là STM32 đã nhận và chấp nhận command.

---

## 7.2. DONE Packet

STM32 gửi `DONE` khi motor đã chạy xong command tương ứng.

### Format

```text
$D,<seq>,<angle1_x100>,<angle2_x100>,<angle3_x100>\n
```

### Ví dụ

```text
$D,1,9000,4550,-3000\n
```

Ý nghĩa:

```text
Command seq = 1 đã chạy xong
STM32 echo lại 3 góc đã nhận:
angle1 = 90.00°
angle2 = 45.50°
angle3 = -30.00°
```

PC dùng packet `DONE` để cập nhật lại mô phỏng hoặc xác nhận robot arm đã đi tới vị trí mong muốn.

---

## 7.3. ERROR Packet

STM32 gửi `ERROR` khi có lỗi xảy ra.

### Format

```text
$E,<seq>,<error_code>\n
```

### Ví dụ

```text
$E,1,RANGE\n
```

Ý nghĩa:

```text
Command seq = 1 bị lỗi RANGE
```

Nếu STM32 không đọc được `seq` do packet sai format nghiêm trọng, STM32 có thể gửi:

```text
$E,0,FORMAT\n
```

---

# 8. Error Code

| Error code     | Ý nghĩa              | Khi nào xảy ra                                                       |
| -------------- | -------------------- | -------------------------------------------------------------------- |
| `FORMAT`       | Sai định dạng packet | Thiếu field, sai dấu phẩy, không bắt đầu bằng `$`, type không hợp lệ |
| `RANGE`        | Góc vượt giới hạn    | Một hoặc nhiều góc nằm ngoài giới hạn cho phép                       |
| `QUEUE_FULL`   | Queue đầy            | STM32 không thể nhận thêm command                                    |
| `BUSY`         | Hệ thống đang bận    | Dùng khi không muốn nhận thêm command trong chế độ không queue       |
| `MOTOR`        | Lỗi motor            | Motor không chạy được hoặc timeout                                   |
| `UNKNOWN_TYPE` | Type không hỗ trợ    | Packet type không nằm trong danh sách protocol                       |

---

# 9. Giới hạn góc

Giới hạn góc cần được thống nhất giữa PC và STM32.

Phiên bản mặc định:

| Motor   |        Min |       Max | Dạng `angle_x100`    |
| ------- | ---------: | --------: | -------------------- |
| Motor 1 | `-180.00°` | `180.00°` | `-18000` đến `18000` |
| Motor 2 | `-180.00°` | `180.00°` | `-18000` đến `18000` |
| Motor 3 | `-180.00°` | `180.00°` | `-18000` đến `18000` |

Nếu robot thật có giới hạn khác, cần sửa bảng này theo cơ khí thực tế.

Ví dụ:

| Motor    |        Min |       Max |
| -------- | ---------: | --------: |
| Base     |  `-90.00°` |  `90.00°` |
| Shoulder |    `0.00°` | `135.00°` |
| Elbow    | `-120.00°` | `120.00°` |

---

# 10. Luồng giao tiếp chuẩn

## 10.1. Trường hợp thành công

```text
PC    -> STM32: $M,1,9000,4550,-3000\n
STM32 -> PC   : $A,1\n
STM32 -> PC   : $D,1,9000,4550,-3000\n
```

Ý nghĩa:

1. PC gửi command `seq = 1`.
2. STM32 nhận đúng và đưa vào queue.
3. STM32 gửi `ACK`.
4. Motion task chạy motor.
5. Chạy xong, STM32 gửi `DONE`.

---

## 10.2. Trường hợp sai giới hạn góc

```text
PC    -> STM32: $M,2,999999,0,0\n
STM32 -> PC   : $E,2,RANGE\n
```

Ý nghĩa:

```text
Command seq = 2 bị từ chối vì góc vượt giới hạn.
```

---

## 10.3. Trường hợp queue đầy

```text
PC    -> STM32: $M,3,1000,2000,3000\n
STM32 -> PC   : $E,3,QUEUE_FULL\n
```

Ý nghĩa:

```text
Command seq = 3 không được đưa vào queue vì queue đang đầy.
```

---

## 10.4. Trường hợp sai format

```text
PC    -> STM32: hello world\n
STM32 -> PC   : $E,0,FORMAT\n
```

Hoặc:

```text
PC    -> STM32: $M,4,9000,4550\n
STM32 -> PC   : $E,4,FORMAT\n
```

---

# 11. Quy tắc phía PC

PC cần thực hiện các bước sau:

1. Nhận 3 góc từ phần mô phỏng.
2. Kiểm tra giới hạn góc trước khi gửi.
3. Convert từ degree sang `angle_x100`.
4. Tạo packet `$M`.
5. Gửi packet qua COM.
6. Chờ phản hồi từ STM32.
7. Nếu nhận `$A,<seq>`:
   - Command đã được STM32 nhận.

8. Nếu nhận `$D,<seq>,...`:
   - Command đã chạy xong.
   - PC cập nhật lại mô phỏng theo góc STM32 echo về.

9. Nếu nhận `$E,<seq>,<error_code>`:
   - PC xử lý lỗi tương ứng.

10. Nếu timeout:

- PC báo lỗi mất phản hồi hoặc gửi lại nếu cần.

---

## 11.1. Chính sách gửi command

Trong phiên bản đầu tiên, khuyến nghị dùng chế độ an toàn:

```text
PC gửi 1 command
PC chờ DONE
Sau đó mới gửi command tiếp theo
```

Flow:

```text
SEND CMD 1 -> WAIT ACK -> WAIT DONE -> SEND CMD 2
```

Ưu điểm:

- Dễ debug.
- Không làm đầy queue.
- Ít lỗi trong giai đoạn đầu.

Sau khi hệ thống ổn định, có thể nâng cấp sang chế độ queue:

```text
PC được gửi nhiều command liên tiếp
Nhưng số command chưa DONE không vượt quá giới hạn cho phép
```

Ví dụ:

```text
max_pending_command = 5
```

---

# 12. Quy tắc phía STM32

STM32 cần chia xử lý thành các phần độc lập:

## 12.1. USB Receive

Nhiệm vụ:

- Nhận byte từ `CDC_Receive_FS`.
- Không parse packet trong callback.
- Không điều khiển motor trong callback.
- Chỉ đẩy byte vào buffer.

Nguyên tắc:

```text
CDC_Receive_FS chỉ nhận dữ liệu và lưu lại.
Parser xử lý ở vòng main hoặc task riêng.
```

---

## 12.2. Line Buffer

Vì USB_CDC là byte stream, STM32 không được giả định mỗi lần receive là đúng một packet.

Ví dụ PC gửi:

```text
$M,1,9000,4550,-3000\n
```

STM32 có thể nhận thành nhiều phần:

```text
$M,1,90
00,4550
,-3000\n
```

Vì vậy STM32 phải ghép byte cho tới khi gặp:

```text
\n
```

Khi gặp `\n`, STM32 mới xem là nhận được một packet hoàn chỉnh.

---

## 12.3. Packet Decoder

Decoder nhận một line hoàn chỉnh, ví dụ:

```text
$M,1,9000,4550,-3000
```

Sau đó kiểm tra:

1. Có bắt đầu bằng `$` không.
2. Type có phải `M` không.
3. Có đúng 5 field không.
4. `seq` có hợp lệ không.
5. 3 góc có phải số nguyên không.
6. 3 góc có nằm trong giới hạn không.

Nếu hợp lệ:

```text
Tạo RobotCommand
Đưa vào command queue
Gửi ACK
```

Nếu không hợp lệ:

```text
Gửi ERROR
```

---

## 12.4. Command Queue

Command hợp lệ được đưa vào queue.

Nếu queue còn chỗ:

```text
Push command vào queue
Gửi ACK
```

Nếu queue đầy:

```text
Không push command
Gửi ERROR QUEUE_FULL
```

---

## 12.5. Motion Task

Motion task lấy command từ queue và điều khiển motor.

Flow:

```text
Nếu queue rỗng:
    Không làm gì

Nếu queue có command:
    Pop command
    Convert angle_x100 sang step
    Điều khiển motor chạy tới vị trí
    Khi chạy xong gửi DONE
```

---

# 13. Cấu trúc command trên STM32

Cấu trúc dữ liệu khuyến nghị:

```c
typedef struct
{
    uint8_t seq;
    int32_t angle1_x100;
    int32_t angle2_x100;
    int32_t angle3_x100;
} RobotCommand;
```

---

# 14. Interface module khuyến nghị

## 14.1. Protocol Decoder

```c
typedef enum
{
    PROTO_OK = 0,
    PROTO_ERR_FORMAT,
    PROTO_ERR_RANGE,
    PROTO_ERR_UNKNOWN_TYPE
} ProtocolStatus;

ProtocolStatus Protocol_DecodeMoveLine(const char *line, RobotCommand *cmd);
```

---

## 14.2. Command Queue

```c
bool CommandQueue_Push(const RobotCommand *cmd);
bool CommandQueue_Pop(RobotCommand *cmd);
bool CommandQueue_IsFull(void);
bool CommandQueue_IsEmpty(void);
```

---

## 14.3. Response Sender

```c
void Response_SendACK(uint8_t seq);
void Response_SendDONE(const RobotCommand *cmd);
void Response_SendERR(uint8_t seq, const char *err_code);
```

---

## 14.4. USB RX

```c
void USB_RX_PushBytes(uint8_t *data, uint32_t len);
int USB_RX_GetLine(char *line, uint16_t max_len);
```

---

# 15. Ví dụ encode phía PC

```python
def encode_move(seq, angle1_deg, angle2_deg, angle3_deg):
    a1 = int(angle1_deg * 100)
    a2 = int(angle2_deg * 100)
    a3 = int(angle3_deg * 100)

    packet = f"$M,{seq},{a1},{a2},{a3}\n"
    return packet.encode("ascii")
```

Ví dụ:

```python
packet = encode_move(1, 90.0, 45.5, -30.0)
print(packet)
```

Output:

```text
b'$M,1,9000,4550,-3000\n'
```

---

# 16. Ví dụ decode phía STM32

Input:

```text
$M,1,9000,4550,-3000
```

Sau khi decode thành:

```c
RobotCommand cmd;

cmd.seq = 1;
cmd.angle1_x100 = 9000;
cmd.angle2_x100 = 4550;
cmd.angle3_x100 = -3000;
```

---

# 17. Test case bắt buộc

## 17.1. Packet đúng

Input:

```text
$M,1,9000,4550,-3000\n
```

Expected:

```text
$A,1\n
$D,1,9000,4550,-3000\n
```

---

## 17.2. Sai format

Input:

```text
hello\n
```

Expected:

```text
$E,0,FORMAT\n
```

---

## 17.3. Thiếu field

Input:

```text
$M,2,9000,4550\n
```

Expected:

```text
$E,2,FORMAT\n
```

---

## 17.4. Góc vượt giới hạn

Input:

```text
$M,3,999999,0,0\n
```

Expected:

```text
$E,3,RANGE\n
```

---

## 17.5. Queue đầy

Input:

```text
$M,4,1000,2000,3000\n
```

Khi queue đầy, expected:

```text
$E,4,QUEUE_FULL\n
```

---

## 17.6. Số âm

Input:

```text
$M,5,-3000,0,4500\n
```

Expected:

```text
$A,5\n
$D,5,-3000,0,4500\n
```

---

## 17.7. Nhiều packet liên tiếp

Input:

```text
$M,6,1000,2000,3000\n$M,7,4000,5000,6000\n
```

Expected:

```text
$A,6\n
$A,7\n
$D,6,1000,2000,3000\n
$D,7,4000,5000,6000\n
```

Lưu ý:

Nếu motion task chạy tuần tự, `DONE` của command `6` phải xuất hiện trước `DONE` của command `7`.

---

# 18. Quy tắc timeout phía PC

Khuyến nghị PC có timeout khi chờ phản hồi.

Ví dụ:

| Trạng thái |          Timeout đề xuất |
| ---------- | -----------------------: |
| Chờ ACK    |                   500 ms |
| Chờ DONE   | tùy thời gian motor chạy |
| Chờ ERROR  |      xử lý ngay khi nhận |

Nếu quá thời gian mà không nhận được ACK:

```text
PC báo lỗi: STM32 không phản hồi
```

Nếu đã nhận ACK nhưng không nhận DONE:

```text
PC báo lỗi: motor chạy quá lâu hoặc STM32 bị treo
```

---

# 19. Các điểm không được làm

## 19.1. Không điều khiển motor trong USB callback

Không làm:

```c
CDC_Receive_FS()
{
    decode_packet();
    motor_run();
}
```

Nên làm:

```c
CDC_Receive_FS()
{
    USB_RX_PushBytes(Buf, Len);
}
```

---

## 19.2. Không parse dựa trên từng lần receive

Sai tư duy:

```text
Mỗi lần CDC_Receive_FS là một packet hoàn chỉnh
```

Đúng tư duy:

```text
USB_CDC là byte stream
Cần ghép byte cho tới khi gặp '\n'
```

---

## 19.3. Không dùng float trong packet

Không gửi:

```text
$M,1,90.0,45.5,-30.0\n
```

Nên gửi:

```text
$M,1,9000,4550,-3000\n
```

---

# 20. Phiên bản protocol

Phiên bản hiện tại:

```text
Protocol Version: 1.0
Packet Type: ASCII Text
Line Ending: \n
Checksum: None
```

Ghi chú:

Phiên bản 1.0 chưa dùng checksum hoặc CRC để giữ packet đơn giản và dễ chạy.

Trong tương lai có thể nâng cấp thành:

```text
$M,<seq>,<a1>,<a2>,<a3>,<checksum>\n
```

Hoặc chuyển sang packet nhị phân nếu cần tốc độ cao hơn.

---

# 21. Tóm tắt nhanh

PC gửi command:

```text
$M,<seq>,<angle1_x100>,<angle2_x100>,<angle3_x100>\n
```

STM32 phản hồi khi nhận đúng:

```text
$A,<seq>\n
```

STM32 phản hồi khi chạy xong:

```text
$D,<seq>,<angle1_x100>,<angle2_x100>,<angle3_x100>\n
```

STM32 phản hồi khi lỗi:

```text
$E,<seq>,<error_code>\n
```

Packet mẫu:

```text
PC    -> STM32: $M,1,9000,4550,-3000\n
STM32 -> PC   : $A,1\n
STM32 -> PC   : $D,1,9000,4550,-3000\n
```
