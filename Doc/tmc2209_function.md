# TMC2209 Function Reference

Tai lieu nay mo ta nhanh cac ham trong thu vien `TMC2209`. Truoc khi goi cac ham dieu khien, can chuan bi mot bien `TMC2209_HandleTypeDef`, gan du timer PWM, chan `DIR`, chan `EN`, thong so motor va dia chi UART slave.

Vi du handle da duoc cau hinh trong `my_app.c`:

```c
TMC2209_HandleTypeDef motor1;

motor1.htim = &htim2;
motor1.tim_channel = TIM_CHANNEL_1;
motor1.dir_port = DIR_1_GPIO_Port;
motor1.dir_pin = DIR_1_Pin;
motor1.en_port = EN_1_GPIO_Port;
motor1.en_pin = EN_1_Pin;
motor1.timer_clock_hz = TMC2209_TIMER_CLOCK_HZ;
motor1.prescaler = TMC2209_TIMER_PRESCALER;
motor1.steps_per_rev = TMC2209_MOTOR_STEPS_REV;
motor1.microstep = TMC2209_MICROSTEP_16;
motor1.slave_address = TMC2209_SLAVE_ADD1;
```

## Bang Ham

| Ham | Muc dich | Tham so chinh | Tra ve | Cach su dung nhanh | Luu y |
|---|---|---|---|---|---|
| `TMC2209_Init(TMC2209_HandleTypeDef *hmotor)` | Khoi tao driver, set GPIO mac dinh, ghi cac register co ban qua UART, cau hinh PWM mac dinh 100 Hz. | `hmotor`: con tro toi handle da cau hinh du phan cung. | `TMC2209_OK`, `TMC2209_ERROR`, `TMC2209_TIMEOUT`, `TMC2209_CRC_ERROR`. | `(void)TMC2209_Init(&motor1);` | Goi sau khi `MX_GPIO_Init()`, `MX_USARTx_UART_Init()` va `MX_TIMx_Init()` da chay. |
| `TMC2209_SetCurrent(TMC2209_HandleTypeDef *hmotor, uint8_t run_current, uint8_t hold_current, uint8_t hold_delay)` | Cai dong chay va dong giu cho motor. | `run_current`: 0..31, `hold_current`: 0..31, `hold_delay`: 0..15. | `TMC2209_StatusTypeDef`. | `(void)TMC2209_SetCurrent(&motor1, 20, 8, 6);` | Gia tri qua gioi han se bi kep ve max trong ham. |
| `TMC2209_SetMicrostep(TMC2209_HandleTypeDef *hmotor, TMC2209_MicrostepTypeDef microstep)` | Doi do chia microstep. | `microstep`: `TMC2209_MICROSTEP_FULL`, `2`, `4`, `8`, `16`, `32`, `64`, `128`, `256`. | `TMC2209_StatusTypeDef`. | `(void)TMC2209_SetMicrostep(&motor1, TMC2209_MICROSTEP_16);` | Nen set truoc khi chay motor de tinh toc do/goc dung. |
| `TMC2209_SetStealthChop(TMC2209_HandleTypeDef *hmotor, bool enable)` | Bat/tat che do StealthChop. | `enable`: `true` bat StealthChop, `false` dung SpreadCycle. | `TMC2209_StatusTypeDef`. | `(void)TMC2209_SetStealthChop(&motor1, true);` | StealthChop thuong em hon o toc do thap. |
| `TMC2209_Enable(TMC2209_HandleTypeDef *hmotor)` | Bat driver motor. | `hmotor`. | `void`. | `TMC2209_Enable(&motor1);` | Chan `EN` active-low, ham se keo `EN` xuong thap. |
| `TMC2209_Disable(TMC2209_HandleTypeDef *hmotor)` | Tat driver motor. | `hmotor`. | `void`. | `TMC2209_Disable(&motor1);` | Ham keo `EN` len cao, motor mat luc giu. |
| `TMC2209_SetDirection(TMC2209_HandleTypeDef *hmotor, TMC2209_DirectionTypeDef dir)` | Dat chieu quay. | `dir`: `TMC2209_DIR_CW` hoac `TMC2209_DIR_CCW`. | `void`. | `TMC2209_SetDirection(&motor1, TMC2209_DIR_CW);` | Goi truoc `TMC2209_Start()` hoac truoc khi doi chieu. |
| `TMC2209_SetSpeedHz(TMC2209_HandleTypeDef *hmotor, uint32_t freq_hz)` | Dat toc do theo tan so xung STEP. | `freq_hz`: tan so xung Hz. | `TMC2209_StatusTypeDef`. | `(void)TMC2209_SetSpeedHz(&motor1, 3200);` | Ham tu tinh `ARR` va `CCR` cho timer PWM. `freq_hz = 0` tra loi. |
| `TMC2209_SetSpeedRPM(TMC2209_HandleTypeDef *hmotor, float rpm)` | Dat toc do theo vong/phut. | `rpm`: toc do RPM. | `TMC2209_StatusTypeDef`. | `(void)TMC2209_SetSpeedRPM(&motor1, 60.0f);` | Dua vao `steps_per_rev` va `microstep` trong handle. `rpm <= 0` tra loi. |
| `TMC2209_Start(TMC2209_HandleTypeDef *hmotor)` | Bat dau phat PWM STEP de motor quay. | `hmotor`. | `TMC2209_StatusTypeDef`. | `(void)TMC2209_Start(&motor1);` | Neu motor dang disable, ham se tu goi `TMC2209_Enable()`. |
| `TMC2209_Stop(TMC2209_HandleTypeDef *hmotor)` | Dung PWM STEP. | `hmotor`. | `void`. | `TMC2209_Stop(&motor1);` | Chi dung xung, khong tu disable driver. |
| `TMC2209_MoveSteps(TMC2209_HandleTypeDef *hmotor, uint32_t steps, TMC2209_DirectionTypeDef dir, float speed_rpm)` | Quay them mot so buoc, khong blocking. | `steps`: so xung/buoc can chay, `dir`: chieu quay, `speed_rpm`: toc do. | `TMC2209_StatusTypeDef`. | `(void)TMC2209_MoveSteps(&motor1, 1600, TMC2209_DIR_CW, 60.0f);` | Can goi `TMC2209_UpdateSteps()` trong interrupt de tu dung khi du buoc. |
| `TMC2209_MoveToAngle(TMC2209_HandleTypeDef *hmotor, float angle_deg, float speed_rpm)` | Quay toi goc tuyet doi. | `angle_deg`: goc muc tieu, `speed_rpm`: toc do. | `TMC2209_StatusTypeDef`. | `(void)TMC2209_MoveToAngle(&motor1, 90.0f, 30.0f);` | Dua vao `current_angle` trong handle. Can update buoc trong interrupt. |
| `TMC2209_UpdateSteps(TMC2209_HandleTypeDef *hmotor)` | Dem buoc va tu dung motor khi du so buoc muc tieu. | `hmotor`. | `true` neu vua dung vi du buoc, `false` neu chua dung. | `if (htim == motor1.htim) TMC2209_UpdateSteps(&motor1);` | Goi trong `HAL_TIM_PWM_PulseFinishedCallback()` hoac callback timer phu hop. |
| `TMC2209_ReadDrvStatus(TMC2209_HandleTypeDef *hmotor, uint32_t *status_out)` | Doc register `DRV_STATUS`. | `status_out`: noi luu gia tri doc duoc. | `TMC2209_StatusTypeDef`. | `uint32_t st; (void)TMC2209_ReadDrvStatus(&motor1, &st);` | Dung de debug loi driver, nhiet, short, trang thai phan cung. |
| `TMC2209_ReadSGResult(TMC2209_HandleTypeDef *hmotor, uint16_t *sg_out)` | Doc gia tri StallGuard `SG_RESULT`. | `sg_out`: noi luu ket qua 9 bit. | `TMC2209_StatusTypeDef`. | `uint16_t sg; (void)TMC2209_ReadSGResult(&motor1, &sg);` | Gia tri nay phu thuoc tai, toc do va cau hinh driver. |
| `TMC2209_IsStalled(TMC2209_HandleTypeDef *hmotor, uint8_t threshold)` | Kiem tra motor co dau hieu ket qua StallGuard. | `threshold`: nguong so sanh. | `true` neu `SG_RESULT < threshold`, nguoc lai `false`. | `if (TMC2209_IsStalled(&motor1, 50)) { TMC2209_Stop(&motor1); }` | Neu doc UART loi, ham hien tra `false`. |
| `TMC2209_HasFault(TMC2209_HandleTypeDef *hmotor)` | Kiem tra loi driver nhu qua nhiet hoac short. | `hmotor`. | `true` neu co fault, `false` neu khong co hoac doc loi. | `if (TMC2209_HasFault(&motor1)) { TMC2209_Disable(&motor1); }` | Ham kiem tra cac bit fault chinh trong `DRV_STATUS`. |
| `TMC2209_WriteReg(TMC2209_HandleTypeDef *hmotor, uint8_t reg_addr, uint32_t value)` | Ghi truc tiep mot register TMC2209 qua UART. | `reg_addr`: dia chi register, `value`: du lieu 32 bit. | `TMC2209_StatusTypeDef`. | `(void)TMC2209_WriteReg(&motor1, TMC2209_REG_GCONF, value);` | Ham cap thap, chi dung khi can cau hinh register chua co wrapper. |
| `TMC2209_ReadReg(TMC2209_HandleTypeDef *hmotor, uint8_t reg_addr, uint32_t *value_out)` | Doc truc tiep mot register TMC2209 qua UART. | `reg_addr`: dia chi register, `value_out`: noi luu du lieu 32 bit. | `TMC2209_StatusTypeDef`. | `uint32_t val; (void)TMC2209_ReadReg(&motor1, TMC2209_REG_GCONF, &val);` | Ham co kiem tra CRC reply. |
| `TMC2209_RPM_to_Hz(const TMC2209_HandleTypeDef *hmotor, float rpm)` | Doi RPM sang tan so STEP. | `rpm`: toc do vong/phut. | `uint32_t` Hz. | `uint32_t hz = TMC2209_RPM_to_Hz(&motor1, 60.0f);` | Inline trong header, dung cong thuc `rpm * steps_per_rev * microstep / 60`. |
| `TMC2209_Hz_to_RPM(const TMC2209_HandleTypeDef *hmotor, uint32_t hz)` | Doi tan so STEP sang RPM. | `hz`: tan so STEP. | `float` RPM. | `float rpm = TMC2209_Hz_to_RPM(&motor1, 3200);` | Inline trong header, dung de kiem tra hoac hien thi toc do. |

## Trinh Tu Dung Co Ban

```c
Motors_Config();

(void)TMC2209_Init(&motor1);
(void)TMC2209_SetStealthChop(&motor1, true);
(void)TMC2209_SetCurrent(&motor1, 20, 8, 6);

TMC2209_Enable(&motor1);
TMC2209_SetDirection(&motor1, TMC2209_DIR_CW);
(void)TMC2209_SetSpeedRPM(&motor1, 60.0f);
(void)TMC2209_Start(&motor1);

HAL_Delay(2000U);

TMC2209_Stop(&motor1);
TMC2209_Disable(&motor1);
```

## Vi Du Chay Theo So Buoc

```c
(void)TMC2209_MoveSteps(&motor1, 3200, TMC2209_DIR_CW, 60.0f);
```

Sau do can goi update trong callback timer/PWM:

```c
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == motor1.htim)
    {
        (void)TMC2209_UpdateSteps(&motor1);
    }
}
```

## Gia Tri Tra Ve

| Gia tri | Y nghia |
|---|---|
| `TMC2209_OK` | Lenh chay thanh cong. |
| `TMC2209_ERROR` | Loi chung, tham so khong hop le hoac HAL tra loi. |
| `TMC2209_TIMEOUT` | UART transmit/receive bi timeout. |
| `TMC2209_CRC_ERROR` | Du lieu doc ve sai CRC. |

## Ghi Chu

- `TMC2209_Start()` chi bat dau phat xung PWM; neu muon chay dung so buoc thi phai dung `TMC2209_MoveSteps()` hoac `TMC2209_MoveToAngle()` kem `TMC2209_UpdateSteps()`.
- `TMC2209_Stop()` khong disable driver, vi vay motor van co the giu luc neu driver dang enable.
- Cac ham doc/ghi register phu thuoc cau hinh UART trong `TMC2209.h`, hien dang dung `huart3` qua macro `TMC2209_UART`.
- Cac ham toc do phu thuoc dung gia tri `timer_clock_hz`, `prescaler`, `steps_per_rev` va `microstep` trong handle.

## Cách tính tần số 
* `spr` : số bước trên 1 vòng quay của động cơ (bao gồm vi bước nếu có)
* `speed_rpm ` :  tốc độ mong muốn (vòng/phút)
* `f_step ` : tần số xung cấp vào chân STEP (Hz)

```text
f_step (Hz) = (speed_rpm × spr) / 60
```

Ví dụ:
* Động cơ bước 1.8° → 200 bước/vòng (full step).
* Dùng TMC2209 ở chế độ vi bước 1/16 → spr = 200 × 16 = 3200 bước/vòng.
* Muốn quay 120 RPM → f_step = 120 × 3200 / 60 = 6400 Hz (6.4 kHz).

## Cách dùng Timer
**1. Cấu trúc của timer** 
* `Prescaler (PSC)`: chia nhỏ xung nhịp cho bộ đếm. (thời gian + thêm 1 giá trị CNT)
* `CNT_CLK = TIM_CLK / (PSC + 1)`
```markdown
ví dụ 
TIM_CLK = 72.000.000 Hz (72 MHz), PSC = 71 
-> CNT_CLK = 72.000.000 / (71+1) = 1.000.000 Hz 
-> thời gian tăng 1 giá trị CNT là `t = 1/1M = 1 µs` 
```
* `Counter (CNT)` – đếm từ 0 đến Auto-reload (ARR) rồi quay về 0 (chế độ đếm lên).
* `Auto-reload (ARR)`: giá trị tràn.
* Timer đếm từ 0 → ARR rồi quay lại (hoặc đếm xuống tùy chế độ).
* Tần số cập nhật (update event): `f_update` = `CNT_CLK / (ARR + 1)` or `TIM_CLK / ((PSC + 1) × (ARR + 1))`
```markdown
ví dụ
TIM_CLK = 72.000.000 Hz (72 MHz), PSC = 71
nếu muốn ARR = 999
-> f_update = 1.000.000 / 1000 = 1000hz 
-> t = 1/1000 = 1ms (đếm 1000 lần 1 µs = 1ms)
```
* Counter mode: Up, Down, Center-aligned (dùng cho PWM đối xứng).

**2. Cách tạo PWM**
* Chọn kênh (CHx) ra chân GPIO, set chế độ PWM mode 1 hoặc 2.
* Độ rộng xung điều khiển qua thanh ghi **`CCRx`**:
    * `duty = (CCR / (ARR + 1)) * 100%`
* Tần số PWM chính là `f_update` (tần số tràn) trong chế độ PWM cạnh thẳng hàng (edge-aligned).

## Cách dùng PWM để tạo ra xung STEP 

công thức 
`PWM frequency = Timer clock / ((PSC + 1) * (ARR + 1))`