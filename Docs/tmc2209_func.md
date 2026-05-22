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

## Danh Sach Ham

### <code style="color: #facc15;">TMC2209_Init(TMC2209_HandleTypeDef *hmotor)</code>

Khoi tao driver TMC2209. Ham nay set GPIO mac dinh, ghi cac register co ban qua UART va cau hinh PWM mac dinh 100 Hz.

- Tham so: `hmotor` la con tro toi handle da cau hinh du timer, chan `DIR`, chan `EN`, thong so motor va dia chi UART slave.
- Tra ve: `TMC2209_OK`, `TMC2209_ERROR`, `TMC2209_TIMEOUT` hoac `TMC2209_CRC_ERROR`.
- Vi du:

```c
(void)TMC2209_Init(&motor1);
```

- Luu y: goi sau khi `MX_GPIO_Init()`, `MX_USARTx_UART_Init()` va `MX_TIMx_Init()` da chay.

### <code style="color: #facc15;">TMC2209_SetCurrent(TMC2209_HandleTypeDef *hmotor, uint8_t run_current, uint8_t hold_current, uint8_t hold_delay)</code>

Cai dong chay va dong giu cho motor.

- Tham so: `run_current` tu 0..31, `hold_current` tu 0..31, `hold_delay` tu 0..15.
- Tra ve: `TMC2209_StatusTypeDef`.
- Vi du:

```c
(void)TMC2209_SetCurrent(&motor1, 20, 8, 6);
```

- Luu y: gia tri qua gioi han se bi kep ve max trong ham.

### <code style="color: #facc15;">TMC2209_SetMicrostep(TMC2209_HandleTypeDef *hmotor, TMC2209_MicrostepTypeDef microstep)</code>

Doi do chia microstep cua driver.

- Tham so: `microstep` co the la `TMC2209_MICROSTEP_FULL`, `TMC2209_MICROSTEP_2`, `TMC2209_MICROSTEP_4`, `TMC2209_MICROSTEP_8`, `TMC2209_MICROSTEP_16`, `TMC2209_MICROSTEP_32`, `TMC2209_MICROSTEP_64`, `TMC2209_MICROSTEP_128` hoac `TMC2209_MICROSTEP_256`.
- Tra ve: `TMC2209_StatusTypeDef`.
- Vi du:

```c
(void)TMC2209_SetMicrostep(&motor1, TMC2209_MICROSTEP_16);
```

- Luu y: nen set truoc khi chay motor de tinh toc do va goc dung.

### <code style="color: #facc15;">TMC2209_SetStealthChop(TMC2209_HandleTypeDef *hmotor, bool enable)</code>

Bat hoac tat che do StealthChop.

- Tham so: `enable = true` de bat StealthChop, `enable = false` de dung SpreadCycle.
- Tra ve: `TMC2209_StatusTypeDef`.
- Vi du:

```c
(void)TMC2209_SetStealthChop(&motor1, true);
```

- Luu y: StealthChop thuong em hon o toc do thap.

### <code style="color: #facc15;">TMC2209_Enable(TMC2209_HandleTypeDef *hmotor)</code>

Bat driver motor.

- Tham so: `hmotor` la handle cua motor can bat.
- Tra ve: `void`.
- Vi du:

```c
TMC2209_Enable(&motor1);
```

- Luu y: chan `EN` active-low, ham se keo `EN` xuong thap.

### <code style="color: #facc15;">TMC2209_Disable(TMC2209_HandleTypeDef *hmotor)</code>

Tat driver motor.

- Tham so: `hmotor` la handle cua motor can tat.
- Tra ve: `void`.
- Vi du:

```c
TMC2209_Disable(&motor1);
```

- Luu y: ham keo `EN` len cao, motor mat luc giu.

### <code style="color: #facc15;">TMC2209_SetDirection(TMC2209_HandleTypeDef *hmotor, TMC2209_DirectionTypeDef dir)</code>

Dat chieu quay cua motor.

- Tham so: `dir` la `TMC2209_DIR_CW` hoac `TMC2209_DIR_CCW`.
- Tra ve: `void`.
- Vi du:

```c
TMC2209_SetDirection(&motor1, TMC2209_DIR_CW);
```

- Luu y: goi truoc `TMC2209_Start()` hoac truoc khi doi chieu.

### <code style="color: #facc15;">TMC2209_SetSpeedHz(TMC2209_HandleTypeDef *hmotor, uint32_t freq_hz)</code>

Dat toc do theo tan so xung STEP.

- Tham so: `freq_hz` la tan so xung STEP tinh bang Hz.
- Tra ve: `TMC2209_StatusTypeDef`.
- Vi du:

```c
(void)TMC2209_SetSpeedHz(&motor1, 3200);
```

- Luu y: ham tu tinh `ARR` va `CCR` cho timer PWM. `freq_hz = 0` se tra loi.

### <code style="color: #facc15;">TMC2209_SetSpeedRPM(TMC2209_HandleTypeDef *hmotor, float rpm)</code>

Dat toc do theo vong/phut.

- Tham so: `rpm` la toc do mong muon tinh bang RPM.
- Tra ve: `TMC2209_StatusTypeDef`.
- Vi du:

```c
(void)TMC2209_SetSpeedRPM(&motor1, 60.0f);
```

- Luu y: ham dua vao `steps_per_rev` va `microstep` trong handle. `rpm <= 0` se tra loi.

### <code style="color: #facc15;">TMC2209_Start(TMC2209_HandleTypeDef *hmotor)</code>

Bat dau phat PWM STEP de motor quay.

- Tham so: `hmotor` la handle cua motor can chay.
- Tra ve: `TMC2209_StatusTypeDef`.
- Vi du:

```c
(void)TMC2209_Start(&motor1);
```

- Luu y: neu motor dang disable, ham se tu goi `TMC2209_Enable()`.

### <code style="color: #facc15;">TMC2209_Stop(TMC2209_HandleTypeDef *hmotor)</code>

Dung phat PWM STEP.

- Tham so: `hmotor` la handle cua motor can dung.
- Tra ve: `void`.
- Vi du:

```c
TMC2209_Stop(&motor1);
```

- Luu y: ham chi dung xung STEP, khong tu disable driver.

### <code style="color: #facc15;">TMC2209_MoveSteps(TMC2209_HandleTypeDef *hmotor, uint32_t steps, TMC2209_DirectionTypeDef dir, float speed_rpm)</code>

Quay them mot so buoc theo chieu va toc do mong muon. Ham nay khong blocking.

- Tham so: `steps` la so xung/buoc can chay, `dir` la chieu quay, `speed_rpm` la toc do.
- Tra ve: `TMC2209_StatusTypeDef`.
- Vi du:

```c
(void)TMC2209_MoveSteps(&motor1, 1600, TMC2209_DIR_CW, 60.0f);
```

- Luu y: can goi `TMC2209_UpdateSteps()` trong interrupt de motor tu dung khi du buoc.

### <code style="color: #facc15;">TMC2209_MoveToAngle(TMC2209_HandleTypeDef *hmotor, float angle_deg, float speed_rpm)</code>

Quay toi mot goc tuyet doi.

- Tham so: `angle_deg` la goc muc tieu, `speed_rpm` la toc do.
- Tra ve: `TMC2209_StatusTypeDef`.
- Vi du:

```c
(void)TMC2209_MoveToAngle(&motor1, 90.0f, 30.0f);
```

- Luu y: ham dua vao `current_angle` trong handle. Can goi `TMC2209_UpdateSteps()` trong interrupt.

### <code style="color: #facc15;">TMC2209_UpdateSteps(TMC2209_HandleTypeDef *hmotor)</code>

Dem buoc da chay va tu dung motor khi du so buoc muc tieu.

- Tham so: `hmotor` la handle cua motor dang chay.
- Tra ve: `true` neu vua dung vi da du buoc, `false` neu chua dung.
- Vi du:

```c
if (htim == motor1.htim) {
    (void)TMC2209_UpdateSteps(&motor1);
}
```

- Luu y: goi trong `HAL_TIM_PWM_PulseFinishedCallback()` hoac callback timer phu hop.

### <code style="color: #facc15;">TMC2209_ReadDrvStatus(TMC2209_HandleTypeDef *hmotor, uint32_t *status_out)</code>

Doc register `DRV_STATUS` cua TMC2209.

- Tham so: `status_out` la noi luu gia tri doc duoc.
- Tra ve: `TMC2209_StatusTypeDef`.
- Vi du:

```c
uint32_t st;
(void)TMC2209_ReadDrvStatus(&motor1, &st);
```

- Luu y: dung de debug loi driver, nhiet, short hoac trang thai phan cung.

### <code style="color: #facc15;">TMC2209_ReadSGResult(TMC2209_HandleTypeDef *hmotor, uint16_t *sg_out)</code>

Doc gia tri StallGuard `SG_RESULT`.

- Tham so: `sg_out` la noi luu ket qua 9 bit.
- Tra ve: `TMC2209_StatusTypeDef`.
- Vi du:

```c
uint16_t sg;
(void)TMC2209_ReadSGResult(&motor1, &sg);
```

- Luu y: gia tri nay phu thuoc tai, toc do va cau hinh driver.

### <code style="color: #facc15;">TMC2209_IsStalled(TMC2209_HandleTypeDef *hmotor, uint8_t threshold)</code>

Kiem tra motor co dau hieu ket qua StallGuard hay khong.

- Tham so: `threshold` la nguong so sanh voi `SG_RESULT`.
- Tra ve: `true` neu `SG_RESULT < threshold`, nguoc lai tra ve `false`.
- Vi du:

```c
if (TMC2209_IsStalled(&motor1, 50)) {
    TMC2209_Stop(&motor1);
}
```

- Luu y: neu doc UART loi, ham hien tra `false`.

### <code style="color: #facc15;">TMC2209_HasFault(TMC2209_HandleTypeDef *hmotor)</code>

Kiem tra driver co loi nhu qua nhiet hoac short hay khong.

- Tham so: `hmotor` la handle cua motor can kiem tra.
- Tra ve: `true` neu co fault, `false` neu khong co hoac doc loi.
- Vi du:

```c
if (TMC2209_HasFault(&motor1)) {
    TMC2209_Disable(&motor1);
}
```

- Luu y: ham kiem tra cac bit fault chinh trong `DRV_STATUS`.

### <code style="color: #facc15;">TMC2209_WriteReg(TMC2209_HandleTypeDef *hmotor, uint8_t reg_addr, uint32_t value)</code>

Ghi truc tiep mot register TMC2209 qua UART.

- Tham so: `reg_addr` la dia chi register, `value` la du lieu 32 bit can ghi.
- Tra ve: `TMC2209_StatusTypeDef`.
- Vi du:

```c
(void)TMC2209_WriteReg(&motor1, TMC2209_REG_GCONF, value);
```

- Luu y: day la ham cap thap, chi dung khi can cau hinh register chua co wrapper.

### <code style="color: #facc15;">TMC2209_ReadReg(TMC2209_HandleTypeDef *hmotor, uint8_t reg_addr, uint32_t *value_out)</code>

Doc truc tiep mot register TMC2209 qua UART.

- Tham so: `reg_addr` la dia chi register, `value_out` la noi luu du lieu 32 bit doc duoc.
- Tra ve: `TMC2209_StatusTypeDef`.
- Vi du:

```c
uint32_t val;
(void)TMC2209_ReadReg(&motor1, TMC2209_REG_GCONF, &val);
```

- Luu y: ham co kiem tra CRC reply.

### <code style="color: #facc15;">TMC2209_RPM_to_Hz(const TMC2209_HandleTypeDef *hmotor, float rpm)</code>

Doi RPM sang tan so STEP.

- Tham so: `rpm` la toc do vong/phut.
- Tra ve: `uint32_t` tan so Hz.
- Vi du:

```c
uint32_t hz = TMC2209_RPM_to_Hz(&motor1, 60.0f);
```

- Luu y: ham inline trong header, dung cong thuc `rpm * steps_per_rev * microstep / 60`.

### <code style="color: #facc15;">TMC2209_Hz_to_RPM(const TMC2209_HandleTypeDef *hmotor, uint32_t hz)</code>

Doi tan so STEP sang RPM.

- Tham so: `hz` la tan so STEP.
- Tra ve: `float` RPM.
- Vi du:

```c
float rpm = TMC2209_Hz_to_RPM(&motor1, 3200);
```

- Luu y: ham inline trong header, dung de kiem tra hoac hien thi toc do.

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
