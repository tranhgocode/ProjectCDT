# AS5600 Function Reference

Tai lieu nay mo ta nhanh cac ham trong thu vien `AS5600`. Truoc khi dung, can tao `AS5600_Handle_t` va gan cac ham HAL callback cho I2C:

```c
AS5600_Handle_t as5600;

as5600.i2c_write = My_I2C_Write;
as5600.i2c_read  = My_I2C_Read;
as5600.delay_ms  = HAL_Delay;

(void)AS5600_Init(&as5600);
```

Mac dinh dia chi I2C cua AS5600 la `0x36`. Neu `dev->i2c_addr = 0`, ham `AS5600_Init()` se tu gan ve `AS5600_I2C_ADDR`.

## Danh Sach Ham

### <code style="color: #facc15;">AS5600_Init(AS5600_Handle_t *dev)</code>

Khoi tao cam bien AS5600 voi cau hinh mac dinh.

- Tham so: `dev` la handle AS5600 da gan `i2c_write`, `i2c_read`, va co the gan `delay_ms`.
- Tra ve: `AS5600_OK` neu thanh cong, hoac ma loi `AS5600_Status_t`.
- Vi du:

```c
(void)AS5600_Init(&as5600);
```

- Luu y: ham se kiem tra ket noi bang cach doc thanh ghi `STATUS`, sau do ghi cau hinh mac dinh vao `CONF`.

### <code style="color: #facc15;">AS5600_SetConfig(AS5600_Handle_t *dev, AS5600_Config_t *cfg)</code>

Ghi cau hinh hoat dong vao thanh ghi `CONF`.

- Tham so: `dev` la handle AS5600, `cfg` la cau hinh can ghi.
- Tra ve: `AS5600_OK` neu ghi thanh cong.
- Vi du:

```c
AS5600_Config_t cfg = as5600.config;
cfg.slow_filter = AS5600_SF_8X;
(void)AS5600_SetConfig(&as5600, &cfg);
```

- Luu y: neu ghi thanh cong, ham cap nhat lai `dev->config`.

### <code style="color: #facc15;">AS5600_GetConfig(AS5600_Handle_t *dev, AS5600_Config_t *cfg)</code>

Doc cau hinh hien tai tu thanh ghi `CONF`.

- Tham so: `dev` la handle AS5600, `cfg` la noi luu cau hinh doc ve.
- Tra ve: `AS5600_OK` neu doc thanh cong.
- Vi du:

```c
AS5600_Config_t cfg;
(void)AS5600_GetConfig(&as5600, &cfg);
```

### <code style="color: #facc15;">AS5600_SetZone(AS5600_Handle_t *dev, AS5600_Zone_t *zone)</code>

Cai dat vung goc hoat dong bang cac thanh ghi `ZPOS`, `MPOS`, va `MANG`.

- Tham so: `zone->start_pos`, `zone->stop_pos`, `zone->max_angle` la gia tri 12-bit `0..4095`.
- Tra ve: `AS5600_OK` neu ghi thanh cong.
- Vi du:

```c
AS5600_Zone_t zone = {
    .start_pos = 0,
    .stop_pos = 4095,
    .max_angle = 4095
};
(void)AS5600_SetZone(&as5600, &zone);
```

- Luu y: ham chi lay 12 bit thap cua moi gia tri.

### <code style="color: #facc15;">AS5600_ReadAll(AS5600_Handle_t *dev, AS5600_Data_t *data)</code>

Doc day du du lieu cam bien: trang thai nam cham, goc tho, goc da loc, goc do, goc radian, AGC va magnitude.

- Tham so: `data` la noi luu ket qua doc ve.
- Tra ve: `AS5600_OK` neu doc tat ca thanh cong.
- Vi du:

```c
AS5600_Data_t data;
if (AS5600_ReadAll(&as5600, &data) == AS5600_OK) {
    float angle = data.angle_deg;
}
```

- Luu y: neu thanh cong, ham cap nhat `dev->data`.

### <code style="color: #facc15;">AS5600_ReadAngle(AS5600_Handle_t *dev, uint16_t *angle)</code>

Doc goc da loc tu thanh ghi `ANGLE`.

- Tham so: `angle` la noi luu gia tri raw 12-bit `0..4095`.
- Tra ve: `AS5600_OK` neu doc thanh cong.
- Vi du:

```c
uint16_t angle_raw;
(void)AS5600_ReadAngle(&as5600, &angle_raw);
```

- Luu y: day la ham doc nhanh khi chi can gia tri raw cua goc da loc.

### <code style="color: #facc15;">AS5600_ReadRawAngle(AS5600_Handle_t *dev, uint16_t *raw_angle)</code>

Doc goc tho tu thanh ghi `RAW_ANGLE`.

- Tham so: `raw_angle` la noi luu gia tri raw 12-bit `0..4095`.
- Tra ve: `AS5600_OK` neu doc thanh cong.
- Vi du:

```c
uint16_t raw_angle;
(void)AS5600_ReadRawAngle(&as5600, &raw_angle);
```

- Luu y: gia tri nay chua qua bo loc noi cua AS5600.

### <code style="color: #facc15;">AS5600_ReadAngleDeg(AS5600_Handle_t *dev, float *degrees)</code>

Doc goc da loc va doi sang don vi do.

- Tham so: `degrees` la noi luu goc `0.0..360.0`.
- Tra ve: `AS5600_OK` neu doc thanh cong.
- Vi du:

```c
float angle_deg;
(void)AS5600_ReadAngleDeg(&as5600, &angle_deg);
```

- Luu y: ham dung `AS5600_RawToDegrees()` de doi tu raw 12-bit sang do.

### <code style="color: #facc15;">AS5600_GetMagnetStatus(AS5600_Handle_t *dev, AS5600_MagnetStatus_t *status)</code>

Doc va phan tich trang thai nam cham.

- Tham so: `status` la noi luu trang thai nam cham.
- Tra ve: `AS5600_OK` neu doc thanh cong.
- Gia tri co the nhan:
  - `AS5600_MAG_NOT_DETECTED`: khong thay nam cham.
  - `AS5600_MAG_DETECTED`: nam cham hop le.
  - `AS5600_MAG_TOO_WEAK`: tu truong qua yeu.
  - `AS5600_MAG_TOO_STRONG`: tu truong qua manh.
- Vi du:

```c
AS5600_MagnetStatus_t magnet;
(void)AS5600_GetMagnetStatus(&as5600, &magnet);
```

### <code style="color: #facc15;">AS5600_ReadAGC(AS5600_Handle_t *dev, uint8_t *agc)</code>

Doc gia tri `AGC` cua AS5600.

- Tham so: `agc` la noi luu gia tri `0..255`.
- Tra ve: `AS5600_OK` neu doc thanh cong.
- Vi du:

```c
uint8_t agc;
(void)AS5600_ReadAGC(&as5600, &agc);
```

- Luu y: AGC giup danh gia muc tu truong ma cam bien dang tu dong bu.

### <code style="color: #facc15;">AS5600_ReadMagnitude(AS5600_Handle_t *dev, uint16_t *magnitude)</code>

Doc do lon tu truong `MAGNITUDE`.

- Tham so: `magnitude` la noi luu gia tri 12-bit `0..4095`.
- Tra ve: `AS5600_OK` neu doc thanh cong.
- Vi du:

```c
uint16_t magnitude;
(void)AS5600_ReadMagnitude(&as5600, &magnitude);
```

### <code style="color: #facc15;">AS5600_BurnSettings(AS5600_Handle_t *dev)</code>

Burn cau hinh hien tai vao OTP.

- Tham so: `dev` la handle AS5600.
- Tra ve: `AS5600_OK` neu gui lenh burn thanh cong, `AS5600_ERR_BURN_FAIL` neu da het so lan burn.
- Vi du:

```c
(void)AS5600_BurnSettings(&as5600);
```

- Luu y: thao tac burn OTP khong the hoan tac. Ham kiem tra thanh ghi `ZMCO`, neu `ZMCO >= 3` thi khong burn nua.

### <code style="color: #facc15;">AS5600_BurnAngle(AS5600_Handle_t *dev)</code>

Burn goc zero vao OTP.

- Tham so: `dev` la handle AS5600.
- Tra ve: `AS5600_OK` neu gui lenh burn thanh cong.
- Vi du:

```c
(void)AS5600_BurnAngle(&as5600);
```

- Luu y: thao tac burn OTP khong the hoan tac, chi dung khi da chac chan can luu goc zero vao chip.

### <code style="color: #facc15;">AS5600_IsConnected(AS5600_Handle_t *dev)</code>

Kiem tra AS5600 co phan hoi tren bus I2C hay khong.

- Tham so: `dev` la handle AS5600.
- Tra ve: `AS5600_OK` neu doc duoc thanh ghi `STATUS`.
- Vi du:

```c
if (AS5600_IsConnected(&as5600) == AS5600_OK) {
    /* sensor online */
}
```

### <code style="color: #facc15;">AS5600_RawToDegrees(uint16_t raw)</code>

Doi gia tri raw 12-bit sang don vi do.

- Tham so: `raw` la gia tri `0..4095`.
- Tra ve: goc dang `float`, xap xi `0.0..360.0`.
- Vi du:

```c
float deg = AS5600_RawToDegrees(2048);
```

- Luu y: ham chi lay 12 bit thap bang `raw & 0x0FFF`.

### <code style="color: #facc15;">AS5600_RawToRadians(uint16_t raw)</code>

Doi gia tri raw 12-bit sang radian.

- Tham so: `raw` la gia tri `0..4095`.
- Tra ve: goc dang `float`, xap xi `0..2*pi`.
- Vi du:

```c
float rad = AS5600_RawToRadians(2048);
```

## Gia Tri Tra Ve

- `AS5600_OK`: thanh cong.
- `AS5600_ERR_I2C`: loi giao tiep I2C.
- `AS5600_ERR_NO_MAGNET`: khong phat hien nam cham.
- `AS5600_ERR_MAGNET_WEAK`: tu truong qua yeu.
- `AS5600_ERR_MAGNET_STRONG`: tu truong qua manh.
- `AS5600_ERR_NULL_PTR`: con tro null hoac callback HAL chua gan.
- `AS5600_ERR_BURN_FAIL`: burn OTP that bai hoac da het so lan burn.

## Ghi Chu

- Cac ham doc I2C tra loi neu callback `i2c_read` hoac `i2c_write` tra khac `0`.
- `AS5600_ReadAngle()` doc thanh ghi `ANGLE`, con `AS5600_ReadRawAngle()` doc thanh ghi `RAW_ANGLE`.
- `AS5600_ReadAll()` phu hop de debug day du; neu can toc do nhanh hon thi dung `AS5600_ReadAngle()` hoac `AS5600_ReadAngleDeg()`.
