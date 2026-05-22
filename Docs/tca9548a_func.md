# TCA9548A Function Reference

Tai lieu nay mo ta nhanh cac ham trong thu vien `TCA9548A`. TCA9548A la I2C multiplexer 8 kenh, duoc dung trong project de ket noi nhieu cam bien `AS5600` co cung dia chi I2C `0x36`.

Truoc khi dung, can tao `TCA9548A_Handle_t` va gan callback HAL cho I2C:

```c
TCA9548A_Handle_t mux;

mux.i2c_write = My_I2C_Write;
mux.i2c_read  = My_I2C_Read;
mux.delay_ms  = HAL_Delay;

(void)TCA9548A_Init(&mux, TCA9548A_ADDR_A000);
```

## Danh Sach Ham

### <code style="color: #facc15;">TCA9548A_Init(TCA9548A_Handle_t *mux, TCA9548A_I2CAddr_t addr)</code>

Khoi tao TCA9548A va tat tat ca kenh.

- Tham so: `mux` la handle da gan callback I2C, `addr` la dia chi TCA9548A tu `0x70..0x77`.
- Tra ve: `TCA9548A_OK` neu thanh cong, hoac ma loi `TCA9548A_Status_t`.
- Vi du:

```c
(void)TCA9548A_Init(&mux, TCA9548A_ADDR_A000);
```

- Luu y: ham set mode mac dinh la `TCA9548A_MODE_SINGLE`, xoa danh sach sensor, kiem tra ket noi roi goi `TCA9548A_Reset()`.

### <code style="color: #facc15;">TCA9548A_Reset(TCA9548A_Handle_t *mux)</code>

Reset trang thai mux bang cach tat tat ca kenh.

- Tham so: `mux` la handle TCA9548A.
- Tra ve: `TCA9548A_OK` neu ghi control register thanh cong.
- Vi du:

```c
(void)TCA9548A_Reset(&mux);
```

- Luu y: sau khi reset, `active_channel = TCA9548A_CH_NONE`.

### <code style="color: #facc15;">TCA9548A_SelectChannel(TCA9548A_Handle_t *mux, TCA9548A_Channel_t ch)</code>

Chon mot kenh duy nhat tren TCA9548A.

- Tham so: `ch` la `TCA9548A_CH0` den `TCA9548A_CH7`.
- Tra ve: `TCA9548A_OK` neu chon kenh thanh cong.
- Vi du:

```c
(void)TCA9548A_SelectChannel(&mux, TCA9548A_CH0);
```

- Luu y: ham ghi mask `1 << ch`, nen cac kenh khac se tat.

### <code style="color: #facc15;">TCA9548A_SetChannelMask(TCA9548A_Handle_t *mux, uint8_t mask)</code>

Bat/tat kenh theo bitmask.

- Tham so: `mask` la bitmask kenh, bit0 la CH0, bit7 la CH7.
- Tra ve: `TCA9548A_OK` neu ghi control register thanh cong.
- Vi du:

```c
(void)TCA9548A_SetChannelMask(&mux, 0x03); /* bat CH0 va CH1 */
```

- Luu y: `mask = 0x00` la tat tat ca kenh. `active_channel` se la kenh thap nhat dang bat.

### <code style="color: #facc15;">TCA9548A_DisableAll(TCA9548A_Handle_t *mux)</code>

Tat tat ca kenh cua TCA9548A.

- Tham so: `mux` la handle TCA9548A.
- Tra ve: `TCA9548A_OK` neu thanh cong.
- Vi du:

```c
(void)TCA9548A_DisableAll(&mux);
```

### <code style="color: #facc15;">TCA9548A_GetChannelMask(TCA9548A_Handle_t *mux, uint8_t *mask)</code>

Doc bitmask kenh hien tai tu chip.

- Tham so: `mask` la noi luu bitmask doc ve.
- Tra ve: `TCA9548A_OK` neu doc thanh cong.
- Vi du:

```c
uint8_t mask;
(void)TCA9548A_GetChannelMask(&mux, &mask);
```

- Luu y: neu doc thanh cong, ham cap nhat `mux->active_mask`.

### <code style="color: #facc15;">TCA9548A_GetChannelState(TCA9548A_Handle_t *mux, TCA9548A_Channel_t ch, TCA9548A_ChannelState_t *state)</code>

Kiem tra mot kenh dang bat hay tat dua tren `mux->active_mask`.

- Tham so: `ch` la kenh can kiem tra, `state` la noi luu `TCA9548A_CH_ENABLED` hoac `TCA9548A_CH_DISABLED`.
- Tra ve: `TCA9548A_OK` neu tham so hop le.
- Vi du:

```c
TCA9548A_ChannelState_t state;
(void)TCA9548A_GetChannelState(&mux, TCA9548A_CH0, &state);
```

- Luu y: neu muon trang thai moi nhat tu chip, nen goi `TCA9548A_GetChannelMask()` truoc.

### <code style="color: #facc15;">TCA9548A_IsConnected(TCA9548A_Handle_t *mux)</code>

Kiem tra TCA9548A co phan hoi tren bus I2C hay khong.

- Tham so: `mux` la handle TCA9548A.
- Tra ve: `TCA9548A_OK` neu doc duoc control register.
- Vi du:

```c
if (TCA9548A_IsConnected(&mux) == TCA9548A_OK) {
    /* mux online */
}
```

### <code style="color: #facc15;">TCA9548A_RegisterSensor(TCA9548A_Handle_t *mux, TCA9548A_Channel_t ch, const char *label)</code>

Dang ky mot cam bien AS5600 nam tren kenh TCA9548A.

- Tham so: `ch` la kenh cua cam bien, `label` la nhan tuy chon, co the truyen `NULL`.
- Tra ve: `TCA9548A_OK` neu dang ky thanh cong.
- Vi du:

```c
(void)TCA9548A_RegisterSensor(&mux, TCA9548A_CH0, "MOTOR_1");
```

- Luu y: ham tao `AS5600_Handle_t` noi bo cho slot nay va gan chung callback I2C cua mux. Neu kenh da dang ky, ham tra ve `TCA9548A_OK`.

### <code style="color: #facc15;">TCA9548A_InitAllSensors(TCA9548A_Handle_t *mux)</code>

Khoi tao tat ca AS5600 da dang ky.

- Tham so: `mux` la handle TCA9548A.
- Tra ve: `TCA9548A_OK` neu tat ca sensor khoi tao thanh cong.
- Vi du:

```c
(void)TCA9548A_RegisterSensor(&mux, TCA9548A_CH0, "AS5600_CH0");
(void)TCA9548A_InitAllSensors(&mux);
```

- Luu y: ham lan luot chon tung kenh, delay 2 ms neu co `delay_ms`, goi `AS5600_Init()`, sau do tat tat ca kenh.

### <code style="color: #facc15;">TCA9548A_ReadSensor(TCA9548A_Handle_t *mux, TCA9548A_Channel_t ch, AS5600_Data_t *data)</code>

Doc day du du lieu tu AS5600 tren kenh chi dinh.

- Tham so: `ch` la kenh can doc, `data` la noi luu du lieu AS5600.
- Tra ve: `TCA9548A_OK` neu doc thanh cong.
- Vi du:

```c
AS5600_Data_t data;
(void)TCA9548A_ReadSensor(&mux, TCA9548A_CH0, &data);
```

- Luu y: kenh phai duoc dang ky truoc bang `TCA9548A_RegisterSensor()`. Ham goi `AS5600_ReadAll()` sau khi chon kenh.

### <code style="color: #facc15;">TCA9548A_ReadAngleDeg(TCA9548A_Handle_t *mux, TCA9548A_Channel_t ch, float *degrees)</code>

Doc rieng goc theo do tu AS5600 tren kenh chi dinh.

- Tham so: `ch` la kenh can doc, `degrees` la noi luu goc `0.0..360.0`.
- Tra ve: `TCA9548A_OK` neu doc thanh cong.
- Vi du:

```c
float angle_deg;
(void)TCA9548A_ReadAngleDeg(&mux, TCA9548A_CH0, &angle_deg);
```

- Luu y: ham nhanh hon `TCA9548A_ReadSensor()` khi chi can gia tri goc.

### <code style="color: #facc15;">TCA9548A_ScanAllSensors(TCA9548A_Handle_t *mux, TCA9548A_ScanResult_t *result)</code>

Quet va doc tat ca AS5600 da dang ky.

- Tham so: `result` la noi luu danh sach kenh, du lieu va trang thai tung sensor.
- Tra ve: `TCA9548A_OK` neu qua trinh scan chay xong.
- Vi du:

```c
TCA9548A_ScanResult_t result;
(void)TCA9548A_ScanAllSensors(&mux, &result);
```

- Luu y: neu mot sensor loi, ham van tiep tuc scan sensor khac va luu loi vao `result.status[index]`.

### <code style="color: #facc15;">TCA9548A_GetSensorHandle(TCA9548A_Handle_t *mux, TCA9548A_Channel_t ch)</code>

Lay con tro den handle AS5600 noi bo cua mot kenh.

- Tham so: `ch` la kenh can lay sensor handle.
- Tra ve: con tro `AS5600_Handle_t *`, hoac `NULL` neu kenh khong hop le/chua dang ky.
- Vi du:

```c
AS5600_Handle_t *sensor = TCA9548A_GetSensorHandle(&mux, TCA9548A_CH0);
if (sensor != NULL) {
    (void)AS5600_ReadAngleDeg(sensor, &angle_deg);
}
```

- Luu y: neu tu doc truc tiep qua handle nay, ban phai tu chon kenh TCA9548A truoc.

## Gia Tri Tra Ve

- `TCA9548A_OK`: thanh cong.
- `TCA9548A_ERR_I2C`: loi giao tiep I2C.
- `TCA9548A_ERR_NULL_PTR`: con tro null hoac callback I2C chua gan.
- `TCA9548A_ERR_INVALID_CH`: kenh khong hop le, lon hon 7.
- `TCA9548A_ERR_SENSOR_FAIL`: loi khi thao tac voi AS5600.
- `TCA9548A_ERR_NOT_INIT`: sensor tren kenh do chua duoc dang ky/khoi tao.
- `TCA9548A_ERR_CHANNEL_BUSY`: khong con slot trong de dang ky sensor.

## Trinh Tu Dung Co Ban

```c
TCA9548A_Handle_t mux;

mux.i2c_write = My_I2C_Write;
mux.i2c_read  = My_I2C_Read;
mux.delay_ms  = HAL_Delay;

(void)TCA9548A_Init(&mux, TCA9548A_ADDR_A000);
(void)TCA9548A_RegisterSensor(&mux, TCA9548A_CH0, "AS5600_CH0");
(void)TCA9548A_InitAllSensors(&mux);

float angle_deg;
(void)TCA9548A_ReadAngleDeg(&mux, TCA9548A_CH0, &angle_deg);
```

## Ghi Chu

- TCA9548A chi co mot thanh ghi dieu khien 1 byte; ghi byte nay de bat/tat cac kenh.
- Khi dung `TCA9548A_MODE_SINGLE`, cac ham doc sensor se tat tat ca kenh sau khi doc xong.
- Cac callback I2C trong project can xu ly rieng TCA9548A va AS5600: TCA9548A dung transmit/receive truc tiep, AS5600 dung mem read/write theo register.
