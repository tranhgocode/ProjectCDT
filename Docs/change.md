# Change Log

## Muc tieu

Them luong dieu khien trong tang app:

1. Cho gia tri goc nhap tu USB CDC.
2. Doc goc AS5600 hien tai truoc khi chay motor.
3. Tinh so buoc motor can di tu goc input.
4. Dieu khien motor quay theo so buoc do.
5. Khi motor dung, doc lai goc AS5600.
6. Gui ket qua ra cong COM USB CDC.
7. Quay lai trang thai cho lenh tiep theo.

## Cach nhap lenh

Gui mot gia tri goc theo don vi degree qua COM USB CDC:

```text
90
-45.50
10.25
```

Pham vi dang chap nhan:

```text
-360.00 .. 360.00 deg
```

Gia tri duong quay theo `TMC2209_DIR_CW`, gia tri am quay theo `TMC2209_DIR_CCW`.

## Format phan hoi USB CDC

```text
input_deg=...,start_deg=...,end_deg=...,delta_deg=...,error_deg=...,steps=...
```

Y nghia:

- `input_deg`: gia tri nhan duoc, don vi degree.
- `start_deg`: goc cam bien luc dau, don vi degree.
- `end_deg`: goc cam bien luc sau, don vi degree.
- `delta_deg`: delta giua goc luc sau va luc dau, don vi degree.
- `error_deg`: `input_deg - delta_deg`, don vi degree.
- `steps`: so microstep da tinh de dieu khien motor.

Vi du `90.00 deg` duoc in ra la `90.00`.

## File da sua

- `Core/Src/main.c`
  - Goi `My_app()` trong vong lap chinh.
  - Xoa doan code test USB CDC da comment.

- `Mylib/Inc/my_app.h`
  - Them prototype `MyApp_ScaleAngleTo4000()`.
  - Them prototype `MyApp_CalculateMotorStepsFromAngle()`.
  - Them prototype `My_app()`.

- `Mylib/Src/my_app.c`
  - Them state machine cho app: cho lenh, chay motor, report ket qua.
  - Them parser doc goc tu USB CDC.
  - Them ham doc AS5600 va doi sang centidegree.
  - Format gia tri report ra USB CDC theo don vi degree.
  - Them ham tinh buoc motor tu goc input.
  - Them callback `HAL_TIM_PWM_PulseFinishedCallback()` de dem buoc motor.

- `USB_DEVICE/App/usbd_cdc_if.h`
  - Them API `CDC_ReadCommand()` de tang app lay lenh USB CDC.

- `USB_DEVICE/App/usbd_cdc_if.c`
  - Doi callback receive: chi copy lenh USB vao buffer, khong echo trong callback.
  - Them buffer lenh pending de `My_app()` xu ly o main loop.

- `Mylib/Src/TMC2209.c`
  - Doi PWM start/stop sang ban co interrupt: `HAL_TIM_PWM_Start_IT()` va
    `HAL_TIM_PWM_Stop_IT()`.
  - Muc dich la de `TMC2209_UpdateSteps()` duoc goi va motor tu dung dung so buoc.

## Luu y ky thuat

- AS5600 chi do goc trong mot vong, nen ket qua `delta_deg` duoc tinh theo huong input.
- Neu input gan `360 deg`, sai so co the bi anh huong boi diem wrap `359.99 -> 0.00`.
- Motor dang dung `motor1` lam truc dieu khien chinh.
