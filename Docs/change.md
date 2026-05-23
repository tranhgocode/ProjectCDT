# Change Log

## Current App Flow

1. On boot, the app sets the current yaw coordinate to `0.00 deg`.
2. The AS5600 sensor path is initialized through TCA9548A channel 0.
3. The current AS5600 position is used as the initial yaw zero.
4. USB CDC command `zero` can be sent at any time while idle to reset the
   current AS5600 position as yaw `0.00 deg`.
5. Numeric USB CDC input is treated as an absolute target yaw in degrees.
6. The valid target range is `0.00..360.00 deg`.
7. The app calculates `target_yaw - current_yaw`.
8. Positive delta rotates `TMC2209_DIR_CW`.
9. Negative delta rotates `TMC2209_DIR_CCW`.
10. After the motor stops, the app reads AS5600 again and prints the result.

## Command Examples

```text
zero
120
360
90.50
```

Examples:

- Current yaw `0.00`, input `120.00` -> rotate forward `+120.00 deg`.
- Current yaw `360.00`, input `120.00` -> rotate backward `-240.00 deg`.

## USB CDC Report

```text
input_deg=120.00,start_deg=0.00,end_deg=119.97,delta_deg=119.97,error_deg=0.03,steps=4800
```

Fields:

- `input_deg`: target yaw received from USB CDC.
- `start_deg`: AS5600 yaw before motor movement.
- `end_deg`: AS5600 yaw after motor movement.
- `delta_deg`: measured AS5600 movement.
- `error_deg`: commanded delta minus measured delta.
- `steps`: motor microsteps generated for the move.

## Files Changed

- `Core/Src/main.c`
  - Calls `my_app_init()` once during startup.
  - Calls `my_app_process()` repeatedly in the main loop.

- `Mylib/Inc/my_app.h`
  - Uses snake_case app API names.
  - Keeps only public functions currently used by the app.

- `Mylib/Src/my_app.c`
  - Reworked app logic from relative move input to absolute yaw target input.
  - Added `zero` command handling.
  - Tracks `s_current_yaw_cdeg` internally.
  - Uses AS5600 zero offset stored in `s_sensor_zero_cdeg`.
  - Removed old unused relative-move helpers.

- `USB_DEVICE/App/usbd_cdc_if.c`
  - Keeps received USB CDC command in a pending buffer for the app layer.

- `USB_DEVICE/App/usbd_cdc_if.h`
  - Exposes `CDC_ReadCommand()` for app-layer command polling.

- `Mylib/Src/TMC2209.c`
  - Uses PWM interrupt start/stop so step pulses can be counted and stopped.
