# Tài liệu phân chia nhiệm vụ trong `Mylib`

Thư mục `Mylib` chứa các module người dùng tự viết cho firmware STM32. Các
file trong thư mục này được tách theo từng lớp trách nhiệm: tầng ứng dụng nhận
lệnh, tầng điều phối chuyển động, tầng driver phần cứng và các module cấu hình
hoặc mở rộng.

## 1. Cấu trúc tổng quan

```text
Mylib/
├── Inc/    Header public: khai báo API, kiểu dữ liệu, macro và cấu hình.
└── Src/    Source implementation: triển khai logic tương ứng với header.
```

Nguyên tắc sử dụng:

- File `.h` trong `Mylib/Inc` là giao diện công khai để module khác include.
- File `.c` trong `Mylib/Src` chứa phần triển khai chi tiết của module.
- Các module cấp cao gọi xuống module cấp thấp hơn, không gọi ngược lên.
- Logic ứng dụng nên đặt trong `my_app` hoặc `my_controller`, không nhồi trực
  tiếp vào file sinh bởi STM32CubeMX nếu không cần thiết.

## 2. Luồng xử lý chính

```text
USB CDC
  ↓
command
  ↓
my_app
  ↓
my_controller
  ↓
TMC2209 / TCA9548A / AS5600
  ↓
Motor stepper / cảm biến góc
```

Tầng `command` nhận và chuẩn hóa dữ liệu USB CDC. Tầng `my_app` quản lý máy
trạng thái không chặn, quyết định lệnh nào đang được xử lý. Tầng
`my_controller` chuyển lệnh cấp ứng dụng thành số bước motor, hướng quay, thao
tác đọc cảm biến và báo kết quả. Các driver `TMC2209`, `TCA9548A`, `AS5600`
làm việc trực tiếp với ngoại vi UART, PWM timer và I2C.

## 3. Nhóm file ứng dụng

| File | Nhiệm vụ chính |
|---|---|
| `Inc/my_app.h` | Khai báo API khởi tạo và chạy vòng xử lý ứng dụng. |
| `Src/my_app.c` | Xử lý lệnh USB CDC theo kiểu không chặn, quản lý trạng thái chờ lệnh, chờ motor yaw và chờ ba motor hoàn tất. |

`my_app` là điểm vào của logic người dùng. Trong `main`, thường chỉ cần gọi
`my_app_init()` một lần khi khởi động và gọi lặp `my_app_process()` trong vòng
lặp chính.

Các nhiệm vụ chính của `my_app.c`:

- Khởi tạo toàn bộ tầng điều khiển thông qua `MyController_Init()`.
- Nhận lệnh từ USB CDC khi hệ thống đang rảnh.
- Hỗ trợ lệnh `zero`, một góc yaw đơn hoặc ba góc cho ba motor.
- Chuyển lệnh hợp lệ xuống `my_controller`.
- Theo dõi khi motor chạy xong và gửi báo cáo kết quả qua USB.
- Tránh block trong vòng lặp chính để firmware vẫn phản hồi tốt.

## 4. Nhóm file phân tích lệnh USB CDC

| File | Nhiệm vụ chính |
|---|---|
| `Inc/command.h` | Khai báo bộ đệm USB, API đọc/gửi dữ liệu, parse lệnh và format kết quả. |
| `Src/command.c` | Triển khai giao tiếp USB CDC, nhận diện lệnh `zero`, parse góc theo centi-độ và định dạng chuỗi phản hồi. |

`command` là lớp tiện ích giữa USB CDC và ứng dụng. Module này không quyết định
motor chạy thế nào; nó chỉ chịu trách nhiệm biến dữ liệu dạng text thành dữ
liệu có cấu trúc để `my_app` sử dụng.

Các nhiệm vụ chính:

- Kiểm tra USB CDC đã sẵn sàng truyền dữ liệu chưa.
- Đọc một lệnh từ bộ đệm USB CDC.
- Gửi chuỗi hoặc buffer nhị phân qua USB CDC.
- Nhận diện lệnh `zero` không phân biệt chữ hoa/thường.
- Parse một góc mục tiêu hoặc ba góc mục tiêu.
- Format góc centi-độ thành chuỗi có hai chữ số thập phân.
- Format kết quả đọc AS5600 thành góc hoặc mã lỗi.

## 5. Nhóm file điều phối motor và cảm biến

| File | Nhiệm vụ chính |
|---|---|
| `Inc/my_controller.h` | Khai báo API điều khiển yaw, điều khiển ba motor, đọc AS5600 và các struct ngữ cảnh/kết quả. |
| `Src/my_controller.c` | Điều phối ba driver TMC2209, mux TCA9548A và cảm biến AS5600; tính số bước, hướng quay, cập nhật vị trí phần mềm và xử lý callback timer. |

`my_controller` là lớp nghiệp vụ trung tâm của hệ thống chuyển động. Module này
biết motor nào dùng timer nào, cảm biến AS5600 nằm trên kênh mux nào, tỉ lệ
quy đổi góc sang bước ra sao và kết quả đo được báo về như thế nào.

Các nhiệm vụ chính:

- Cấu hình `motor1`, `motor2`, `motor3` với timer, chân DIR/EN, microstep và
  địa chỉ UART TMC2209.
- Khởi tạo driver TMC2209, TCA9548A và AS5600.
- Đọc góc tuyệt đối từ AS5600 và quy đổi sang centi-độ.
- Đặt mốc zero phần mềm cho yaw.
- Bắt đầu lệnh chạy yaw tới góc mục tiêu.
- Bắt đầu lệnh chạy đồng thời ba motor theo ba góc mục tiêu.
- Theo dõi trạng thái motor đang chạy hay đã hoàn tất.
- Tính sai số dựa trên góc cảm biến trước và sau khi chạy.
- Cập nhật vị trí phần mềm của từng motor sau lệnh chạy.
- Nhận callback PWM timer để đếm bước thông qua `TMC2209_UpdateSteps()`.

## 6. Nhóm driver TMC2209

| File | Nhiệm vụ chính |
|---|---|
| `Inc/TMC2209.h` | Khai báo register map, enum trạng thái, handle driver và API điều khiển TMC2209. |
| `Src/TMC2209.c` | Triển khai giao tiếp UART với TMC2209, cấu hình register, điều khiển STEP/DIR/EN và đếm bước bằng PWM timer. |

Driver `TMC2209` chịu trách nhiệm ở mức phần cứng motor stepper. Module này
không biết ý nghĩa lệnh ứng dụng là yaw hay ba trục; nó chỉ nhận yêu cầu chạy
theo số bước, hướng quay và tốc độ.

Các nhiệm vụ chính:

- Ghi/đọc register TMC2209 qua UART datagram có CRC.
- Cấu hình dòng chạy, dòng giữ, microstep và chế độ StealthChop.
- Bật/tắt driver qua chân EN active-low.
- Đặt hướng quay qua chân DIR.
- Chuyển RPM sang tần số STEP.
- Cấu hình timer PWM để phát xung STEP.
- Bắt đầu/dừng motor.
- Đếm số xung đã chạy trong callback timer.
- Đọc trạng thái lỗi, nhiệt, short và StallGuard.

## 7. Nhóm driver AS5600

| File | Nhiệm vụ chính |
|---|---|
| `Inc/as5600.h` | Khai báo register map, cấu hình, dữ liệu đọc, trạng thái nam châm và API AS5600. |
| `Src/as5600.c` | Triển khai đọc/ghi thanh ghi AS5600 qua I2C callback, đọc góc raw/góc đã lọc, trạng thái nam châm, AGC và magnitude. |

Driver `AS5600` làm việc với cảm biến góc từ tính 12-bit. Module này dùng các
callback `i2c_write`, `i2c_read`, `delay_ms` do tầng cao hơn gán vào handle, nhờ
đó có thể dùng trực tiếp hoặc dùng sau mux TCA9548A.

Các nhiệm vụ chính:

- Khởi tạo AS5600 với cấu hình mặc định.
- Ghi/đọc cấu hình thanh ghi `CONF`.
- Đọc góc raw, góc đã lọc, góc theo độ và radian.
- Đọc trạng thái nam châm: chưa có nam châm, quá yếu, quá mạnh hoặc hợp lệ.
- Đọc AGC và độ lớn từ trường.
- Hỗ trợ cấu hình vùng góc `ZPOS`, `MPOS`, `MANG`.
- Hỗ trợ lệnh burn OTP, cần dùng cẩn thận vì không thể hoàn tác.

## 8. Nhóm driver TCA9548A

| File | Nhiệm vụ chính |
|---|---|
| `Inc/tca9548a.h` | Khai báo địa chỉ, kênh mux, handle, slot AS5600 và API quản lý TCA9548A. |
| `Src/tca9548a.c` | Triển khai chọn/tắt kênh mux, đăng ký AS5600 theo kênh, khởi tạo và đọc cảm biến qua mux. |

Driver `TCA9548A` quản lý bộ chia kênh I2C. Module này giúp nhiều AS5600 có
cùng địa chỉ I2C vẫn hoạt động được bằng cách chỉ mở đúng kênh cần đọc.

Các nhiệm vụ chính:

- Khởi tạo TCA9548A và tắt toàn bộ kênh khi bắt đầu.
- Chọn một kênh duy nhất hoặc ghi bitmask nhiều kênh.
- Đọc lại trạng thái bitmask kênh.
- Đăng ký AS5600 vào từng kênh mux.
- Khởi tạo tất cả AS5600 đã đăng ký.
- Đọc một cảm biến trên kênh chỉ định.
- Quét tất cả cảm biến đã đăng ký và lưu trạng thái từng kênh.
- Tắt kênh sau khi đọc để bus I2C ở trạng thái xác định.

## 9. Nhóm cấu hình hệ thống

| File | Nhiệm vụ chính |
|---|---|
| `Inc/my_config.h` | Lưu các thông số cấu hình dùng chung cho motor và timer. |

`my_config.h` hiện chứa các hằng số quan trọng cho TMC2209:

- `TMC2209_TIMER_CLOCK_HZ`: tần số clock cấp cho timer.
- `TMC2209_TIMER_PRESCALER`: prescaler timer đã cấu hình trong CubeMX.
- `TMC2209_MOTOR_STEPS_REV`: số full-step trong một vòng motor.

Khi thay đổi cấu hình timer, microstep cơ bản hoặc loại motor, cần kiểm tra lại
file này để phép đổi từ RPM/góc sang xung STEP vẫn đúng.

## 10. Nhóm module dự phòng hoặc mở rộng

| File | Trạng thái hiện tại | Hướng sử dụng dự kiến |
|---|---|---|
| `Inc/pid.h` | Đã tạo khung, chưa có API. | Dành cho bộ điều khiển PID nếu cần điều khiển kín theo sai số cảm biến. |
| `Src/pid.c` | Đã tạo khung, chưa triển khai logic. | Có thể chứa hàm khởi tạo, reset và tính toán PID. |
| `Inc/trajectory.h` | Đã tạo khung, chưa có API. | Dành cho tính toán quỹ đạo, ramp tốc độ hoặc profile chuyển động. |
| `Src/trajectory.c` | Đã tạo khung, chưa triển khai logic. | Có thể chứa profile tăng/giảm tốc, giới hạn vận tốc và gia tốc. |

Hai nhóm file này chưa tham gia vào luồng chạy hiện tại. Khi bổ sung, nên giữ
chúng độc lập: `trajectory` chỉ tính quỹ đạo, `pid` chỉ tính điều khiển phản
hồi, còn việc quyết định dùng kết quả đó để chạy motor vẫn nên nằm ở
`my_controller`.

## 11. Quan hệ phụ thuộc giữa các module

```text
my_app
├── command
└── my_controller
    ├── TMC2209
    ├── tca9548a
    │   └── as5600
    └── my_config
```

Gợi ý khi phát triển tiếp:

- Thêm lệnh USB mới: ưu tiên sửa `command` để parse dữ liệu, sau đó sửa
  `my_app` để chọn hành động.
- Thêm hành vi điều khiển mới: ưu tiên sửa `my_controller`.
- Thêm chức năng phần cứng TMC2209: sửa `TMC2209`.
- Thêm chức năng đọc cảm biến góc: sửa `AS5600` hoặc `TCA9548A` tùy cảm biến
  đọc trực tiếp hay đọc qua mux.
- Thêm cấu hình hằng số motor/timer: sửa `my_config.h`.

## 12. Quy ước bảo trì

- Mỗi module nên có một file `.h` và một file `.c` cùng tên.
- API public đặt trong header, helper nội bộ để `static` trong source.
- Header chỉ chứa phần cần thiết cho module khác sử dụng.
- Source chịu trách nhiệm triển khai chi tiết và giữ trạng thái nội bộ.
- Không đổi tên API public nếu không thật sự cần, vì các module khác có thể đã
  phụ thuộc vào tên đó.
- Khi thêm module mới, hãy cập nhật README này để người đọc biết file mới làm
  nhiệm vụ gì.
