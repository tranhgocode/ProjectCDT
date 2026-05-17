## Các dự định 
- làm điều khiển 4 timer PWM riêng biệt 
- gửi đầu vào là góc theta (dích đến), và vận tốc (rad/s) để chạy đến đó 
- từ vận tốc đó chia ra băm tần số -> điều khiển step 
- xong sẽ feed back lại 
- sẽ có 1 theta ở vị trí 1 kiếm xuống

## PI 
- góc thức tế -> góc mục tiêu 
- điều chỉnh tốc độ động cơ để biết nó đi nhanh hay chậm đến mục tiêu 

## đề xuất 
- nên từ rad/s chuyển thành rpm luôn 
- Công thức: `rpm = (rad/s) x 60 / 2(pi)`

## yêu cầu 
- giao tiếp com ảo để chứa các trạng thái hoạt động của project
{start, stop, running, error}