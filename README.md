# Điều Khiển Bám Quỹ Đạo Cho Robot Di Động Hai Bánh

## Tổng Quan
Dự án này tập trung vào việc thiết kế và đánh giá các bộ điều khiển bám quỹ đạo cho robot di động hai bánh vi sai. Nghiên cứu bao gồm việc phát triển và so sánh hiệu suất của nhiều phương pháp điều khiển như PID, SMC (Điều Khiển Trượt), ASMC (Điều Khiển Trượt Thích Nghi), Fuzzy Logic và Neural Network. Các bộ điều khiển được mô phỏng trong môi trường MATLAB/Simulink và sau đó được triển khai trên phần cứng thực tế thông qua mã nhúng C/C++ trên ESP32.

## Nội Dung Dự Án
- Mô hình hóa động học và động lực học của robot di động hai bánh vi sai
- Thiết kế các bộ điều khiển bám quỹ đạo:
  - Bộ điều khiển PID cổ điển
  - Bộ điều khiển trượt (SMC)
  - Bộ điều khiển trượt thích nghi (ASMC)
  - Bộ điều khiển logic mờ (Fuzzy Logic)
  - Bộ điều khiển mạng nơ-ron (Neural Network)
- Mô phỏng sử dụng MATLAB/Simulink
- Thiết kế mô hình phần cứng trên Solidworks
- Triển khai nhúng sử dụng ESP32
- So sánh hiệu suất giữa kết quả mô phỏng và thực nghiệm

## Cấu Trúc Thư Mục
- `Embedded_code/`: Chứa tất cả các bộ điều khiển cho ESP32
  - `NeuralNetwork.ino`: Triển khai bộ điều khiển mạng nơ-ron
  - `SMC.ino`: Triển khai bộ điều khiển trượt
  - `ASMC.ino`: Triển khai bộ điều khiển trượt thích nghi
  - `Fuzzy.ino`: Triển khai bộ điều khiển logic mờ
  - `PID_backstepping.ino`: Triển khai bộ điều khiển PID với backstepping
  - `GUI_CAR.py`: Giao diện Python để điều khiển và giám sát robot
- `MATLAB/`: Chứa các mô hình mô phỏng cho tất cả các bộ điều khiển
  - `NeuralNetwork/`: Mô phỏng bộ điều khiển mạng nơ-ron
  - `SMC/`: Mô phỏng bộ điều khiển trượt
  - `ASMC/`: Mô phỏng bộ điều khiển trượt thích nghi
  - `Fuzzy/`: Mô phỏng bộ điều khiển logic mờ
  - `PID/`: Mô phỏng bộ điều khiển PID
- `Solidworks/`: Chứa các mô hình CAD 3D của robot
- `docs/`: Tài liệu dự án và các bài báo nghiên cứu
  - `NeuralNetwork.pdf`: Tài liệu về bộ điều khiển mạng nơ-ron
  - `ITCA2024_NeuralNetwork.pdf`: Bài báo nghiên cứu về triển khai mạng nơ-ron
  - `VCCA2024_Fuzzy.pdf`: Bài báo nghiên cứu về triển khai logic mờ

## Yêu Cầu Phần Mềm
- MATLAB R2021B (hoặc cao hơn)
- Arduino IDE 2.xx (để lập trình ESP32)
- Solidworks 2022
- Python 3.x (cho ứng dụng giao diện)

## Yêu Cầu Phần Cứng
- Vi điều khiển ESP32
- Bộ điều khiển động cơ
- Động cơ DC có bộ mã hóa
- Khung cơ khí (theo thiết kế Solidworks)
- Nguồn điện

## Hướng Dẫn Cài Đặt
1. **Cài Đặt ESP32:**
   - Cài đặt board ESP32 của Espressif (phiên bản 2.0.17) trong Arduino IDE
   - Mở một trong các file điều khiển từ thư mục `Embedded_code/` trong Arduino IDE
   - Cập nhật SSID và mật khẩu WiFi trong mã
   - Tải mã lên ESP32

2. **Cài Đặt Giao Diện:**
   - Kết nối ESP32 và máy tính đến cùng một mạng WiFi (2.4GHz)
   - Ghi lại địa chỉ IP do ESP32 cung cấp sau khi kết nối
   - Cập nhật biến `ip_address` trong file `GUI_CAR.py`
   - Chạy ứng dụng giao diện Python

3. **Vận Hành:**
   - Sử dụng giao diện để bắt đầu/dừng robot
   - Giám sát hiệu suất và khả năng bám quỹ đạo trong thời gian thực

## Video Demo
- Xem demo: [Điều Khiển Bám Quỹ Đạo Cho Robot Di Động Hai Bánh](https://www.youtube.com/watch?v=6Xz11GfAjuo&t=38s)

## Lưu Ý Quan Trọng
- ESP32 và laptop phải kết nối cùng một mạng WiFi 2.4GHz
- Đảm bảo cài đặt tất cả các thư viện cần thiết cho ESP32 (WiFi, WebServer, MatrixMath)
- Nên giám sát mức pin trong quá trình hoạt động

## Người Đóng Góp
- Nguyễn Văn A - Phát triển thuật toán điều khiển mạng nơ-ron
- Trần Thị B - Thiết kế mô hình CAD và phần cứng
- Lê Văn C - Triển khai giao diện người dùng và phần mềm đồ họa
- Phạm Thị D - Kiểm thử và đánh giá hiệu suất

## Giấy Phép
Dự án này được phân phối theo giấy phép MIT. Xem file LICENSE để biết thêm chi tiết.
