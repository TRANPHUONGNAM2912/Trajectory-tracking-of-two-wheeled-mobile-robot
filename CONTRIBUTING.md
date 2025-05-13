# Đóng Góp Cho Dự Án Robot Di Động Hai Bánh

Cảm ơn bạn đã quan tâm đến việc đóng góp cho dự án Điều Khiển Bám Quỹ Đạo Cho Robot Di Động Hai Bánh! Tài liệu này cung cấp các hướng dẫn và quy tắc để giúp bạn đóng góp hiệu quả.

## Mục Lục
- [Quy Tắc Ứng Xử](#quy-tắc-ứng-xử)
- [Bắt Đầu](#bắt-đầu)
- [Cách Đóng Góp](#cách-đóng-góp)
- [Hướng Dẫn Viết Mã](#hướng-dẫn-viết-mã)
- [Hướng Dẫn Viết Thông Điệp Commit](#hướng-dẫn-viết-thông-điệp-commit)
- [Quy Trình Pull Request](#quy-trình-pull-request)
- [Cài Đặt Môi Trường Phát Triển](#cài-đặt-môi-trường-phát-triển)

## Quy Tắc Ứng Xử

Chúng tôi mong muốn tất cả người đóng góp đều tôn trọng, cân nhắc và hợp tác. Vui lòng đảm bảo bạn:
- Sử dụng ngôn ngữ thân thiện và hòa nhập
- Tôn trọng các quan điểm và kinh nghiệm khác nhau
- Chấp nhận phê bình mang tính xây dựng một cách lịch sự
- Tập trung vào những gì tốt nhất cho dự án
- Thể hiện sự đồng cảm với các thành viên khác trong cộng đồng

## Bắt Đầu

1. Fork repository trên GitHub
2. Clone repository đã fork về máy tính của bạn
3. Thiết lập môi trường phát triển (xem [Cài Đặt Môi Trường Phát Triển](#cài-đặt-môi-trường-phát-triển))
4. Tạo nhánh mới cho tính năng hoặc sửa lỗi của bạn

## Cách Đóng Góp

Có nhiều cách để đóng góp vào dự án này:

1. **Phát Triển Bộ Điều Khiển Mới**: Thêm thuật toán điều khiển mới cho bám quỹ đạo
2. **Cải Thiện Mã Hiện Có**: Nâng cao hiệu suất, khả năng đọc hoặc tài liệu
3. **Sửa Lỗi**: Giải quyết các vấn đề trong triển khai hiện tại
4. **Tài Liệu**: Cải thiện hoặc thêm tài liệu
5. **Kiểm Thử**: Phát triển hoặc cải thiện quy trình kiểm thử cho các bộ điều khiển
6. **Hỗ Trợ Phần Cứng**: Thêm hỗ trợ cho các thành phần phần cứng mới

## Hướng Dẫn Viết Mã

### Nguyên Tắc Chung
- Viết mã đơn giản và dễ đọc
- Chú thích mã một cách phù hợp
- Tuân theo mẫu và phong cách viết mã đã được thiết lập
- Đảm bảo mã của bạn được kiểm thử kỹ lưỡng

### Nguyên Tắc Arduino/C++
- Sử dụng tên biến và hàm có ý nghĩa
- Nhóm các biến liên quan với nhau
- Tổ chức các hàm một cách hợp lý
- Thêm chú thích phù hợp cho các phần phức tạp
- Giữ cho các hàm tập trung vào một nhiệm vụ duy nhất
- Tránh sử dụng các số ma thuật; sử dụng hằng số có tên
- Sử dụng thụt đầu dòng nhất quán (2 hoặc 4 dấu cách)

### Nguyên Tắc MATLAB
- Sử dụng tên biến mô tả
- Chú thích các phần của mã một cách phù hợp
- Tổ chức mã thành các hàm hợp lý
- Bao gồm chú thích tiêu đề cho mỗi hàm
- Sử dụng thụt đầu dòng nhất quán
- Vector hóa các phép toán khi có thể để tăng hiệu suất

## Hướng Dẫn Viết Thông Điệp Commit

Vui lòng tuân thủ các nguyên tắc sau cho thông điệp commit:
- Sử dụng thì hiện tại ("Thêm tính năng" không phải "Đã thêm tính năng")
- Sử dụng dạng mệnh lệnh ("Di chuyển con trỏ đến..." không phải "Di chuyển con trỏ đến...")
- Giới hạn dòng đầu tiên tối đa 72 ký tự
- Tham chiếu đến các issues và pull requests sau dòng đầu tiên
- Cân nhắc bắt đầu thông điệp commit với biểu tượng cảm xúc phù hợp:
  - 🚀 cho tính năng mới
  - 🐛 cho sửa lỗi
  - 📝 cho tài liệu
  - 🎨 cho định dạng mã
  - ♻️ cho tái cấu trúc
  - 🧪 cho kiểm thử

## Quy Trình Pull Request

1. Tạo thay đổi của bạn trong một nhánh git mới:
   ```
   git checkout -b feature/tinh-nang-moi main
   ```
2. Commit các thay đổi của bạn bằng thông điệp commit mô tả
3. Đẩy nhánh của bạn lên GitHub
4. Gửi pull request tới nhánh `main`
5. Đảm bảo mô tả PR của bạn mô tả rõ ràng các thay đổi
6. Bao gồm số issue liên quan trong mô tả PR
7. Cập nhật README.md hoặc tài liệu nếu cần thiết

## Cài Đặt Môi Trường Phát Triển

### Phát Triển Arduino
1. Cài đặt Arduino IDE 2.xx
2. Cài đặt hỗ trợ board ESP32 (Espressif 2.0.17) qua Trình quản lý board
3. Cài đặt các thư viện cần thiết:
   - WiFi
   - WebServer
   - MatrixMath
4. Thiết lập cài đặt board:
   - Board: "ESP32 Dev Module"
   - Tốc độ Upload: "921600"
   - Tần số CPU: "240MHz"
   - Tần số Flash: "80MHz"
   - Chế độ Flash: "QIO"
   - Kích thước Flash: "4MB"
   - Sơ đồ phân vùng: "Default"

### Phát Triển MATLAB
1. Cài đặt MATLAB R2021b hoặc cao hơn
2. Đảm bảo Simulink được cài đặt
3. Các toolbox cần thiết:
   - Control System Toolbox
   - Simscape
   - Simscape Multibody
   - Simulink Control Design

### Phát Triển Python (cho GUI)
1. Cài đặt Python 3.x
2. Cài đặt các gói cần thiết:
   ```
   pip install numpy matplotlib tkinter requests
   ```

Cảm ơn bạn đã đóng góp cho dự án! 