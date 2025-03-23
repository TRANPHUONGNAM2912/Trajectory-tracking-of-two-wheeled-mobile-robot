# Thiết kế bộ điều khiển bám quỹ đạo cho robot di động 2 bánh vi sai
## Tổng quan  
Dự án này tập trung vào việc thiết kế và đánh giá các bộ điều khiển bám quỹ đạo cho robot di động 2 bánh vi sai. Nghiên cứu bao gồm việc phát triển và so sánh hiệu suất của nhiều phương pháp điều khiển như PID, SMC (Sliding Mode Control), Fuzzy Logic và Neural Network. Các bộ điều khiển được mô phỏng trong môi trường MATLAB/Simulink và sau đó được nhúng xuống phần cứng thực tế thông qua mã nguồn C. Điều khiển và theo dõi quỹ đạo Robot từ xa thông qua Wifi.
## Nội dung chính  
-Mô hình hóa động học, động lực học của robot di động 2 bánh vi sai  
-Thiết kế các bộ điều khiển bám quỹ đạo:  
+Bộ điều khiển PID cổ điển  
+Bộ điều khiển trượt SMC (Sliding Mode Control)  
+Bộ điều khiển mờ Fuzzy Logic  
+Bộ điều khiển mạng nơ-ron Neural Network  
-Mô phỏng sử dụng MATLAB/Simulink  
-Thiết kế mô hình phần cứng trên Solidworks  
-Nhúng code vào phần cứng sử dụng C    
-So sánh hiệu suất mô phỏng và thực nghiệm    
## Phần mềm
-Matlab R2021B(hoặc cao hơn)  
-Arduino IDE 2.xx(lập trình cho ESP32)  
-Solidworks 2022
## Thực nghiệm
-Thay đổi ssid và password trong file **ino** và nạp xuống ESP32  
-Lấy địa chỉ ip do ESP32 cung cấp khi đã kết nối được Wifi và điền vào ip_address trong file **python**  
-Khởi chạy file **python** và nhấn Start  
![image](https://github.com/user-attachments/assets/2832ba36-b5d9-4cb8-bb4a-0e6739e8c6ab)


-Link video: https://www.youtube.com/watch?v=6Xz11GfAjuo&t=38s
## Lưu ý
-Tải borad ESP32 by Esppressif 2.0.17  
-ESP32 và Laptop phải kết nối chung 1 Wifi có tần số 2.4Ghz
