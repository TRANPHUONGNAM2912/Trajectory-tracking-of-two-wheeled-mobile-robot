#include <WiFi.h>
#include <WebServer.h>
#include <MatrixMath.h>

// Cấu trúc lưu trữ dữ liệu liên quan đến robot
struct RobotData {
  float x = 0, y = 0, theta = 0;
  float toadox = 0, toadoy = 0;
  float vthuc = 0, wthuc = 0;
  float ex = 0, ey = 0, eth = 0;
};

// Cấu trúc lưu trữ dữ liệu liên quan đến PID
struct PIDData {
  float kp = 0.02;
  float ki = 0.00015;
  float kd = 0.01;
  float error_now1 = 0, error_prev1 = 0, integ_now1 = 0, integ_prev1 = 0;
  float error_now2 = 0, error_prev2 = 0, integ_now2 = 0, integ_prev2 = 0;
};

// Cấu trúc lưu trữ dữ liệu liên quan đến quỹ đạo
struct TrajectoryData {
  float xd = 0, yd = 0;
  float xd_dot = 0, yd_dot = 0;
  float xd_2dot = 0, yd_2dot = 0;
  float Thetad = 0, Theta0 = 0;
  float x1d = 0, y1d = 0;
  float x1d_dot = 0, y1d_dot = 0;
  float wd = 0;
  float vr = 0, wr = 0;
  float Vc = 0, Wc = 0;
  float xdat = 0, ydat = 0;
};

// Khởi tạo các cấu trúc
RobotData robot;
PIDData pid;
TrajectoryData traj;

mtx_type Td[2][2];
mtx_type TdInv[2][2];

// Thông số WiFi
const char* ssid = "VinhUni";
const char* password = "1111111111";

// Timer
hw_timer_t *timer1 = NULL;

// Pins động cơ
const int motor1Pin1 = 2;
const int motor1Pin2 = 4;
const int enable1Pin = 12;
const int motor2Pin1 = 19;
const int motor2Pin2 = 18;
const int enable2Pin = 15;

// Thiết lập PWM
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;

// Pins encoder
const int encoA_motor1 = 26;
const int encoB_motor1 = 13;
const int encoA_motor2 = 32;
const int encoB_motor2 = 14;

// Thông số robot
float quangduong1 = 0;
float quangduong2 = 0;
float quangduongthuc = 0;
const float l = 0.38;  // Khoảng cách giữa hai bánh xe
float vantoc = 0;
volatile unsigned long count = 0;
unsigned long count_prev = 0;
float Theta_now1; 
float Theta_prev1 = 0;
float Theta_now2; 
float Theta_prev2 = 0;
float Vl_output;
float RPM_output1, RPM_input1;
float Vr_output;
float RPM_output2, RPM_input2;
float dt1;  
float dt2;                     
 
unsigned long t_prev = 0;
unsigned long t_now1;
unsigned long t_prev1 = 0;
unsigned long t_now2;
unsigned long t_prev2 = 0;
volatile long dem_motor1 = 0;
volatile long dem_motor2 = 0;
volatile long dem_motor11 = 0;
volatile long dem_motor22 = 0;
volatile long dem_motor111 = 0;
volatile long dem_motor222 = 0;
int PWMval1 = 0;
int PWMval2 = 0;
const float Vmax = 24;      
const float Vmin = 0;
float V1 = 0;
float V2 = 0;
float globalVL = 0;
float globalVR = 0;
float globalVLL = 0;
float globalVRR = 0;
float v = 0, w = 0;
unsigned long prevTime = 0;
float leftWheelSpeedpre = 0;
float rightWheelSpeedpre = 0;
float omega = 0.3;
float R = 125;
float t = 0;
const float b = 0.03;
float Kx = 1.15, Ky = 2.1, Kth = 1.2;
const float pi = 3.14159265359;
const float d = 0.2;
bool check = false;

WebServer server(80); 

// Hàm ngắt xử lý encoder motor 1
void IRAM_ATTR dem_xung_motor1() {
  if (digitalRead(encoB_motor1) == HIGH) {
    dem_motor1++;
    dem_motor11++;
  } else {
    dem_motor1++;
    dem_motor11--;
  }
}

// Hàm ngắt xử lý encoder motor 2
void IRAM_ATTR dem_xung_motor2() {
  if (digitalRead(encoB_motor2) == HIGH) {
    dem_motor2++;
    dem_motor22++;
  } else {
    dem_motor2++;
    dem_motor22--;
  }
}

// Hàm ngắt timer
void IRAM_ATTR onTimer1() {
  count++;
}

void setup() {
  Serial.begin(56000); 
  
  // Cấu hình pins động cơ
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  
  // Cấu hình PWM
  ledcSetup(pwmChannel, freq, resolution);
  ledcSetup(pwmChannel + 1, freq, resolution);
  ledcAttachPin(enable1Pin, pwmChannel);
  ledcAttachPin(enable2Pin, pwmChannel + 1);

  // Cấu hình encoder
  pinMode(encoA_motor1, INPUT);
  pinMode(encoB_motor1, INPUT);
  pinMode(encoA_motor2, INPUT);
  pinMode(encoB_motor2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoA_motor1), dem_xung_motor1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoA_motor2), dem_xung_motor2, RISING);
  
  // Cấu hình timer
  timer1 = timerBegin(0, 80, true); 
  timerAttachInterrupt(timer1, &onTimer1, true); 
  timerAlarmWrite(timer1, 150000, true); 
  timerAlarmEnable(timer1); 

  // Kết nối WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Cấu hình server
  server.on("/", handleRoot);
  server.on("/start", handleStart);
  server.on("/stop", stop);

  server.begin();
  Serial.println("HTTP server started");
  Serial.println("CLEARDATA"); // Xóa tất cả dữ liệu Excel
  Serial.println("LABEL,Date,Time,t,x,y,ex,ey,eth,vd,wd,vt,wt");
}

void loop() {
  server.handleClient();
  tinhtoado();

  // Dừng động cơ nếu không có tín hiệu
  if(globalVLL == 0) {
    ledcWrite(pwmChannel + 1, 0);
  }
  if(globalVRR == 0) {
    ledcWrite(pwmChannel, 0);
  }
  
  // Xử lý khi có tín hiệu timer
  if (count > count_prev) {
    // Ghi dữ liệu đến Excel nếu t nằm trong khoảng hợp lệ
    if(t > 0.01 && t < 30.1) {
      sendDataToExcel();
    }
    
    quydao();
    
    // Tính toán thời gian
    t_now1 = millis();
    t_now2 = millis();
    dt1 = (t_now1 - t_prev1);
    dt2 = (t_now2 - t_prev2);
    
    // Thực hiện PID
    PID1();
    PID2();
    
    // Cập nhật vận tốc thực
    robot.vthuc = (Vl_output + Vr_output) / 2;
    robot.wthuc = (Vr_output - Vl_output) / l;
    
    // Cập nhật thời gian trước đó
    t_prev1 = t_now1;
    t_prev2 = t_now2;
    count_prev = count;
  }
}

// Hàm gửi dữ liệu đến Excel
void sendDataToExcel() {
  Serial.print("DATA,DATE,TIME,"); 
  Serial.print(t); 
  Serial.print(",");
  Serial.print(robot.toadox); 
  Serial.print(",");
  Serial.print(robot.toadoy); 
  Serial.print(",");
  Serial.print(robot.ex * 100); 
  Serial.print(",");
  Serial.print(robot.ey * 100); 
  Serial.print(",");
  Serial.print(robot.eth); 
  Serial.print(",");
  Serial.print(traj.Vc); 
  Serial.print(",");
  Serial.print(traj.Wc); 
  Serial.print(",");
  Serial.print(robot.vthuc); 
  Serial.print(",");
  Serial.print(robot.wthuc); 
  Serial.println(",");
}

// Hàm tính toán tọa độ robot
void tinhtoado() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;
  
  // Tính toán vận tốc bánh xe
  float leftWheelSpeed = (dem_motor22 / 249.6) * pi * d;
  float rightWheelSpeed = (dem_motor11 / 249.6) * pi * d;
  
  // Tính toán v và w
  v = (rightWheelSpeed + leftWheelSpeed) / 2;
  w = (rightWheelSpeed - leftWheelSpeed) / l;
  
  // Cập nhật vị trí robot
  robot.theta = robot.theta + w;
  robot.x = robot.x + v * cos(robot.theta + (w / 2));
  robot.y = robot.y + v * sin(robot.theta + (w / 2));

  robot.toadox = robot.x * 100;
  robot.toadoy = robot.y * 100;
  
  // Reset bộ đếm encoder
  dem_motor11 = 0;
  dem_motor22 = 0;
  prevTime = currentTime;
}

// Xử lý yêu cầu HTTP root
void handleRoot() {
  String motorData = "{\"toadox\":" + String(robot.toadox) +
                     ",\"toadoy\":" + String(robot.toadoy) +
                     ",\"xdat\":" + String(traj.xdat) +
                     ",\"ydat\":" + String(traj.ydat) +
                     ",\"theta\":" + String(robot.theta) +
                     ",\"Thetad\":" + String(traj.Thetad) +
                     ",\"Vc\":" + String(traj.Vc) +
                     ",\"Wc\":" + String(traj.Wc) +
                     ",\"ex\":" + String(robot.ex * 100.0) +
                     ",\"ey\":" + String(robot.ey * 100.0) +
                     ",\"eth\":" + String(robot.eth) +
                     ",\"t\":" + String(t) +
                     "}";
  server.send(200, "application/json", motorData);
}

// Xử lý yêu cầu bắt đầu
void handleStart() {
  check = true;
}

// Hàm tính toán quỹ đạo
void quydao() {
  if(check == true) {
    // Tính toán tọa độ mong muốn
    traj.xdat = R * cos(omega * t);
    traj.ydat = R * sin(omega * t);
    traj.xd = (R / 100) * cos(omega * t);
    traj.yd = (R / 100) * sin(omega * t);
    
    // Tính toán đạo hàm đầu tiên
    traj.xd_dot = -(R / 100) * omega * sin(omega * t);
    traj.yd_dot = (R / 100) * omega * cos(omega * t);
    
    // Tính toán đạo hàm thứ 2
    traj.xd_2dot = -(R / 100) * omega * omega * cos(omega * t);
    traj.yd_2dot = -(R / 100) * omega * omega * sin(omega * t);
    
    // Điểm tham chiếu
    float x0d_dot = -(R / 100) * omega * sin(omega);
    float y0d_dot = (R / 100) * omega * cos(omega);
    
    // Tính góc Theta0 và Thetad
    traj.Theta0 = atan2(y0d_dot, x0d_dot) - omega;
    traj.Thetad = omega * t + traj.Theta0;
    
    // Tính toán vị trí mong muốn x1d, y1d
    traj.x1d = traj.xd + b * cos(traj.Thetad);
    traj.y1d = traj.yd + b * sin(traj.Thetad);
    
    // Tính toán vận tốc góc mong muốn
    traj.wd = (traj.yd_2dot * traj.xd_dot - traj.xd_2dot * traj.yd_dot) / 
             (traj.xd_dot * traj.xd_dot + traj.yd_dot * traj.yd_dot);
    
    // Tính toán vận tốc tuyến tính mong muốn
    traj.x1d_dot = traj.xd_dot - b * traj.wd * sin(traj.Thetad);
    traj.y1d_dot = traj.yd_dot + b * traj.wd * cos(traj.Thetad);
    
    // Ma trận chuyển đổi Td
    Td[0][0] = cos(traj.Thetad);
    Td[0][1] = -b * sin(traj.Thetad);
    Td[1][0] = sin(traj.Thetad);
    Td[1][1] = b * cos(traj.Thetad);
    
    // Tính ma trận nghịch đảo
    Matrix.Invert((mtx_type*)Td, 2);
    Matrix.Copy((mtx_type*)Td, 2, 2, (mtx_type*)TdInv);
    
    // Tính toán vận tốc tham chiếu
    traj.vr = TdInv[0][0] * traj.x1d_dot + TdInv[0][1] * traj.y1d_dot;
    traj.wr = TdInv[1][0] * traj.x1d_dot + TdInv[1][1] * traj.y1d_dot;
    
    // Vị trí thực tế của điểm tham chiếu trên robot
    float X1 = robot.x + b * cos(robot.theta);
    float Y1 = robot.y + b * sin(robot.theta);

    // Tính sai số vị trí
    robot.ex = traj.xd - robot.x;
    robot.ey = traj.yd - robot.y;
    robot.eth = traj.Thetad - robot.theta;

    // Chuyển sai số sang hệ tọa độ robot
    float xe = cos(robot.theta) * robot.ex + sin(robot.theta) * robot.ey;
    float ye = -sin(robot.theta) * robot.ex + cos(robot.theta) * robot.ey;
    float thetae = robot.eth;

    // Tính toán tín hiệu điều khiển
    traj.Vc = traj.vr * cos(thetae) + Kx * xe; 
    traj.Wc = traj.wr + traj.vr * Ky * ye + Kth * sin(thetae);

    // Tính toán tín hiệu điều khiển cho từng bánh xe
    globalVRR = (2 * traj.Vc + (l * traj.Wc)) / 2;
    globalVLL = 2 * traj.Vc - globalVRR;
    
    // Tăng thời gian
    t = t + 0.15;
  }
}

// Dừng robot
void stop() {
  check = false;
  globalVRR = 0;
  globalVLL = 0;
  RPM_input1 = 0;
  RPM_input2 = 0;
  server.send(200, "text/html", "stop");
}

// Tiếp tục di chuyển
void continuee() {
  String bankinh_str = server.arg("bankinh");
  R = bankinh_str.toFloat();
  String omega_str = server.arg("omega");
  omega = omega_str.toFloat();
  String kx_str = server.arg("kx");
  Kx = kx_str.toFloat();
  String ky_str = server.arg("ky");
  Ky = ky_str.toFloat();
  String ktheta_str = server.arg("ktheta");
  Kth = ktheta_str.toFloat();
  server.send(200, "text/html", "continuee");
}

// Reset robot
void reset() {
  server.send(200, "text/html", "reset");
}

// Điều khiển PID cho motor 1
void PID1() {
  // Tính toán RPM đầu vào từ vận tốc mong muốn
  RPM_input1 = abs((globalVRR / (d * pi)) * 60);
  
  // Tính vị trí góc hiện tại
  Theta_now1 = dem_motor1 / 249.6;
  
  // Tính RPM đầu ra
  RPM_output1 = (Theta_now1 - Theta_prev1) / (dt1 / 1000.0) * 60;
  
  // Chuyển đổi từ RPM sang vận tốc tuyến tính
  Vr_output = (RPM_output1 / 60) * (d * pi);
  
  // Tính sai số
  pid.error_now1 = RPM_input1 - RPM_output1;
  
  // Tính thành phần tích phân
  pid.integ_now1 = pid.integ_prev1 + (dt1 * (pid.error_now1 + pid.error_prev1) / 2);

  // Tính tín hiệu điều khiển PID
  V1 = pid.kp * pid.error_now1 + pid.ki * pid.integ_now1 + (pid.kd * (pid.error_now1 - pid.error_prev1) / dt1);
  
  // Giới hạn điện áp điều khiển
  if (V1 > Vmax) {
    V1 = Vmax;
    pid.integ_now1 = pid.integ_prev1; // Chống tích lũy tích phân
  }
  if (V1 < Vmin) {
    V1 = Vmin;
    pid.integ_now1 = pid.integ_prev1;
  }
  
  // Chuyển đổi sang giá trị PWM
  PWMval1 = int(255 * abs(V1) / Vmax);
  if (PWMval1 > 255) {
    PWMval1 = 255;
  }
  
  // Điều khiển chiều quay
  if(globalVRR > 0) {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    ledcWrite(pwmChannel, PWMval1);
  }
  if(globalVRR < 0) {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel, PWMval1);
  }
  
  // Cập nhật các giá trị
  Theta_prev1 = Theta_now1;
  pid.integ_prev1 = pid.integ_now1;
  pid.error_prev1 = pid.error_now1;
}

// Điều khiển PID cho motor 2
void PID2() {
  // Tính toán RPM đầu vào từ vận tốc mong muốn
  RPM_input2 = abs((globalVLL / (d * pi)) * 60);
  
  // Tính vị trí góc hiện tại
  Theta_now2 = dem_motor2 / 249.6;
  
  // Tính RPM đầu ra
  RPM_output2 = (Theta_now2 - Theta_prev2) / (dt2 / 1000.0) * 60;
  
  // Chuyển đổi từ RPM sang vận tốc tuyến tính
  Vl_output = (RPM_output2 / 60) * (d * pi);
  
  // Tính sai số
  pid.error_now2 = RPM_input2 - RPM_output2;
  
  // Tính thành phần tích phân
  pid.integ_now2 = pid.integ_prev2 + (dt2 * (pid.error_now2 + pid.error_prev2) / 2);

  // Tính tín hiệu điều khiển PID
  V2 = pid.kp * pid.error_now2 + pid.ki * pid.integ_now2 + (pid.kd * (pid.error_now2 - pid.error_prev2) / dt2);
  
  // Giới hạn điện áp điều khiển
  if (V2 > Vmax) {
    V2 = Vmax;
    pid.integ_now2 = pid.integ_prev2; // Chống tích lũy tích phân
  }
  if (V2 < Vmin) {
    V2 = Vmin;
    pid.integ_now2 = pid.integ_prev2;
  }
  
  // Chuyển đổi sang giá trị PWM
  PWMval2 = int(255 * abs(V2) / Vmax);
  if (PWMval2 > 255) {
    PWMval2 = 255;
  }
  
  // Điều khiển chiều quay
  if(globalVLL > 0) {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    ledcWrite(pwmChannel + 1, PWMval2);
  }
  if(globalVLL < 0) {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    ledcWrite(pwmChannel + 1, PWMval2);
  }

  // Cập nhật các giá trị
  Theta_prev2 = Theta_now2;
  pid.integ_prev2 = pid.integ_now2;
  pid.error_prev2 = pid.error_now2;
}