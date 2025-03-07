#include <WiFi.h>
#include <WebServer.h>
#include <MatrixMath.h>

mtx_type Td[2][2];
mtx_type TdInv[2][2];
mtx_type u[2][1];
const char* ssid = "";
const char* password = "";
WebServer server(80); 
float kp1=0.1;   
float ki1=0.1;  
// float kd=0.01; 
hw_timer_t *timer1 = NULL;

int motor1Pin1 = 2; 
int motor1Pin2 = 4; 
int enable1Pin = 12;

int motor2Pin1 = 19; 
int motor2Pin2 = 18; 
int enable2Pin = 15; 

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;

int encoA_motor1 = 26; 
int encoB_motor1 = 13; 
int encoA_motor2 = 32; 
int encoB_motor2 = 14; 

float quangduong1= 0;
float quangduong2= 0;
float quangduongthuc= 0;
float l = 0.38;
float vantoc = 0;
float vthuc = 0;
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
int PWMval1=0;
int PWMval2=0;
float Vc_dot, Wc_dot;       // Biến tỷ lệ thay đổi
float K1=30, K2=30;               // Biến điều chỉnh
float n1=1, n2=1;               // Biến khuếch đại
float m0, I0,m,I,Iw=4,mc=4,mw=5,d=0.2,Ic=6,Im=1; // Các hằng số vật lý
float s1, s2;               // Biến trạng thái
float V_dot, W_dot;         // Biến thay đổi tốc độ
float ueq1, ueq2;           // Biến điều khiển tương đương
float ud1, ud2;             // Biến điều khiển rối
float u1, u2;               // Biến điều khiển tổng
float vd_dot, wd_dot;       // Biến thay đổi tốc độ tương ứng
float vdat, wdat;               // Biến tốc độ tương ứng
float vd_dot_prev, wd_dot_prev; // Biến tốc độ trước đó
float Vmax = 24;      
float Vmin = 0;
float V1 = 0;
float V2 = 0;
float error_now1, error_prev1 = 0, integ_now1, integ_prev1 = 0;
float error_now2, error_prev2 = 0, integ_now2, integ_prev2 = 0;
float error_now11, error_prev11 = 0, integ_now11, integ_prev11 = 0;
float error_now22, error_prev22 = 0, integ_now22, integ_prev22 = 0;
float globalVL = 0;
float globalVR = 0;
float globalVLL = 0;
float globalVRR = 0;
float v = 0 , w = 0;
float x = 0, y = 0, toadox = 0, toadoy = 0;
float theta = 0;
unsigned long prevTime = 0;
float leftWheelSpeedpre = 0;
float rightWheelSpeedpre = 0;
float omega = 0.1;
float R = 150;
float xd=0,yd=0,xd_dot=0,yd_dot=0,xd_2dot,yd_2dot=0,Theta=0,x1d=0,y1d=0,wd=0,x1d_dot=0,y1d_dot=0,
V=0,W=0,VL=0,VR=0,Theta0=0,Thetad=0,X1=0,Y1=0,ex=0,ey=0,eth=0,xe=0,ye=0,thetae=0,Vc=0,Wc=0,vt=0,vp=0,
x0d_dot=0,y0d_dot=0,xdat=0,ydat=0;
float t =0;
float b = 0.03,Kx=1,Ky=2,Kth=1.2,pi=3.14;
float wthuc=0;
float vr =0;
float wr=0;
bool check = false;
float Vc_prev=0,Wc_prev=0,vd=0;
float deltaTime;
float RPM_input3;    // Biến đầu vào RPM cho PID1
float Theta_now3;    // Góc hiện tại của động cơ cho PID1
float RPM_output3;   // Biến đầu ra RPM cho PID1
// float Vr_output;     // Tốc độ thực tế đầu ra cho PID1
float error_now3;    // Lỗi hiện tại cho PID1
float integ_now3;    // Tích phân hiện tại cho PID1
float V3;            // Giá trị điều khiển cho PID1
// int PWMval3;         // Giá trị PWM cho PID1
float Theta_prev3;   // Góc trước đó của động cơ cho PID1
float integ_prev3;   // Tích phân trước đó cho PID1
float error_prev3;   // Lỗi trước đó cho PID1

// Biến toàn cục cho PID4
float RPM_input4;    // Biến đầu vào RPM cho PID4
float Theta_now4;    // Góc hiện tại của động cơ cho PID4
float RPM_output4;   // Biến đầu ra RPM cho PID4
// float Vl_output;     // Tốc độ thực tế đầu ra cho PID4
float error_now4;    // Lỗi hiện tại cho PID4
float integ_now4;    // Tích phân hiện tại cho PID4
float V4;            // Giá trị điều khiển cho PID4
// int PWMval4;         // Giá trị PWM cho PID4
float Theta_prev4;   // Góc trước đó của động cơ cho PID4
float integ_prev4;   // Tích phân trước đó cho PID4
float error_prev4;   // Lỗi trước đó cho PID4

// Biến toàn cục chung
// float globalVRR;     // Tốc độ quay toàn cục cho PID1
// float globalVLL;     // Tốc độ quay toàn cục cho PID4
float kp = 0.02;      // Hệ số điều khiển P
float ki = 0.00015;      // Hệ số điều khiển I
float kd = 0.01;     // Hệ số điều khiển D
// float dt1 = 100.0;   // Thời gian lấy mẫu cho PID1 (ms)
// float dt3 = 100.0;   // Thời gian lấy mẫu cho PID4 (ms)
// float Vmax = 255;    // Giá trị tối đa cho điều khiển
// float Vmin = 0;      // Giá trị tối thiểu cho điều khiển

void IRAM_ATTR dem_xung_motor1() {
  if (digitalRead(encoB_motor1) == HIGH) {
    dem_motor1++;
    dem_motor11++;
    dem_motor111++;
  } else {
    dem_motor1++;
    dem_motor11--;
    dem_motor111--;
  }
}

void IRAM_ATTR dem_xung_motor2() {
  if (digitalRead(encoB_motor2) == HIGH) {
    dem_motor2++;
    dem_motor22++;
    dem_motor222++;
  } else {
    dem_motor2++;
    dem_motor22--;
    dem_motor222--;
  }
}
void IRAM_ATTR onTimer1() {
  count++;
}

void setup() {
  Serial.begin(115200); 
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  ledcSetup(pwmChannel, freq, resolution);
  ledcSetup(pwmChannel + 1, freq, resolution);
  ledcAttachPin(enable1Pin, pwmChannel);
  ledcAttachPin(enable2Pin, pwmChannel + 1);

  pinMode(encoA_motor1, INPUT);
  pinMode(encoB_motor1, INPUT);
  pinMode(encoA_motor2, INPUT);
  pinMode(encoB_motor2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoA_motor1), dem_xung_motor1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoA_motor2), dem_xung_motor2, RISING);
  timer1 = timerBegin(0, 80, true); 
  timerAttachInterrupt(timer1, &onTimer1, true); 
  timerAlarmWrite(timer1, 150000, true); 
  timerAlarmEnable(timer1); 

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    // delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/start", handleStart);
  server.on("/stop", stop);
  server.on("/reset", reset);
  server.on("/continuee", continuee);
  server.begin();
  Serial.println("HTTP server started");
}
void loop() {
  server.handleClient();
  tinhtoado();
  Serial.println(vd,10);
  // Serial.println(wd);
  // Serial.println(Kth);
  // Serial.println(R);
  // Serial.println(omega);

  if(globalVLL==0){
    ledcWrite(pwmChannel+1, 0);
  }
  if(globalVRR==0){
    ledcWrite(pwmChannel, 0);
  }
  if (count > count_prev) {
    smc();
    // Serial.println("input1");
    // Serial.println(dem_motor111);
    // Serial.println("input2");
    // Serial.println(dem_motor222);
    // Serial.println("input1");
    // Serial.println(RPM_input1);
    // Serial.println("input2");
    // Serial.println(RPM_input2);
    // Serial.println(check);
    quydao();
    t_now1 = millis();
    t_now2 = millis();
    dt1 = (t_now1 - t_prev1);
    dt2 = (t_now2 - t_prev2);
    A1();
    A2();
    PID1();
    PID2();
    vthuc = (Vl_output+Vr_output)/2;
    wthuc = (Vr_output-Vl_output)/l;
    t_prev1 = t_now1;
    t_prev2 = t_now2;
    count_prev = count;
  }
}

void tinhtoado(){
  unsigned long currentTime = millis();
  deltaTime = (currentTime - prevTime) / 1000.0;

  float leftWheelSpeed = (dem_motor22 / 249.6) * 3.14*0.2;
  float rightWheelSpeed = (dem_motor11 / 249.6) * 3.14*0.2;
  // Serial.println("input1");
  //   Serial.println(leftWheelSpeed);
  //   Serial.println("input2");

  v = (rightWheelSpeed + leftWheelSpeed)/2;
  w = (rightWheelSpeed - leftWheelSpeed)/l;
  theta = theta + w;
  x = x + v*cos(theta + (w/2));
  y = y + v*sin(theta + (w/2));

  toadox = x*100;
  toadoy = y*100;
  dem_motor11 = 0;
  dem_motor22 = 0;
  prevTime = currentTime;
}

void handleRoot() {
  String motorData = "{\"tocdomotor\":" + String(vthuc) +
                     ",\"wthuc\":" + String(wthuc) +
                     ",\"toadox\":" + String(toadox) +
                     ",\"toadoy\":" + String(toadoy) +
                     ",\"xdat\":" + String(xdat) +
                     ",\"ydat\":" + String(ydat) +
                     ",\"theta\":" + String(theta) +
                     ",\"Thetad\":" + String(Thetad) +
                     ",\"Vc\":" + String(Vc) +
                     ",\"Wc\":" + String(Wc) +
                     ",\"ex\":" + String(ex) +
                     ",\"ey\":" + String(ey) +
                     ",\"eth\":" + String(eth) +
                     ",\"t\":" + String(t) +
                     "}";
  server.send(200, "application/json", motorData);
}

void handleStart() {
  // String vantocgoc_str = server.arg("vantocgoc");
  // String vantoctrungbinh_str = server.arg("vantoctrungbinh");
  // vantocgoc = vantocgoc_str.toFloat();
  // vantoctrungbinh = vantoctrungbinh_str.toFloat();
  // vantoc = vantoctrungbinh_str.toFloat();
  // String bankinh_str = server.arg("bankinh");
  // R = bankinh_str.toFloat();
  // String omega_str = server.arg("omega");
  // omega = omega_str.toFloat();
  // Serial.println(omega);
  // Serial.println(R);  
  check = true;
}

void quydao(){
  if(check==true){
  xdat = (R)*cos(omega*t);
  ydat = (R)*sin(omega*t);
  xd = (R/100)*cos(omega*t);
  yd = (R/100)*sin(omega*t);
  xd_dot = -(R/100)*omega*sin(omega*t);
  yd_dot = (R/100)*omega*cos(omega*t);
  x0d_dot = -(R/100)*omega*sin(omega);
  y0d_dot = (R/100)*omega*cos(omega);
  xd_2dot = -(R/100)*omega*omega*cos(omega*t);
  yd_2dot = -(R/100)*omega*omega*sin(omega*t);
  Theta0 = atan2(y0d_dot,x0d_dot)-omega;
  Thetad = omega*t + Theta0;
  x1d = xd + b*cos(Thetad);
  y1d = yd + b*sin(Thetad);
  wd = (yd_2dot * xd_dot - xd_2dot * yd_dot) / (xd_dot * xd_dot + yd_dot * yd_dot);
  x1d_dot = xd_dot - b * wd * sin(Thetad);
  y1d_dot = yd_dot + b * wd * cos(Thetad);
  Td[0][0] = cos((Thetad));
  Td[0][1] = -b * sin((Thetad));
  Td[1][0] = sin((Thetad));
  Td[1][1] = b * cos((Thetad));
  Matrix.Invert((mtx_type*)Td, 2);
  Matrix.Copy((mtx_type*)Td, 2, 2, (mtx_type*)TdInv);
  vr = TdInv[0][0]*x1d_dot + TdInv[0][1]*y1d_dot;
  wr = TdInv[1][0]*x1d_dot + TdInv[1][1]*y1d_dot;
  X1=x+b*cos(theta);
  Y1=y+b*sin(theta);

  ex = xd - x;
  ey = yd - y;
  eth = Thetad - theta;

  xe = cos((theta))*ex + sin((theta))*ey;
  ye = -sin((theta))*ex + cos((theta))*ey;
  thetae = eth;

  Vc=vr*cos(thetae)+ Kx*xe; 
  Wc=wr+vr*Ky*ye + Kth*sin(thetae);

  // V= Vc-vthuc;
  // W= Wc-wthuc;
  // globalVR = (((2*60)/(d*pi))*V+((0.2*60)/(d*pi))*W)/2; 
  // globalVL = (((2*60)/(d*pi))*V-((0.2*60)/(d*pi))*W)/2;
  globalVRR = (2 * vd + (l * (wd))) / 2;
  globalVLL = 2 * vd - globalVRR;
  // Serial.println(ex);
  // Serial.println(ey);
  // Serial.println(eth);
  t = t+0.15;
  }
}
void stop() {
  check=false;
  globalVRR=0;
  globalVLL=0;
  // R=0;
  // Vc=0;
  // Wc=0;
  RPM_input1=0;
  RPM_input2=0;
  server.send(200, "text/html", "stop");
}
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
void reset() {
  server.send(200, "text/html", "reset");
}
void A1(){
  // RPM_input1=abs((globalVRR/(0.2*3.14))*60);
  // Theta_now1 = dem_motor1 / 249.6;
  // RPM_output1 = (Theta_now1 - Theta_prev1) / (dt1 / 1000.0) * 60;
  // quangduong1 = float(float(dem_motor111/249.6)*0.2*3.14) ;
  // Vr_output = (RPM_output1/60)*(0.2*3.14);
  error_now11 = vd - vthuc;
  integ_now11 = integ_prev11 + (dt1 * (error_now11 + error_prev11) / 2);
  Vc_dot = (vd - Vc_prev) / dt1;
  V1 = kp1 * error_now11 + ki1 * integ_now11  ;
    
  // if (V1 > Vmax) {
  //   V1 = Vmax;
  //   integ_now1 = integ_prev1;
  // }
  // if (V1 < Vmin) {
  //   V1 = Vmin;
  //   integ_now1 = integ_prev1;
  // }
    
  // PWMval1 = int(255 * abs(V1) / Vmax);
  // if (PWMval1 > 255) {
  //   PWMval1 = 255;
  // }
  // if(globalVRR>0){
  //   digitalWrite(motor1Pin1, LOW);
  //   digitalWrite(motor1Pin2, HIGH);
  //   ledcWrite(pwmChannel, PWMval1);
  // }
  // if(globalVRR<0){
  //   digitalWrite(motor1Pin1, HIGH);
  //   digitalWrite(motor1Pin2, LOW);
  //   ledcWrite(pwmChannel, PWMval1);
  // }
  // Theta_prev1 = Theta_now1;
  integ_prev11 = integ_now11;
  error_prev11 = error_now11;
  Vc_prev = vd;
  
}

void A2(){
  // RPM_input2=abs((globalVLL/(0.2*3.14))*60);
  // Theta_now2 = dem_motor2 / 249.6;
  // RPM_output2 = (Theta_now2 - Theta_prev2) / (dt2 / 1000.0) * 60;
  // quangduong2 = float(float(dem_motor222/249.6)*0.2*3.14) ;
  // Vl_output = (RPM_output2/60)*(0.2*3.14);
  error_now22 = wd - wthuc;
  integ_now22 = integ_prev22 + (dt2 * (error_now22 + error_prev22) / 2);
  Wc_dot = (wd - Wc_prev) / dt2;
  V2 = kp1 * error_now22 + ki1 * integ_now22 ;
    
  // if (V2 > Vmax) {
  //   V2 = Vmax;
  //   integ_now2 = integ_prev2;
  // }
  // if (V2 < Vmin) {
  //   V2 = Vmin;
  //   integ_now2 = integ_prev2;
  // }
    
  // PWMval2 = int(255 * abs(V2) / Vmax);
  // if (PWMval2 > 255) {
  //   PWMval2 = 255;
  // }
  // if(globalVLL>0){
  //   digitalWrite(motor2Pin1, LOW);
  //   digitalWrite(motor2Pin2, HIGH);
  //   ledcWrite(pwmChannel + 1, PWMval2);
  // }
  // if(globalVLL<0){
  //   digitalWrite(motor2Pin1, HIGH);
  //   digitalWrite(motor2Pin2, LOW);
  //   ledcWrite(pwmChannel + 1, PWMval2);
  // }

  // Theta_prev2 = Theta_now2;
  integ_prev22 = integ_now22;
  error_prev22 = error_now22;
  Wc_prev = wd;
}

void PID1(){
  // if(globalVRR>0){
  //     RPM_input1=abs((globalVRR/(0.2*3.14))*60);
  // }
  RPM_input1=abs((globalVRR/(0.2*3.14))*60);
  Theta_now1 = dem_motor1 / 249.6;
  RPM_output1 = (Theta_now1 - Theta_prev1) / (dt1 / 1000.0) * 60;
  // quangduong1 = float(float(dem_motor111/249.6)*0.2*3.14) ;
  Vr_output = (RPM_output1/60)*(0.2*3.14);
  error_now1 = RPM_input1 - RPM_output1;
  integ_now1 = integ_prev1 + (dt1 * (error_now1 + error_prev1) / 2);

  V1 = kp * error_now1 + ki * integ_now1 + (kd * (error_now1 - error_prev1) / dt1) ;
    
  if (V1 > Vmax) {
    V1 = Vmax;
    integ_now1 = integ_prev1;
  }
  if (V1 < Vmin) {
    V1 = Vmin;
    integ_now1 = integ_prev1;
  }
    
  PWMval1 = int(255 * abs(V1) / Vmax);
  if (PWMval1 > 255) {
    PWMval1 = 255;
  }
  if(globalVRR>0){
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    ledcWrite(pwmChannel, PWMval1);
  }
  if(globalVRR<0){
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel, PWMval1);
  }
  Theta_prev1 = Theta_now1;
  integ_prev1 = integ_now1;
  error_prev1 = error_now1;
  
}

void PID2(){
  //   if(globalVLL>0){
  //       RPM_input2=abs((globalVLL/(0.2*3.14))*60);
  // }
  RPM_input2=abs((globalVLL/(0.2*3.14))*60);
  Theta_now2 = dem_motor2 / 249.6;
  RPM_output2 = (Theta_now2 - Theta_prev2) / (dt2 / 1000.0) * 60;
  // quangduong2 = float(float(dem_motor222/249.6)*0.2*3.14) ;
  Vl_output = (RPM_output2/60)*(0.2*3.14);
  error_now2 = RPM_input2 - RPM_output2;
  integ_now2 = integ_prev2 + (dt2 * (error_now2 + error_prev2) / 2);

  V2 = kp * error_now2 + ki * integ_now2 + (kd * (error_now2 - error_prev2) / dt2) ;
    
  if (V2 > Vmax) {
    V2 = Vmax;
    integ_now2 = integ_prev2;
  }
  if (V2 < Vmin) {
    V2 = Vmin;
    integ_now2 = integ_prev2;
  }
    
  PWMval2 = int(255 * abs(V2) / Vmax);
  if (PWMval2 > 255) {
    PWMval2 = 255;
  }
  if(globalVLL>0){
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    ledcWrite(pwmChannel + 1, PWMval2);
  }
  if(globalVLL<0){
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    ledcWrite(pwmChannel + 1, PWMval2);
  }

  Theta_prev2 = Theta_now2;
  integ_prev2 = integ_now2;
  error_prev2 = error_now2;
}

// void PID1(){
//   // if(globalVRR>0){
//   //     RPM_input3=abs((globalVRR/(0.2*3.14))*60);
//   // }
//   RPM_input1 = abs((globalVRR / (0.2 * 3.14)) * 60);
//   Theta_now1 = dem_motor1 / 249.6;
//   RPM_output1 = (Theta_now1 - Theta_prev1) / (dt1 / 1000.0) * 60;
//   // quangduong3 = float(float(dem_motor333 / 249.6) * 0.2 * 3.14);
//   Vr_output = (RPM_output1 / 60) * (0.2 * 3.14);
//   error_now1 = RPM_input1 - RPM_output1;
//   integ_now1 = integ_prev1 + (dt1 * (error_now1 + error_prev1) / 2);

//   V3 = kp * error_now1 + ki * integ_now1 + (kd * (error_now1 - error_prev1) / dt1);

//   if (V3 > Vmax) {
//     V3 = Vmax;
//     integ_now1 = integ_prev1;
//   }
//   if (V3 < Vmin) {
//     V3 = Vmin;
//     integ_now1 = integ_prev1;
//   }

//   PWMval1 = int(255 * abs(V3) / Vmax);
//   if (PWMval1 > 255) {
//     PWMval1 = 255;
//   }
//   if (globalVRR > 0) {
//     digitalWrite(motor1Pin1, LOW);
//     digitalWrite(motor1Pin2, HIGH);
//     ledcWrite(pwmChannel, PWMval1);
//   }
//   if (globalVRR < 0) {
//     digitalWrite(motor1Pin1, HIGH);
//     digitalWrite(motor1Pin2, LOW);
//     ledcWrite(pwmChannel, PWMval1);
//   }
//   Theta_prev1 = Theta_now1;
//   integ_prev1 = integ_now1;
//   error_prev1 = error_now1;
// }

// void PID2(){
//   //   if(globalVLL>0){
//   //       RPM_input4=abs((globalVLL/(0.2*3.14))*60);
//   // }
//   RPM_input2 = abs((globalVLL / (0.2 * 3.14)) * 60);
//   Theta_now2 = dem_motor2 / 249.6;
//   RPM_output4 = (Theta_now2 - Theta_prev4) / (dt2 / 1000.0) * 60;
//   // quangduong4 = float(float(dem_motor444 / 249.6) * 0.2 * 3.14);
//   Vl_output = (RPM_output4 / 60) * (0.2 * 3.14);
//   error_now2 = RPM_input4 - RPM_output4;
//   integ_now2 = integ_prev4 + (dt2 * (error_now2 + error_prev4) / 2);

//   V4 = kp * error_now2 + ki * integ_now2 + (kd * (error_now2 - error_prev4) / dt2);

//   if (V4 > Vmax) {
//     V4 = Vmax;
//     integ_now2 = integ_prev4;
//   }
//   if (V4 < Vmin) {
//     V4 = Vmin;
//     integ_now2 = integ_prev4;
//   }

//   PWMval2 = int(255 * abs(V4) / Vmax);
//   if (PWMval2 > 255) {
//     PWMval2 = 255;
//   }
//   if (globalVLL > 0) {
//     digitalWrite(motor2Pin1, LOW);
//     digitalWrite(motor2Pin2, HIGH);
//     ledcWrite(pwmChannel + 1, PWMval2);
//   }
//   if (globalVLL < 0) {
//     digitalWrite(motor2Pin1, HIGH);
//     digitalWrite(motor2Pin2, LOW);
//     ledcWrite(pwmChannel + 1, PWMval2);
//   }

//   Theta_prev4 = Theta_now2;
//   integ_prev4 = integ_now2;
//   error_prev4 = error_now2;
// }

void smc(){
  m=mc+2*mw+3*sin(t);
  I=Ic+mc*d*d+2*mw*l*l+2*Im;
  m0=m+(2*Iw)/0.01;
  I0=I+(2*l*l*Iw)/0.01;
  s1=V1;
  s2=V2;
  V_dot=Vc_dot+error_now1;
  W_dot=Wc_dot+error_now2;
  ueq1=K1*V_dot;
  ueq2=K2*W_dot;
  ud1=n1*tanh(s1);
  ud2=n2*tanh(s2);
  u1=ueq1+ud1;
  u2=ueq2+ud2;
  // u1=(Vc_dot + error_now1)*m0*0.1+tanh(s1);
  // u2=((Wc_dot + error_now2)*I0*0.1)/l+tanh(s2);

  vd_dot=(u1/0.1+mc*d*wd)/m0;
  wd_dot=((l*u2)/0.1-mc*d*vd)/I0;
  vd=vd+0.5*(vd_dot_prev+vd_dot)*0.15;
  wd=wd+0.5*(wd_dot_prev+wd_dot)*0.15;

  vd_dot_prev = vd_dot;
  wd_dot_prev = wd_dot;

}