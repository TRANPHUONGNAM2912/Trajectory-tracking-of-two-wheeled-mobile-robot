#include <WiFi.h>
#include <WebServer.h>
#include <MatrixMath.h>

mtx_type Td[2][2];
mtx_type TdInv[2][2];
const char* ssid = "";
const char* password = "";
float kp=0.02;   
float ki=0.00015;  
float kd=0.01; 
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

float ev;             // Lỗi giữa tốc độ mong muốn và tốc độ thực tế
float Vdd;             // Tốc độ mong muốn (desired velocity)
// float vthuc;          // Tốc độ thực tế (actual velocity)
float integ_now11;    // Tích phân hiện tại của lỗi cho A1
float integ_prev11;   // Tích phân trước đó của lỗi cho A1
float error_prev11;   // Lỗi trước đó cho A1
// float Vc;             // Vận tốc điều khiển hiện tại (control velocity)
float Vc_prev;        // Vận tốc điều khiển trước đó
float Vc_dot;         // Đạo hàm vận tốc điều khiển (change rate of control velocity)
float V11;             // Giá trị điều khiển tính toán cho A1
float kp1 = 0.01;      // Hệ số điều khiển P cho A1
float ki1 = 0.001;      // Hệ số điều khiển I cho A1
float kp2 = 0.01;      // Hệ số điều khiển P cho A1
float ki2 = 0.001;
// Các biến sử dụng trong hàm A2
float ew;             // Lỗi giữa tốc độ góc mong muốn và tốc độ góc thực tế
float Wdd;             // Tốc độ góc mong muốn (desired angular velocity)
// float wthuc;          // Tốc độ góc thực tế (actual angular velocity)
float integ_now22;    // Tích phân hiện tại của lỗi cho A2
float integ_prev22;   // Tích phân trước đó của lỗi cho A2
float error_prev22;   // Lỗi trước đó cho A2
// float Wc;             // Tốc độ góc điều khiển hiện tại (control angular velocity)
float Wc_prev;        // Tốc độ góc điều khiển trước đó
float Wc_dot;         // Đạo hàm tốc độ góc điều khiển (change rate of control angular velocity)
float V22;             // Giá trị điều khiển tính toán cho A2

// Các biến sử dụng trong hàm SMC
float m;              // Khối lượng của hệ thống
float mc = 10.0;      // Khối lượng của phần chính
float mw = 1.0;       // Khối lượng của bánh xe
// float t = 0.0;        // Thời gian hoặc góc (tùy thuộc vào ngữ cảnh)
float I;              // Mô men quán tính
float Ic = 0.1;       // Mô men quán tính của phần chính
float d = 0;        // Khoảng cách từ tâm đến điểm tính toán (distance)
float Im = 0.05;       // Mô men quán tính của bánh xe
float Iw = 0.05;       // Mô men quán tính của bánh xe
float m0;             // Khối lượng hiệu chỉnh
float I0;             // Mô men quán tính hiệu chỉnh
float s1;             // Biến trượt 1 (sliding variable 1)
float s2;             // Biến trượt 2 (sliding variable 2)
float V_dot;          // Đạo hàm tốc độ
float W_dot;          // Đạo hàm tốc độ góc
float ueq1;           // Điều khiển tương đương 1 (equivalent control 1)
float ueq2;           // Điều khiển tương đương 2 (equivalent control 2)
float ud1;            // Điều khiển phi tuyến 1 (discontinuous control 1)
float ud2;            // Điều khiển phi tuyến 2 (discontinuous control 2)
float u1;             // Điều khiển tổng hợp 1
float u2;             // Điều khiển tổng hợp 2
float vd_dot;         // Đạo hàm tốc độ mong muốn (desired velocity rate)
float wd_dot;         // Đạo hàm tốc độ góc mong muốn (desired angular velocity rate)
float vd_dot_prev;    // Đạo hàm tốc độ mong muốn trước đó
float wd_dot_prev;    // Đạo hàm tốc độ góc mong muốn trước đó

// Hệ số điều khiển SMC
float K1 = 0.2;       // Hệ số điều khiển tương đương cho V
float K2 = 0.4;       // Hệ số điều khiển tương đương cho W
float n1 = 70;       // Hệ số điều khiển phi tuyến cho V
float n2 = 15;       // Hệ số điều khiển phi tuyến cho W

float gamma_hat, alpha_hat;
float a=0.05, c=0.05;
float gamma_max, gamma_min, alpha_max, alpha_min;
float delta_gamma, delta_alpha, gamma_hat_dot, alpha_hat_dot;
float gamma_hat_dot_prev = 0.0;
float alpha_hat_dot_prev = 0.0;
float Proj_gamma_hat(float value);
float Proj_alpha_hat(float value);

 
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
float Vmax = 24;      
float Vmin = 0;
float V1 = 0;
float V2 = 0;
float error_now1, error_prev1 = 0, integ_now1, integ_prev1 = 0;
float error_now2, error_prev2 = 0, integ_now2, integ_prev2 = 0;
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
float omega = 0.3;
float R = 125;
float xd=0,yd=0,xd_dot=0,yd_dot=0,xd_2dot,yd_2dot=0,Theta=0,x1d=0,y1d=0,wd=0,x1d_dot=0,y1d_dot=0,
V=0,W=0,VL=0,VR=0,Theta0=0,Thetad=0,X1=0,Y1=0,ex=0,ey=0,eth=0,xe=0,ye=0,thetae=0,Vc=0,Wc=0,vt=0,vp=0,
x0d_dot=0,y0d_dot=0,xdat=0,ydat=0;
float t =0;
float b = 0.03,Kx=1.0,Ky=2.0,Kth=1.2,pi=3.14;
float wthuc=0;
float vr =0;
float wr=0;
bool check = false;

WebServer server(80); 

void IRAM_ATTR dem_xung_motor1() {
  if (digitalRead(encoB_motor1) == HIGH) {
    dem_motor1++;
    dem_motor11++;
    // dem_motor111++;
  } else {
    dem_motor1++;
    dem_motor11--;
    // dem_motor111--;
  }
}

void IRAM_ATTR dem_xung_motor2() {
  if (digitalRead(encoB_motor2) == HIGH) {
    dem_motor2++;
    dem_motor22++;
    // dem_motor222++;
  } else {
    dem_motor2++;
    dem_motor22--;
    // dem_motor222--;
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
  // server.on("/continuee", continuee);
  server.begin();
  Serial.println("HTTP server started");
}
void loop() {
  server.handleClient();
  tinhtoado();
  // Serial.println(gamma_hat_dot,10);
  //   Serial.println(alpha_hat_dot,10);

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
    PID1();
    PID2();
    A1();
    A2();
    SMC();
    ASMC();
    vthuc = (Vl_output+Vr_output)/2;
    wthuc = (Vr_output-Vl_output)/l;
    t_prev1 = t_now1;
    t_prev2 = t_now2;
    count_prev = count;
  }
}

void tinhtoado(){
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;
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
                     ",\"Vc\":" + String(Vdd) +
                     ",\"Wc\":" + String(Wdd) +
                     ",\"ex\":" + String(ex*100.0) +
                     ",\"ey\":" + String(ey*100.0) +
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
  globalVRR = (2 * Vdd + (l * (Wdd))) / 2;
  globalVLL = 2 * Vdd - globalVRR;
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
// void continuee() {
//   String bankinh_str = server.arg("bankinh");
//   R = bankinh_str.toFloat();
//   String omega_str = server.arg("omega");
//   omega = omega_str.toFloat();
//   String kx_str = server.arg("kx");
//   Kx = kx_str.toFloat();
//   String ky_str = server.arg("ky");
//   Ky = ky_str.toFloat();
//   String ktheta_str = server.arg("ktheta");
//   Kth = ktheta_str.toFloat();
//   server.send(200, "text/html", "continuee");
// }
void reset() {
  server.send(200, "text/html", "reset");
}
void PID1(){
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

void A1(){
  ev = Vc - vthuc;
  integ_now11 = integ_prev11 + ((dt1/1000) * (ev + error_prev11) / 2);
  Vc_dot = (Vc - Vc_prev) / (dt1/1000);
  V11 = kp1 * ev + K1*ki1 * integ_now11;

  integ_prev11 = integ_now11;
  error_prev11 = ev;
  Vc_prev = Vc;
}
void A2(){
  ew = Wc - wthuc;
  integ_now22 = integ_prev22 + ((dt2/1000) * (ew + error_prev22) / 2);
  Wc_dot = (Wc - Wc_prev) / (dt2/1000);
  V22 = kp2 * ew + K2*ki2 * integ_now22 ;

  integ_prev22 = integ_now22;
  error_prev22 = ew;
  Wc_prev = Wc;
}

void SMC(){
  m=mc+2*mw+3*sin(t);
  I=Ic+mc*d*d+2*mw*l*l+2*Im;
  m0=m+(2*Iw)/0.01;
  I0=I+(2*l*l*Iw)/0.01;
  s1=V11;
  s2=V22;
  V_dot=Vc_dot+K1*ev;
  W_dot=Wc_dot+K2*ew;

  ueq1=gamma_hat*V_dot;
  ueq2=alpha_hat*W_dot;
  ud1=n1*tanh(s1);
  ud2=n2*tanh(s2);

  // u1=(Vc_dot + error_now1)*m0*0.1+tanh(s1);
  // u2=((Wc_dot + error_now2)*I0*0.1)/l+tanh(s2); //bdk



}
void ASMC(){
  gamma_max = m0 * 0.1;
  gamma_min = -gamma_max; // Giả định đối xứng
  alpha_max = (I0 * 0.1 )/ 0.19;
  alpha_min = -alpha_max; // Giả định đối xứng

  delta_gamma = -a*s1*V_dot;
  // gamma_hat_dot = Proj_gamma_hat(delta_gamma);
  gamma_hat_dot=0.03;
  // Cập nhật gamma_hat
  // gamma_hat += gamma_hat_dot; 

  delta_alpha = -c*s1*W_dot;
  // alpha_hat_dot = Proj_alpha_hat(delta_alpha);
  alpha_hat_dot=0.01;
  // Cập nhật gamma_hat
  // alpha_hat += alpha_hat_dot;

  gamma_hat=gamma_hat+0.5*(gamma_hat_dot_prev+gamma_hat_dot)*(dt1/1000);
  alpha_hat=alpha_hat+0.5*(alpha_hat_dot_prev+alpha_hat_dot)*(dt2/1000);
  gamma_hat_dot_prev=gamma_hat_dot;
  alpha_hat_dot_prev=alpha_hat_dot;


  // if ((gamma_hat >= gamma_max && delta_gamma > 0) || (gamma_hat <= gamma_min && delta_gamma < 0)) {
  //   gamma_hat_dot = 0;
  // }

  // if ((alpha_hat >= alpha_max && delta_alpha > 0) || (alpha_hat <= alpha_min && delta_alpha < 0)) {
  //   alpha_hat_dot = 0;
  // }
  u1=ueq1+ud1;
  u2=ueq2+ud2;
  vd_dot=(u1/0.1+mc*d*Wdd)/m0;
  wd_dot=((l*u2)/0.1-mc*d*Vdd)/I0;  //dynamic
  // V_dot=(1/m0*0.1)*u1;
  // W_dot=(0.38/(I0*0.1))*u2;

  Vdd=Vdd+0.5*(vd_dot_prev+vd_dot)*(dt1/1000);
  Wdd=Wdd+0.5*(wd_dot_prev+wd_dot)*(dt2/1000);

  vd_dot_prev = V_dot;
  wd_dot_prev = W_dot;
}
  float Proj_gamma_hat(float delta) {
  if ((gamma_hat >= gamma_max && delta > 0) || (gamma_hat <= gamma_min && delta < 0)) {
    return 0;
  } else {
    return delta;
  }
  }
  float Proj_alpha_hat(float delta) {
  if ((alpha_hat >= alpha_max && delta > 0) || (alpha_hat <= alpha_min && delta < 0)) {
    return 0;
  } else {
    return delta;
  }
  }

