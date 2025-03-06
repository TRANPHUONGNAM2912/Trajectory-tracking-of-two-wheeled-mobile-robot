#include <WiFi.h>
#include <WebServer.h>
#include <MatrixMath.h>

mtx_type Td[2][2];
mtx_type TdInv[2][2];
const char* ssid = " ";
const char* password = " ";
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
 
unsigned long t_prev = 0;
unsigned long t_now1;
unsigned long t_prev1 = 0;
unsigned long t_now2;
unsigned long t_prev2 = 0;
volatile long dem_motor1 = 0;
volatile long dem_motor2 = 0;
volatile long dem_motor11 = 0;
volatile long dem_motor22 = 0;

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
float omega = 0.25;
float R = 122;
float xd=0,yd=0,xd_dot=0,yd_dot=0,xd_2dot,yd_2dot=0,Theta=0,x1d=0,y1d=0,wd=0,x1d_dot=0,y1d_dot=0,
V=0,W=0,VL=0,VR=0,Theta0=0,Thetad=0,X1=0,Y1=0,ex=0,ey=0,eth=0,xe=0,ye=0,thetae=0,Vc=0,Wc=0,vt=0,vp=0,
x0d_dot=0,y0d_dot=0,xdat=0,ydat=0;
float t =0;
float b = 0.03,Kx=0.0,Ky=0.0,Kth=0.0,pi=3.14,d=0.19;
float wthuc=0;
float vr =0;
float wr=0;
bool check = false;
float Whi[20][3] = {
  {-6.4855, -4.0100, -4.5498}, {-6.5890, -4.7984, -2.1504}, {-6.5339, -3.9266, -4.3257}, {-7.4260, -3.7515, -2.3720}, 
  {-6.3217, -4.2807, -3.3511}, {-6.1753, -4.2071, -3.1082}, {-8.9702, -5.8552, -4.7596}, {-7.3099, -6.6322, -4.5009}, 
  {-6.2155, -5.9031, -5.4120}, {-5.9862, -3.5501, -3.1355}, {-5.9129, -4.1946, -3.8804}, {-6.3110, -4.0644, -2.8849}, 
  {-6.2365, -2.9500, -2.3163}, {-8.2328, -6.5682, -3.4148}, {-7.1986, -3.6066, -2.8832}, {-5.9831, -3.4436, -3.6779}, 
  {-6.3094, -3.8544, -3.2696}, {-6.2915, -5.4654, -4.1758}, {-6.5584, -4.4558, -3.2127}, {-5.8346, -3.3768, -2.5838}
};

float Woh[3][21] = {
  {0.0188, -1.9734, 0.6255, 0.4932, -0.5925, -0.4435, 1.0799, 0.3705, -0.1926, -1.1629, -0.0350, -0.9390, 0.3647, -0.6056, 0.5726, -0.1639, -1.3403, -1.5193, -1.6494, -1.8132, -0.2483},
  {-0.0693, 1.2123, 1.3129, 1.6006, 0.7402, 1.1716, 1.5867, 3.1500, 2.3577, 3.3629, 2.4780, 2.6302, 2.9674, 0.7661, 1.5265, 2.0676, 1.7297, 4.0159, 2.3748, 2.4173, 2.9054},
  {6.3216, 3.3388, 1.4826, -0.2754, -2.1694, -0.7430, -2.2963, -4.7001, -4.6418, -4.0895, -4.7556, -4.6468, -4.9112, -5.4908, -3.4376, -4.7539, -4.9947, -4.5363, -5.4131, -3.7736, -4.9652}
};
double Woh_old[3][21];  // Ma trận lưu giá trị cũ của Woh
double DWoh[3][21];  // Ma trận thay đổi trọng số
double DWhi[20][3];  // Ma trận thay đổi trọng số
int Ni = 2;
int No = 3;
int Nh = 20;
float eta = 0.7;
float beta = 0.3;
double Dold = 0;
double Doldi = 0;
double x_old = 0.0;
double y_old = 0.0;
double theta_old = 0.0;
double vc_old = 0.0;
double wc_old = 0.0;
double Kx_old = 1.3;
double Ky_old = 2.5;
double Kth_old = 1.2;

double etaX = 0.01;
double etaY = 0.01;
double etaTh = 0.01;

double DKx_old = 0;
double DKy_old = 0;
double DKth_old = 0;

double Betax = 0.01;
double Betay = 0.01;
double Betath = 0.01;
double Jacv11 =0;
double Jacv12=0;
double Jacv21=0; 
double Jacv22 =0;
double Jacv31 =0;
double Jacv32 =0;
WebServer server(80); 

void IRAM_ATTR dem_xung_motor1() {
  if (digitalRead(encoB_motor1) == HIGH) {
    dem_motor1++;
    dem_motor11++;
  } else {
    dem_motor1++;
    dem_motor11--;
  }
}

void IRAM_ATTR dem_xung_motor2() {
  if (digitalRead(encoB_motor2) == HIGH) {
    dem_motor2++;
    dem_motor22++;
  } else {
    dem_motor2++;
    dem_motor22--;
  }
}
void IRAM_ATTR onTimer1() {
  count++;
}

void setup() {
  Serial.begin(9600); 
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

  if(globalVLL==0){
    ledcWrite(pwmChannel+1, 0);
  }
  if(globalVRR==0){
    ledcWrite(pwmChannel, 0);
  }
  if (count > count_prev) {
    quydao();
    t_now1 = millis();
    t_now2 = millis();
    dt1 = (t_now1 - t_prev1);
    dt2 = (t_now2 - t_prev2);
    PID1();
    PID2();
    NeuralNetwork();
    
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
  float leftWheelSpeed = (dem_motor22 / 249.6) * 3.14*0.19;
  float rightWheelSpeed = (dem_motor11 / 249.6) * 3.14*0.19;
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
  String motorData = "{\"Kx\":" + String(Kx) +
                     ",\"Ky\":" + String(Ky) +
                     ",\"Kth\":" + String(Kth) +
                    ",\"toadox\":" + String(toadox) +
                     ",\"toadoy\":" + String(toadoy) +
                     ",\"xdat\":" + String(xdat) +
                     ",\"ydat\":" + String(ydat) +
                     ",\"theta\":" + String(theta) +
                     ",\"Thetad\":" + String(Thetad) +
                     ",\"Vc\":" + String(Vc) +
                     ",\"Wc\":" + String(Wc) +
                     ",\"ex\":" + String(ex*100.0) +
                     ",\"ey\":" + String(ey*100.0) +
                     ",\"eth\":" + String(eth) +
                     ",\"t\":" + String(t) +
                     "}";
  server.send(200, "application/json", motorData);
}

void handleStart() {
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
  globalVRR = (2 * Vc + (l * (Wc))) / 2;
  globalVLL = 2 * Vc - globalVRR;
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
  RPM_input1=abs((globalVRR/(0.19*3.14))*60);
  Theta_now1 = dem_motor1 / 249.6;
  RPM_output1 = (Theta_now1 - Theta_prev1) / (dt1 / 1000.0) * 60;
  Vr_output = (RPM_output1/60)*(0.19*3.14);
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
  RPM_input2=abs((globalVLL/(0.19*3.14))*60);
  Theta_now2 = dem_motor2 / 249.6;
  RPM_output2 = (Theta_now2 - Theta_prev2) / (dt2 / 1000.0) * 60;
  Vl_output = (RPM_output2/60)*(0.19*3.14);
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
// Hàm sigmoid
float sigmoid(float x) {
  return 1 / (1 + exp(-x));
}
// Hàm tính đạo hàm của hàm sigmoid (sigmoid prime)
float sigmoid_prime(float x) {
  float sig = sigmoid(x);
  return sig * (1 - sig);
}
void NeuralNetwork(){
  // Forward Propagation
  float I[3] = {1, Vc, Wc}; // vector đầu vào I
  
  // Nhân trọng số Whi với đầu vào I
  float A[20];
  float A1[20];
  for (int i = 0; i < 20; i++) {
    A[i] = 0;
    A1[i] = 0;
    for (int j = 0; j < 3; j++) {
      A[i] += Whi[i][j] * I[j];
      A1[i] += Whi[i][j] * I[j];
    }
    A[i] = sigmoid(A[i]);
  }

  // Thêm phần tử bias vào Outh
  float Outh[21];
  Outh[0] = 1;
  for (int i = 1; i <= 20; i++) {
    Outh[i] = A[i-1];
  }

  // Nhân trọng số Woh với đầu ra Outh
  float B[3];
  for (int i = 0; i < 3; i++) {
    B[i] = 0;
    for (int j = 0; j < 21; j++) {
      B[i] += Woh[i][j] * Outh[j];
    }
  }
  float Lo[3];
  for (int i = 0; i < 3; i++) {
      Lo[i] = sigmoid_prime(B[i]); // Thay thế sigmoid prime với từng phần tử của B
    }

  // Tính đạo hàm hàm sigmoid cho A (Lh trong MATLAB)
  double Lh[20];
  for (int i = 0; i < 20; i++) {
    Lh[i] = sigmoid_prime(A1[i]); // Thay thế sigmoid prime với từng phần tử của A
  }
  // Serial.println("Giá trị của Lo:");
  // for (int i = 0; i < 3; i++) {
  //   Serial.println(Lo[i],4);
  // }
  // Serial.println("Giá trị của A1:");
  // for (int i = 0; i < 20; i++) {
  //   Serial.println(A1[i],4);
  // }
  // // In ra các giá trị của Lh
  // Serial.println("Giá trị của Lh:");
  // for (int i = 0; i < 20; i++) {
  //   Serial.println(Lh[i],10);
  // }
  // Tính các lỗi Ex, Ey, Eth và các bình phương tương ứng
  float Outo[3]; // Outo đã được tính từ đoạn mã trước
  float Ex = x - B[0];
  float Ex_sq = 0.5 * Ex * Ex;

  float Ey = y - B[1]; // Lỗi của y
  float Ey_sq = 0.5 * Ey * Ey;

  float Eth = theta - B[2]; // Lỗi của theta
  float Eth_sq = 0.5 * Eth * Eth;

  // Tính tổng lỗi bình phương
  float E_sq = Ex_sq + Ey_sq + Eth_sq;

  // Vector lỗi Em chứa Ex, Ey, Eth
  float Em[3] = {Ex, Ey, Eth};
  // Vòng lặp cập nhật trọng số giống như trong MATLAB
  for (int k = 0; k < No; k++) {
    for (int m = 0; m <= Nh; m++) {  // Nh+1 nơ-ron đầu ra
      DWoh[k][m] = Em[k] * Outh[m] * eta + beta * Dold;
      Dold = DWoh[k][m];

      Woh_old[k][m] = Woh[k][m];  // Lưu giá trị cũ
      Woh[k][m] = Woh[k][m] + DWoh[k][m];  // Cập nhật trọng số
    }
  }

  // In ra các giá trị cập nhật để kiểm tra
  // Serial.println("Ma trận DWoh sau khi cập nhật:");
  for (int k = 0; k < No; k++) {
    for (int m = 0; m <= Nh; m++) {
      // Serial.print(DWoh[k][m], 6);  // In với 6 chữ số thập phân
      // Serial.print(" ");
    }
    // Serial.println();
  }

  // Serial.println("Ma trận Woh sau khi cập nhật:");
  for (int k = 0; k < No; k++) {
    for (int m = 0; m <= Nh; m++) {
      // Serial.print(Woh[k][m], 6);  // In với 6 chữ số thập phân
      // Serial.print(" ");
    }
    // Serial.println();
  }
   // Vòng lặp cập nhật trọng số cho lớp ẩn
  for (int p = 0; p < Nh; p++) {
    for (int k = 0; k <= Ni; k++) {  // Ni + 1 đầu vào
      // Tính toán thay đổi trọng số DWhi như trong MATLAB
      DWhi[p][k] = eta * Lh[p] * I[k] * (Em[0] * Woh_old[0][p + 1] + Em[1] * Woh_old[1][p + 1] + Em[2] * Woh_old[2][p + 1]) + beta * Doldi;
      
      // Cập nhật Doldi
      Doldi = DWhi[p][k];
      
      // Cập nhật trọng số Whi
      Whi[p][k] = Whi[p][k] + DWhi[p][k];
    }
  }

  // In ra các giá trị cập nhật để kiểm tra
  // Serial.println("Ma trận DWhi sau khi cập nhật:");
  for (int p = 0; p < Nh; p++) {
    for (int k = 0; k <= Ni; k++) {
      // Serial.print(DWhi[p][k], 6);  // In với 6 chữ số thập phân
      // Serial.print(" ");
    }
    // Serial.println();
  }

  // Serial.println("Ma trận Whi sau khi cập nhật:");
  for (int p = 0; p < Nh; p++) {
    for (int k = 0; k <= Ni; k++) {
      // Serial.print(Whi[p][k], 6);  // In với 6 chữ số thập phân
      // Serial.print(" ");
    }
    // Serial.println();
  }
  // In kết quả đầu ra
  // Serial.println("Outo (Đầu ra của mạng nơ-ron):");
  // for (int i = 0; i < 3; i++) {
  //   Serial.println(B[i],4);
  // }
  //   Serial.println("Ex, Ey, Eth:");
  // Serial.println(Ex_sq);
  // Serial.println(Ey_sq);
  // Serial.println(Eth_sq);

  // Serial.println("Tổng lỗi bình phương E_sq:");
  // Serial.println(E_sq);
  Jacv11 = (B[0] - x_old) / (Vc - vc_old);
  Jacv12 = (B[0] - x_old) / (Wc - wc_old);
  Jacv21 = (B[1] - y_old) / (Vc - vc_old);
  Jacv22 = (B[1] - y_old) / (Wc - wc_old);
  Jacv31 = (B[2] - theta_old) / (Vc - vc_old);
  Jacv32 = (B[2] - theta_old) / (Wc - wc_old);
  double ep[3] = {ex, ey, eth};
    double Te[3][3] = {
      {cos(theta), sin(theta), 0},
      {-sin(theta), cos(theta), 0},
      {0, 0, 1}
    };

    // Ma trận Jacv
    double Jacv[3][2] = {
      {sign(Jacv11), sign(Jacv12)},
      {sign(Jacv21), sign(Jacv22)},
      {sign(Jacv31), sign(Jacv32)}
    };

    // Ma trận Der1
    double Der1[2][3] = {
      {ex, 0, 0},
      {0, vr * ey, vr * sin(eth)}
    };

    double TeJacv[3][2] = {0};
    double TeJacvDer1[3][3] = {0};
    double Der[3] = {0, 0, 0};

    // Nhân Te * Jacv
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 2; j++) {
        TeJacv[i][j] = 0;
        for (int k = 0; k < 3; k++) {
          TeJacv[i][j] += Te[i][k] * Jacv[k][j];
        }
      }
    }

    // Nhân TeJacv * Der1
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        TeJacvDer1[i][j] = 0;
        for (int k = 0; k < 2; k++) {
          TeJacvDer1[i][j] += TeJacv[i][k] * Der1[k][j];
        }
      }
    }

    // Nhân -ep' * TeJacvDer1
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        Der[i] -= ep[j] * TeJacvDer1[j][i];
      }
    }

  // Tính DKx, DKy, DKth
  double DKx = -etaX * Der[0] + Betax * DKx_old;
  double DKy = -etaY * Der[1] + Betay * DKy_old;
  double DKth = -etaTh * Der[2] + Betath * DKth_old;

  // Cập nhật DKx_old, DKy_old, DKth_old
  DKx_old = DKx;
  DKy_old = DKy;
  DKth_old = DKth;

  // Cập nhật Kx, Ky, Kth
   Kx = Kx_old + DKx;
   Ky = Ky_old + DKy;
   Kth = Kth_old + DKth;
  //   Serial.println(Der[0]);
  // Serial.println(Der[1]);
  // Serial.println(Der[2]);
  // In kết quả ra Serial Monitor
  // Serial.println("Kx: "); Serial.println(Kx, 4);
  // Serial.println("Ky: "); Serial.println(Ky, 4);
  // Serial.println("Kth: "); Serial.println(Kth, 4);

  // Cập nhật các giá trị cũ
  Kx_old = Kx;
  Ky_old = Ky;
  Kth_old = Kth;
  x_old = B[0];
  y_old = B[1];
  theta_old = B[2];
  vc_old = Vc;
  wc_old = Wc;
}
int sign(double value) {
  // Kiểm tra nếu là NaN (Not-a-Number)
  if (isnan(value)) {
    return 0; // Có thể trả về giá trị khác tùy theo yêu cầu, ở đây chọn 0 cho NaN
  }

  // Kiểm tra nếu là +∞ hoặc -∞
  if (isinf(value)) {
    if (value > 0) {
      return 1;  // Trường hợp +∞
    } else {
      return -1; // Trường hợp -∞
    }
  }

  // Xử lý các giá trị thông thường
  if (value > 0) return 1;
  if (value < 0) return -1;

  // Nếu value là 0
  return 0;
}