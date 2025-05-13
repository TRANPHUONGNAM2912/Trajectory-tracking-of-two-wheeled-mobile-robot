/*******************************************************************************
 * Two-Wheeled Mobile Robot Neural Network Controller
 * 
 * This file implements a Neural Network controller for trajectory tracking 
 * of a two-wheeled differential drive mobile robot using ESP32.
 * 
 * Features:
 * - Real-time trajectory tracking using Neural Network controller
 * - Wi-Fi connectivity for remote monitoring and control
 * - Web interface for visualization and parameter adjustment
 * - Online learning with backpropagation algorithm
 * - Hardware interface for motors and encoders
 * 
 * Hardware:
 * - ESP32 microcontroller
 * - DC motors with encoders (249.6 ticks per revolution)
 * - Wheel diameter: 19 cm
 * - Wheelbase: 38 cm
 * 
 * Dependencies:
 * - WiFi.h
 * - WebServer.h
 * - MatrixMath.h
 * 
 * Usage:
 * 1. Set the SSID and password in the code
 * 2. Upload to ESP32
 * 3. Connect to the ESP32's IP address with a web browser
 * 4. Use the web interface to start/stop the robot and monitor performance
 * 
 *******************************************************************************/

#include <WiFi.h>
#include <WebServer.h>
#include <MatrixMath.h>

//=============================================================================
// CONFIGURATION PARAMETERS
//=============================================================================

// WiFi Configuration
const char* ssid = " ";      // WiFi SSID - to be filled before uploading
const char* password = " ";  // WiFi Password - to be filled before uploading

// Controller Parameters
float kp = 0.02;     // Proportional gain for PID
float ki = 0.00015;  // Integral gain for PID
float kd = 0.01;     // Derivative gain for PID

// Robot Dimensions
float l = 0.38;      // Wheelbase (distance between wheels) in meters
float d = 0.19;      // Wheel diameter in meters

//=============================================================================
// HARDWARE PINS
//=============================================================================

// Motor 1 Pins (Right Motor)
int motor1Pin1 = 2;   // Motor driver input 1
int motor1Pin2 = 4;   // Motor driver input 2
int enable1Pin = 12;  // Motor driver enable pin

// Motor 2 Pins (Left Motor)
int motor2Pin1 = 19;  // Motor driver input 1
int motor2Pin2 = 18;  // Motor driver input 2
int enable2Pin = 15;  // Motor driver enable pin

// Encoder Pins
int encoA_motor1 = 26;  // Encoder A for motor 1
int encoB_motor1 = 13;  // Encoder B for motor 1
int encoA_motor2 = 32;  // Encoder A for motor 2
int encoB_motor2 = 14;  // Encoder B for motor 2

// PWM Configuration
const int freq = 30000;     // PWM frequency
const int pwmChannel = 0;   // PWM channel for motor 1
const int resolution = 8;   // PWM resolution (8-bit = 0-255)

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// Timer Variables
hw_timer_t *timer1 = NULL;
volatile unsigned long count = 0;
unsigned long count_prev = 0;

// Transformation Matrices
mtx_type Td[2][2];     // Transformation matrix
mtx_type TdInv[2][2];  // Inverse transformation matrix

// Motion Variables
float vantoc = 0;    // Velocity
float vthuc = 0;     // Actual velocity
float quangduong1 = 0;  // Distance traveled by wheel 1
float quangduong2 = 0;  // Distance traveled by wheel 2
float quangduongthuc = 0;  // Actual distance traveled

// Angular Position Variables
float Theta_now1;    // Current angular position of motor 1
float Theta_prev1 = 0;  // Previous angular position of motor 1
float Theta_now2;    // Current angular position of motor 2
float Theta_prev2 = 0;  // Previous angular position of motor 2

// Velocity and RPM Variables
float Vl_output;     // Left wheel velocity output
float RPM_output1, RPM_input1;  // RPM for motor 1
float Vr_output;     // Right wheel velocity output
float RPM_output2, RPM_input2;  // RPM for motor 2

// Time Variables
float dt1;           // Delta time for motor 1 control
float dt2;           // Delta time for motor 2 control
unsigned long t_prev = 0;   // Previous time
unsigned long t_now1;       // Current time for motor 1
unsigned long t_prev1 = 0;  // Previous time for motor 1
unsigned long t_now2;       // Current time for motor 2
unsigned long t_prev2 = 0;  // Previous time for motor 2

// Encoder Counters
volatile long dem_motor1 = 0;    // Motor 1 encoder counter (total)
volatile long dem_motor2 = 0;    // Motor 2 encoder counter (total)
volatile long dem_motor11 = 0;   // Motor 1 encoder counter (direction-sensitive)
volatile long dem_motor22 = 0;   // Motor 2 encoder counter (direction-sensitive)

// Motor Control Variables
int PWMval1 = 0;     // PWM value for motor 1
int PWMval2 = 0;     // PWM value for motor 2
float Vmax = 24;     // Maximum voltage
float Vmin = 0;      // Minimum voltage
float V1 = 0;        // Voltage for motor 1
float V2 = 0;        // Voltage for motor 2

// PID Control Variables
float error_now1, error_prev1 = 0, integ_now1, integ_prev1 = 0;  // PID variables for motor 1
float error_now2, error_prev2 = 0, integ_now2, integ_prev2 = 0;  // PID variables for motor 2

// Wheel Velocity Variables
float globalVL = 0;    // Left wheel velocity command
float globalVR = 0;    // Right wheel velocity command
float globalVLL = 0;   // Processed left wheel velocity
float globalVRR = 0;   // Processed right wheel velocity
float leftWheelSpeedpre = 0;   // Previous left wheel speed
float rightWheelSpeedpre = 0;  // Previous right wheel speed

// Robot State Variables
float v = 0, w = 0;    // Linear and angular velocity
float x = 0, y = 0;    // Robot position
float toadox = 0, toadoy = 0;  // Position in cm
float theta = 0;       // Robot orientation

// Trajectory Variables
float t = 0;           // Time variable for trajectory
float omega = 0.25;    // Angular frequency for circular trajectory
float R = 122;         // Radius of circular trajectory (cm)
bool check = false;    // Flag for trajectory following

// Trajectory Tracking Variables
float xd = 0, yd = 0;  // Desired position
float xd_dot = 0, yd_dot = 0;  // Desired velocity
float xd_2dot = 0, yd_2dot = 0;  // Desired acceleration
float Theta = 0, Thetad = 0, Theta0 = 0;  // Orientation variables
float x1d = 0, y1d = 0, x1d_dot = 0, y1d_dot = 0;  // Reference point variables
float wd = 0;          // Desired angular velocity
float V = 0, W = 0;    // Linear and angular velocity commands
float VL = 0, VR = 0;  // Left and right wheel velocity commands
float X1 = 0, Y1 = 0;  // Reference point position
float ex = 0, ey = 0, eth = 0;  // Position and orientation errors
float xe = 0, ye = 0, thetae = 0;  // Transformed errors
float Vc = 0, Wc = 0;  // Control inputs
float vt = 0, vp = 0;  // Tangential and normal velocities
float x0d_dot = 0, y0d_dot = 0;  // Initial desired velocities
float xdat = 0, ydat = 0;  // Desired position in cm
float b = 0.03;        // Distance to reference point
float Kx = 0.0, Ky = 0.0, Kth = 0.0;  // Control gains
float pi = 3.14;       // Pi constant
float wthuc = 0;       // Actual angular velocity
float vr = 0;          // Reference linear velocity
float wr = 0;          // Reference angular velocity

//=============================================================================
// NEURAL NETWORK PARAMETERS
//=============================================================================

// Neural Network Architecture
int Ni = 2;            // Number of input neurons
int No = 3;            // Number of output neurons
int Nh = 20;           // Number of hidden neurons

// Neural Network Weights
float Whi[20][3] = {   // Hidden layer weights (20x3)
  {-6.4855, -4.0100, -4.5498}, {-6.5890, -4.7984, -2.1504}, {-6.5339, -3.9266, -4.3257}, {-7.4260, -3.7515, -2.3720}, 
  {-6.3217, -4.2807, -3.3511}, {-6.1753, -4.2071, -3.1082}, {-8.9702, -5.8552, -4.7596}, {-7.3099, -6.6322, -4.5009}, 
  {-6.2155, -5.9031, -5.4120}, {-5.9862, -3.5501, -3.1355}, {-5.9129, -4.1946, -3.8804}, {-6.3110, -4.0644, -2.8849}, 
  {-6.2365, -2.9500, -2.3163}, {-8.2328, -6.5682, -3.4148}, {-7.1986, -3.6066, -2.8832}, {-5.9831, -3.4436, -3.6779}, 
  {-6.3094, -3.8544, -3.2696}, {-6.2915, -5.4654, -4.1758}, {-6.5584, -4.4558, -3.2127}, {-5.8346, -3.3768, -2.5838}
};

float Woh[3][21] = {   // Output layer weights (3x21)
  {0.0188, -1.9734, 0.6255, 0.4932, -0.5925, -0.4435, 1.0799, 0.3705, -0.1926, -1.1629, -0.0350, -0.9390, 0.3647, -0.6056, 0.5726, -0.1639, -1.3403, -1.5193, -1.6494, -1.8132, -0.2483},
  {-0.0693, 1.2123, 1.3129, 1.6006, 0.7402, 1.1716, 1.5867, 3.1500, 2.3577, 3.3629, 2.4780, 2.6302, 2.9674, 0.7661, 1.5265, 2.0676, 1.7297, 4.0159, 2.3748, 2.4173, 2.9054},
  {6.3216, 3.3388, 1.4826, -0.2754, -2.1694, -0.7430, -2.2963, -4.7001, -4.6418, -4.0895, -4.7556, -4.6468, -4.9112, -5.4908, -3.4376, -4.7539, -4.9947, -4.5363, -5.4131, -3.7736, -4.9652}
};

// Learning Parameters
float eta = 0.7;       // Learning rate
float beta = 0.3;      // Momentum coefficient

// Weight Update Variables
double Woh_old[3][21];  // Previous output layer weights
double DWoh[3][21];     // Output layer weight changes
double DWhi[20][3];     // Hidden layer weight changes
double Dold = 0;        // Previous weight change (output layer)
double Doldi = 0;       // Previous weight change (hidden layer)

// State Variables for Learning
double x_old = 0.0;     // Previous x position
double y_old = 0.0;     // Previous y position
double theta_old = 0.0;  // Previous orientation
double vc_old = 0.0;     // Previous linear velocity command
double wc_old = 0.0;     // Previous angular velocity command

// Control Gain Adaptation
double Kx_old = 1.3;    // Previous x gain
double Ky_old = 2.5;    // Previous y gain
double Kth_old = 1.2;   // Previous orientation gain

double etaX = 0.01;     // Learning rate for x gain
double etaY = 0.01;     // Learning rate for y gain
double etaTh = 0.01;    // Learning rate for orientation gain

double DKx_old = 0;     // Previous x gain change
double DKy_old = 0;     // Previous y gain change
double DKth_old = 0;    // Previous orientation gain change

double Betax = 0.01;    // Momentum coefficient for x gain
double Betay = 0.01;    // Momentum coefficient for y gain
double Betath = 0.01;   // Momentum coefficient for orientation gain

// Jacobian Matrix Elements
double Jacv11 = 0, Jacv12 = 0;  // Jacobian elements for x
double Jacv21 = 0, Jacv22 = 0;  // Jacobian elements for y
double Jacv31 = 0, Jacv32 = 0;  // Jacobian elements for orientation

// Web Server
WebServer server(80);   // Web server on port 80

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

/**
 * @brief Timer interrupt handler
 * Increments the counter for timing control loops
 */
void IRAM_ATTR onTimer1() {
  count++;
}

//=============================================================================
// SETUP FUNCTION
//=============================================================================

/**
 * @brief Setup function - runs once at startup
 * Initializes pins, interrupts, timer, WiFi, and web server
 */
void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Configure motor pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  
  // Configure PWM
  ledcSetup(pwmChannel, freq, resolution);
  ledcSetup(pwmChannel + 1, freq, resolution);
  ledcAttachPin(enable1Pin, pwmChannel);
  ledcAttachPin(enable2Pin, pwmChannel + 1);

  // Configure encoder pins
  pinMode(encoA_motor1, INPUT);
  pinMode(encoB_motor1, INPUT);
  pinMode(encoA_motor2, INPUT);
  pinMode(encoB_motor2, INPUT);
  
  // Attach interrupt handlers to encoder pins
  attachInterrupt(digitalPinToInterrupt(encoA_motor1), dem_xung_motor1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoA_motor2), dem_xung_motor2, RISING);
  
  // Configure timer for 150ms intervals (6.67 Hz)
  timer1 = timerBegin(0, 80, true);  // 80MHz clock divided by 80 = 1MHz
  timerAttachInterrupt(timer1, &onTimer1, true); 
  timerAlarmWrite(timer1, 150000, true);  // 150000 ticks = 150ms
  timerAlarmEnable(timer1); 

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
    delay(500);
  }

  // Print connection info
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Configure web server routes
  server.on("/", handleRoot);        // Root page - displays status
  server.on("/start", handleStart);  // Start trajectory following
  server.on("/stop", stop);          // Stop robot
  server.on("/reset", reset);        // Reset robot state
  server.begin();
  Serial.println("HTTP server started");
}

//=============================================================================
// MAIN LOOP
//=============================================================================

/**
 * @brief Main loop function - runs repeatedly
 * Handles web client requests, calculates position, and controls motors
 */
void loop() {
  // Handle web client requests
  server.handleClient();
  
  // Update robot position
  tinhtoado();

  // Safety check - stop motors if commanded velocity is zero
  if (globalVLL == 0) {
    ledcWrite(pwmChannel + 1, 0);
  }
  if (globalVRR == 0) {
    ledcWrite(pwmChannel, 0);
  }
  
  // Run control loop at fixed intervals (triggered by timer)
  if (count > count_prev) {
    // Generate desired trajectory
    quydao();
    
    // Update time measurements
    t_now1 = millis();
    t_now2 = millis();
    dt1 = (t_now1 - t_prev1);
    dt2 = (t_now2 - t_prev2);
    
    // Execute PID controllers for both motors
    PID1();
    PID2();
    
    // Execute Neural Network controller
    NeuralNetwork();
    
    // Calculate actual robot velocity and angular velocity
    vthuc = (Vl_output + Vr_output) / 2;
    wthuc = (Vr_output - Vl_output) / l;
    
    // Store current time for next iteration
    t_prev1 = t_now1;
    t_prev2 = t_now2;
    count_prev = count;
  }
}

//=============================================================================
// POSITION CALCULATION
//=============================================================================

/**
 * @brief Calculate robot position based on wheel encoder readings
 * Uses kinematics equations to determine position and orientation
 */
void tinhtoado() {
  // Calculate time since last update
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;  // Convert to seconds
  
  // Calculate wheel speeds from encoder counts
  float leftWheelSpeed = (dem_motor22 / 249.6) * 3.14 * 0.19;   // Speed in m/s
  float rightWheelSpeed = (dem_motor11 / 249.6) * 3.14 * 0.19;  // Speed in m/s
  
  // Calculate robot linear and angular velocity
  v = (rightWheelSpeed + leftWheelSpeed) / 2;  // Linear velocity
  w = (rightWheelSpeed - leftWheelSpeed) / l;  // Angular velocity
  
  // Update robot orientation
  theta = theta + w;
  
  // Update robot position using the kinematic model
  x = x + v * cos(theta + (w / 2));
  y = y + v * sin(theta + (w / 2));

  // Convert position to centimeters for display
  toadox = x * 100;
  toadoy = y * 100;
  
  // Reset encoder counters for next calculation
  dem_motor11 = 0;
  dem_motor22 = 0;
  
  // Save current time for next calculation
  prevTime = currentTime;
}

//=============================================================================
// WEB SERVER HANDLERS
//=============================================================================

/**
 * @brief Handle root web page request
 * Returns JSON with current robot state and parameters
 */
void handleRoot() {
  // Create JSON string with current robot state
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
                     ",\"ex\":" + String(ex * 100.0) +
                     ",\"ey\":" + String(ey * 100.0) +
                     ",\"eth\":" + String(eth) +
                     ",\"t\":" + String(t) +
                     "}";
  
  // Send JSON response
  server.send(200, "application/json", motorData);
}

/**
 * @brief Handle start command
 * Sets the flag to start trajectory following
 */
void handleStart() {
  check = true;
  server.send(200, "text/html", "started");
}

/**
 * @brief Handle stop command
 * Stops the robot by setting velocities to zero
 */
void stop() {
  // Stop trajectory following
  check = false;
  
  // Set velocities to zero
  globalVRR = 0;
  globalVLL = 0;
  RPM_input1 = 0;
  RPM_input2 = 0;
  
  // Send response
  server.send(200, "text/html", "stop");
}

/**
 * @brief Handle reset command
 * Resets robot state variables
 */
void reset() {
  // Reset could be implemented to zero position, orientation, etc.
  server.send(200, "text/html", "reset");
}

//=============================================================================
// TRAJECTORY GENERATION
//=============================================================================

/**
 * @brief Generate circular trajectory and calculate control inputs
 * Implements trajectory tracking controller based on kinematic model
 */
void quydao() {
  if (check == true) {
    // Generate circular trajectory
    xdat = (R) * cos(omega * t);                  // Desired x position in cm
    ydat = (R) * sin(omega * t);                  // Desired y position in cm
    xd = (R / 100) * cos(omega * t);              // Desired x position in m
    yd = (R / 100) * sin(omega * t);              // Desired y position in m
    xd_dot = -(R / 100) * omega * sin(omega * t); // Desired x velocity
    yd_dot = (R / 100) * omega * cos(omega * t);  // Desired y velocity
    x0d_dot = -(R / 100) * omega * sin(omega);    // Initial x velocity
    y0d_dot = (R / 100) * omega * cos(omega);     // Initial y velocity
    xd_2dot = -(R / 100) * omega * omega * cos(omega * t); // Desired x acceleration
    yd_2dot = -(R / 100) * omega * omega * sin(omega * t); // Desired y acceleration
    
    // Calculate desired orientation
    Theta0 = atan2(y0d_dot, x0d_dot) - omega;
    Thetad = omega * t + Theta0;
    
    // Calculate reference point position (offset by b)
    x1d = xd + b * cos(Thetad);
    y1d = yd + b * sin(Thetad);
    
    // Calculate desired angular velocity
    wd = (yd_2dot * xd_dot - xd_2dot * yd_dot) / (xd_dot * xd_dot + yd_dot * yd_dot);
    
    // Calculate reference point velocities
    x1d_dot = xd_dot - b * wd * sin(Thetad);
    y1d_dot = yd_dot + b * wd * cos(Thetad);
    
    // Calculate transformation matrix
    Td[0][0] = cos((Thetad));
    Td[0][1] = -b * sin((Thetad));
    Td[1][0] = sin((Thetad));
    Td[1][1] = b * cos((Thetad));
    
    // Calculate inverse transformation matrix
    Matrix.Invert((mtx_type*)Td, 2);
    Matrix.Copy((mtx_type*)Td, 2, 2, (mtx_type*)TdInv);
    
    // Calculate reference linear and angular velocities
    vr = TdInv[0][0] * x1d_dot + TdInv[0][1] * y1d_dot;
    wr = TdInv[1][0] * x1d_dot + TdInv[1][1] * y1d_dot;
    
    // Calculate actual reference point position
    X1 = x + b * cos(theta);
    Y1 = y + b * sin(theta);

    // Calculate position and orientation errors
    ex = xd - x;
    ey = yd - y;
    eth = Thetad - theta;

    // Transform errors to robot frame
    xe = cos((theta)) * ex + sin((theta)) * ey;
    ye = -sin((theta)) * ex + cos((theta)) * ey;
    thetae = eth;

    // Calculate control inputs (linear and angular velocity commands)
    Vc = vr * cos(thetae) + Kx * xe;
    Wc = wr + vr * Ky * ye + Kth * sin(thetae);

    // Calculate wheel velocity commands
    globalVRR = (2 * Vc + (l * (Wc))) / 2;
    globalVLL = 2 * Vc - globalVRR;
    
    // Increment time
    t = t + 0.15;
  }
}

//=============================================================================
// MOTOR CONTROL
//=============================================================================

/**
 * @brief PID controller for motor 1 (Right motor)
 * Controls motor speed using PID feedback from encoder
 */
void PID1() {
  // Calculate target RPM from desired velocity
  RPM_input1 = abs((globalVRR / (0.19 * 3.14)) * 60);
  
  // Calculate current angular position and RPM
  Theta_now1 = dem_motor1 / 249.6;
  RPM_output1 = (Theta_now1 - Theta_prev1) / (dt1 / 1000.0) * 60;
  Vr_output = (RPM_output1 / 60) * (0.19 * 3.14);
  
  // Calculate PID terms
  error_now1 = RPM_input1 - RPM_output1;
  integ_now1 = integ_prev1 + (dt1 * (error_now1 + error_prev1) / 2);

  // Calculate control voltage
  V1 = kp * error_now1 + ki * integ_now1 + (kd * (error_now1 - error_prev1) / dt1);
    
  // Apply voltage limits
  if (V1 > Vmax) {
    V1 = Vmax;
    integ_now1 = integ_prev1;  // Anti-windup
  }
  if (V1 < Vmin) {
    V1 = Vmin;
    integ_now1 = integ_prev1;  // Anti-windup
  }
    
  // Convert voltage to PWM value
  PWMval1 = int(255 * abs(V1) / Vmax);
  if (PWMval1 > 255) {
    PWMval1 = 255;
  }
  
  // Set motor direction and PWM based on commanded velocity
  if (globalVRR > 0) {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    ledcWrite(pwmChannel, PWMval1);
  }
  if (globalVRR < 0) {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel, PWMval1);
  }
  
  // Store values for next iteration
  Theta_prev1 = Theta_now1;
  integ_prev1 = integ_now1;
  error_prev1 = error_now1;
}

/**
 * @brief PID controller for motor 2 (Left motor)
 * Controls motor speed using PID feedback from encoder
 */
void PID2() {
  // Calculate target RPM from desired velocity
  RPM_input2 = abs((globalVLL / (0.19 * 3.14)) * 60);
  
  // Calculate current angular position and RPM
  Theta_now2 = dem_motor2 / 249.6;
  RPM_output2 = (Theta_now2 - Theta_prev2) / (dt2 / 1000.0) * 60;
  Vl_output = (RPM_output2 / 60) * (0.19 * 3.14);
  
  // Calculate PID terms
  error_now2 = RPM_input2 - RPM_output2;
  integ_now2 = integ_prev2 + (dt2 * (error_now2 + error_prev2) / 2);

  // Calculate control voltage
  V2 = kp * error_now2 + ki * integ_now2 + (kd * (error_now2 - error_prev2) / dt2);
    
  // Apply voltage limits
  if (V2 > Vmax) {
    V2 = Vmax;
    integ_now2 = integ_prev2;  // Anti-windup
  }
  if (V2 < Vmin) {
    V2 = Vmin;
    integ_now2 = integ_prev2;  // Anti-windup
  }
    
  // Convert voltage to PWM value
  PWMval2 = int(255 * abs(V2) / Vmax);
  if (PWMval2 > 255) {
    PWMval2 = 255;
  }
  
  // Set motor direction and PWM based on commanded velocity
  if (globalVLL > 0) {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    ledcWrite(pwmChannel + 1, PWMval2);
  }
  if (globalVLL < 0) {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    ledcWrite(pwmChannel + 1, PWMval2);
  }

  // Store values for next iteration
  Theta_prev2 = Theta_now2;
  integ_prev2 = integ_now2;
  error_prev2 = error_now2;
}

//=============================================================================
// NEURAL NETWORK FUNCTIONS
//=============================================================================

/**
 * @brief Sigmoid activation function
 * @param x Input value
 * @return Sigmoid of x
 */
float sigmoid(float x) {
  return 1 / (1 + exp(-x));
}

/**
 * @brief Derivative of sigmoid function
 * @param x Input value
 * @return Derivative of sigmoid at x
 */
float sigmoid_prime(float x) {
  float sig = sigmoid(x);
  return sig * (1 - sig);
}

/**
 * @brief Sign function for Jacobian calculation
 * @param value Input value
 * @return -1 for negative, 0 for zero, 1 for positive
 */
int sign(double value) {
  // Check for Not-a-Number
  if (isnan(value)) {
    return 0;
  }

  // Check for infinity
  if (isinf(value)) {
    return (value > 0) ? 1 : -1;
  }

  // Regular values
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

/**
 * @brief Neural Network controller with online learning
 * Implements a 3-layer neural network with backpropagation
 */
void NeuralNetwork() {
  //-------------------------------------------------------------------------
  // Forward Propagation
  //-------------------------------------------------------------------------
  
  // Input vector with bias
  float I[3] = {1, Vc, Wc};
  
  // Calculate hidden layer activations
  float A[20];   // Hidden layer outputs
  float A1[20];  // Hidden layer pre-activation values (for backprop)
  
  for (int i = 0; i < 20; i++) {
    A[i] = 0;
    A1[i] = 0;
    // Weighted sum of inputs
    for (int j = 0; j < 3; j++) {
      A[i] += Whi[i][j] * I[j];
      A1[i] += Whi[i][j] * I[j];
    }
    // Apply sigmoid activation
    A[i] = sigmoid(A[i]);
  }

  // Add bias to hidden layer outputs
  float Outh[21];
  Outh[0] = 1;  // Bias term
  for (int i = 1; i <= 20; i++) {
    Outh[i] = A[i-1];
  }

  // Calculate output layer activations
  float B[3];
  for (int i = 0; i < 3; i++) {
    B[i] = 0;
    // Weighted sum of hidden layer outputs
    for (int j = 0; j < 21; j++) {
      B[i] += Woh[i][j] * Outh[j];
    }
  }
  
  // Output layer derivatives for backpropagation
  float Lo[3];
  for (int i = 0; i < 3; i++) {
    Lo[i] = sigmoid_prime(B[i]);
  }

  // Hidden layer derivatives for backpropagation
  double Lh[20];
  for (int i = 0; i < 20; i++) {
    Lh[i] = sigmoid_prime(A1[i]);
  }

  //-------------------------------------------------------------------------
  // Calculate Errors
  //-------------------------------------------------------------------------
  
  // Position and orientation errors
  float Ex = x - B[0];       // x position error
  float Ex_sq = 0.5 * Ex * Ex;  // Squared error for x
  
  float Ey = y - B[1];       // y position error
  float Ey_sq = 0.5 * Ey * Ey;  // Squared error for y
  
  float Eth = theta - B[2];  // Orientation error
  float Eth_sq = 0.5 * Eth * Eth;  // Squared error for theta
  
  // Total squared error
  float E_sq = Ex_sq + Ey_sq + Eth_sq;
  
  // Error vector
  float Em[3] = {Ex, Ey, Eth};

  //-------------------------------------------------------------------------
  // Backpropagation - Update Output Layer Weights
  //-------------------------------------------------------------------------
  
  for (int k = 0; k < No; k++) {
    for (int m = 0; m <= Nh; m++) {
      // Calculate weight change with momentum
      DWoh[k][m] = Em[k] * Outh[m] * eta + beta * Dold;
      Dold = DWoh[k][m];  // Save for next iteration's momentum
      
      // Save current weight for hidden layer updates
      Woh_old[k][m] = Woh[k][m];
      
      // Update weight
      Woh[k][m] = Woh[k][m] + DWoh[k][m];
    }
  }

  //-------------------------------------------------------------------------
  // Backpropagation - Update Hidden Layer Weights
  //-------------------------------------------------------------------------
  
  for (int p = 0; p < Nh; p++) {
    for (int k = 0; k <= Ni; k++) {
      // Calculate weight change using chain rule with momentum
      DWhi[p][k] = eta * Lh[p] * I[k] * 
                   (Em[0] * Woh_old[0][p + 1] + 
                    Em[1] * Woh_old[1][p + 1] + 
                    Em[2] * Woh_old[2][p + 1]) + 
                   beta * Doldi;
      
      // Save for next iteration's momentum
      Doldi = DWhi[p][k];
      
      // Update weight
      Whi[p][k] = Whi[p][k] + DWhi[p][k];
    }
  }

  //-------------------------------------------------------------------------
  // Calculate Jacobian for Gain Adaptation
  //-------------------------------------------------------------------------
  
  // Calculate Jacobian elements
  Jacv11 = (B[0] - x_old) / (Vc - vc_old);
  Jacv12 = (B[0] - x_old) / (Wc - wc_old);
  Jacv21 = (B[1] - y_old) / (Vc - vc_old);
  Jacv22 = (B[1] - y_old) / (Wc - wc_old);
  Jacv31 = (B[2] - theta_old) / (Vc - vc_old);
  Jacv32 = (B[2] - theta_old) / (Wc - wc_old);
  
  // Error vector
  double ep[3] = {ex, ey, eth};
  
  // Transformation matrix
  double Te[3][3] = {
    {cos(theta), sin(theta), 0},
    {-sin(theta), cos(theta), 0},
    {0, 0, 1}
  };

  // Jacobian matrix with sign function to handle numerical issues
  double Jacv[3][2] = {
    {sign(Jacv11), sign(Jacv12)},
    {sign(Jacv21), sign(Jacv22)},
    {sign(Jacv31), sign(Jacv32)}
  };

  // Partial derivatives matrix
  double Der1[2][3] = {
    {ex, 0, 0},
    {0, vr * ey, vr * sin(eth)}
  };

  // Matrix multiplication for gradient calculation
  double TeJacv[3][2] = {0};
  double TeJacvDer1[3][3] = {0};
  double Der[3] = {0, 0, 0};

  // Calculate Te * Jacv
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      TeJacv[i][j] = 0;
      for (int k = 0; k < 3; k++) {
        TeJacv[i][j] += Te[i][k] * Jacv[k][j];
      }
    }
  }

  // Calculate TeJacv * Der1
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      TeJacvDer1[i][j] = 0;
      for (int k = 0; k < 2; k++) {
        TeJacvDer1[i][j] += TeJacv[i][k] * Der1[k][j];
      }
    }
  }

  // Calculate -ep' * TeJacvDer1
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Der[i] -= ep[j] * TeJacvDer1[j][i];
    }
  }

  //-------------------------------------------------------------------------
  // Adapt Control Gains
  //-------------------------------------------------------------------------
  
  // Calculate gain changes with momentum
  double DKx = -etaX * Der[0] + Betax * DKx_old;
  double DKy = -etaY * Der[1] + Betay * DKy_old;
  double DKth = -etaTh * Der[2] + Betath * DKth_old;

  // Save for next iteration's momentum
  DKx_old = DKx;
  DKy_old = DKy;
  DKth_old = DKth;

  // Update gains
  Kx = Kx_old + DKx;
  Ky = Ky_old + DKy;
  Kth = Kth_old + DKth;

  // Save values for next iteration
  Kx_old = Kx;
  Ky_old = Ky;
  Kth_old = Kth;
  x_old = B[0];
  y_old = B[1];
  theta_old = B[2];
  vc_old = Vc;
  wc_old = Wc;
}