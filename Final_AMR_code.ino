#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// Motor pins
#define LEFT_PWM 6
#define LEFT_DIR 5
#define RIGHT_PWM 10
#define RIGHT_DIR 9

// Encoder pins
#define LEFT_ENC_CH1 18
#define LEFT_ENC_CH2 19
#define RIGHT_ENC_CH1 2
#define RIGHT_ENC_CH2 3

//Parameters
const unsigned long BAUD = 115200;
const int TICKS_PER_REV = 2800;
const float WHEEL_RADIUS = 0.05;
const float MAX_SPEED = 0.325; 
float width = 0.6;  

// Encoder tick counters and states
volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;
volatile uint8_t left_last_state = 0;
volatile uint8_t right_last_state = 0;

// PID Cosntants
float k_p = 3.5, k_i = 3.0, k_d = 0.0;

float left_integral = 0.0 , right_integral = 0.0;
float prev_left_e = 0.0 , prev_right_e = 0.0;

float error_r = 0.0;
float error_l = 0.0;
float lw_error = 0.0;
float rw_error = 0.0;

// Target velocities
float target_left_vel = 0.0, target_right_vel = 0.0;
float target_left_rpm = 0.0, target_right_rpm = 0.0;
float omega_left = 0.0 , omega_right = 0.0;
float linear_velocity = 0.0;
float angular_velocity = 0.0;

float measured_left_vel = 0.0 , measured_right_vel = 0.0;
int output_l = 0.0 , output_r = 0;

// Low-Pass filter Parameters
float filtered_output_l = 0.0;
float filtered_output_r = 0.0;
const float alpha = 0.05;

//odometry
float theta=0;
float x=0;
float y=0;
float v_body_measured=0, w_body_measured=0;

// BNO08x setup
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

unsigned long last_time_us = 0.0;
unsigned long prev_time_ms = 0.0 ;
long prev_left_ticks =0.0;
long prev_right_ticks = 0.0 ;


void setup() {
  Serial.begin(BAUD);
  delay(1000);

  // Motor pins
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  // Encoder pins
  pinMode(LEFT_ENC_CH1, INPUT_PULLUP);
  pinMode(LEFT_ENC_CH2, INPUT_PULLUP);
  pinMode(RIGHT_ENC_CH1, INPUT_PULLUP);
  pinMode(RIGHT_ENC_CH2, INPUT_PULLUP);

  // Initialize last state
  left_last_state = (digitalRead(LEFT_ENC_CH1) << 1) | digitalRead(LEFT_ENC_CH2);
  right_last_state = (digitalRead(RIGHT_ENC_CH1) << 1) | digitalRead(RIGHT_ENC_CH2);

  // Attach interrupts on both channels
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_CH1), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_CH2), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_CH1), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_CH2), rightEncoderISR, CHANGE);

  // BNO08x IMU initialization
  while (!bno08x.begin_I2C()) {
    Serial.println("BNO08x not detected!");
    delay(100);
  }

  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION);
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);

  last_time_us = micros();
}

void loop() {
  // Serial input parsing
 if (Serial.available()) {
   String cmd = Serial.readStringUntil('\n');
   cmd.trim();

    if (cmd.startsWith("CMD")) {
      cmd.remove(0, 4);
      int space_index = cmd.indexOf(' ');
      if (space_index != -1) {
        float v_left = cmd.substring(0, space_index).toFloat();
        float v_right = cmd.substring(space_index + 1).toFloat();
        target_left_vel = v_left;
        target_right_vel = v_right;
      }
    }
    
   if (cmd == "RESET") {
     noInterrupts();
     left_encoder_ticks = 0;
     right_encoder_ticks = 0;
     interrupts();
   }

   // If the input is velocity and angular velocity
   //target_left_vel = (2 * linear_velocity - width * angular_velocity) / 2;
   //target_right_vel = (2 * linear_velocity + width * angular_velocity) / 2;
    
 }
   if (target_left_vel >= MAX_SPEED) {
     target_left_vel = MAX_SPEED;
   } else if (target_left_vel <= -MAX_SPEED) {
     target_left_vel = -MAX_SPEED;
   }
 
   if (target_right_vel >= MAX_SPEED) {
     target_right_vel = MAX_SPEED;
   } else if (target_right_vel <= -MAX_SPEED) {
     target_right_vel = -MAX_SPEED;
   }
  
  long left_ticks, right_ticks;
  noInterrupts();
  left_ticks = left_encoder_ticks;
  right_ticks = right_encoder_ticks;
  interrupts();

  unsigned long current_time_ms = millis();
  float dt_ang = (current_time_ms - prev_time_ms) / 1000.0; // seconds

  if (dt_ang >= 0.15) {  // Only compute every 150 ms
    long delta_left_ticks = left_ticks - prev_left_ticks;
    long delta_right_ticks = right_ticks - prev_right_ticks;

    //calculating omega using ticks
    omega_left = (delta_left_ticks*60/(float)TICKS_PER_REV)/ dt_ang;
    omega_right = (delta_right_ticks*60/(float)TICKS_PER_REV)/ dt_ang;
    
    //calculating velocity using omega
    measured_left_vel = omega_left*WHEEL_RADIUS*2*PI/60;
    measured_right_vel = omega_right*WHEEL_RADIUS*2*PI/60;
    
    // odometry
    w_body_measured=(measured_right_vel - measured_left_vel)/width;
    v_body_measured=(measured_left_vel + measured_right_vel)/2.0;

    x += ((v_body_measured)* cos(theta)* dt_ang);
    y += ((v_body_measured)* sin(theta)* dt_ang);
    theta += (w_body_measured*dt_ang);

    if(theta>(2*PI))
    {
      theta=theta-(2*PI);
    }
    if(theta<(-2*PI))
    {
      theta=theta+(2*PI);
    }

    target_left_rpm = ((target_left_vel*60)/(WHEEL_RADIUS*2*PI));
    target_right_rpm = ((target_right_vel*60)/(WHEEL_RADIUS*2*PI));

    // velocity error
    lw_error = target_left_vel - measured_left_vel;
    rw_error = target_right_vel - measured_right_vel;

    // rpm error
    error_l = target_left_rpm- omega_left;
    error_r = target_right_rpm-omega_right;

    left_integral += error_l*dt_ang;
    right_integral += error_r*dt_ang;

    float d_l = (error_l - prev_left_e)/dt_ang;
    float d_r = (error_r - prev_right_e)/dt_ang;

    output_l = (int)(k_p*error_l + k_i*left_integral + k_d*d_l);
    output_r = (int)(k_p*error_r + k_i*right_integral +k_d*d_r);
    
    if(output_l >= 255){
      output_l = 255;
    }
    else if((output_l <35) && (output_l>=0))
    {
      output_l = 0;
    }
    else if(output_l <= -255){
      output_l = -255;
    }
    else if((output_l > -35) && (output_l<=0)){
      output_l = 0;
    }

    if(output_r >= 255){
      output_r = 255;
    }
    else if((output_r <35) && (output_r>=0)){
      output_r = 0;
    }
    else if(output_r <= -255){
      output_r = -255;
    }
    else if((output_r > -35) && (output_r<=0)){
      output_r= 0;
    }

    // Update previous values
    prev_time_ms = current_time_ms;
    prev_left_ticks = left_ticks;
    prev_right_ticks = right_ticks;
    prev_left_e = error_l;
    prev_right_e = error_r;
  }

  // Print encoder values
  sendENC(left_ticks, right_ticks);

  // IMU read
  if (bno08x.getSensorEvent(&sensorValue)) {
    static float qx, qy, qz, qw, wx, wy, wz, ax, ay, az;
    switch (sensorValue.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        qw = sensorValue.un.gameRotationVector.real;
        qx = sensorValue.un.gameRotationVector.i;
        qy = sensorValue.un.gameRotationVector.j;
        qz = sensorValue.un.gameRotationVector.k;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        wx = sensorValue.un.gyroscope.x;
        wy = sensorValue.un.gyroscope.y;
        wz = sensorValue.un.gyroscope.z;
        break;
      case SH2_LINEAR_ACCELERATION:
        ax = sensorValue.un.linearAcceleration.x;
        ay = sensorValue.un.linearAcceleration.y;
        az = sensorValue.un.linearAcceleration.z;
        break;
    }
    sendIMU(qx, qy, qz, qw, wx, wy, wz, ax, ay, az);
  }

// Serial.print(" T_VEL ");
// Serial.print(target_left_vel, 4); 
// Serial.print(" ");
// Serial.println(target_right_vel, 4);

// Serial.print(" M_VEL ");
// Serial.print(measured_left_vel, 4); 
// Serial.print(" ");
// Serial.println(measured_right_vel, 4);

// Serial.print(" PWM  ");
// Serial.print(output_l); 
// Serial.print(" ");
// Serial.println(output_r);
//
//Serial.print("error ");
//Serial.print(lw_error, 4); 
//Serial.print(" ");
//Serial.println(rw_error, 4);
//
//Serial.print("X: ");
//Serial.print(x, 4); 
//Serial.print(" ");
//
//Serial.print("Y: ");
//Serial.print(y, 4); 
//Serial.print(" ");
//
//Serial.print("theta: ");
//Serial.print(theta, 4); 
//Serial.println(" rad");

// low-pass filter
filtered_output_l = alpha * output_l + (1 - alpha) * filtered_output_l;
filtered_output_r = alpha * output_r + (1 - alpha) * filtered_output_r;

setMotorSpeedLeft(filtered_output_l);
setMotorSpeedRight(filtered_output_r);
}

// Motor driver
void setMotorSpeedLeft(float pwm) {
  bool dir = pwm >= 0;
  digitalWrite(LEFT_DIR, dir ? HIGH : LOW);
  analogWrite(LEFT_PWM, min(abs((int)pwm), 255));
}

void setMotorSpeedRight(float pwm) {
  bool dir = pwm >= 0;
  digitalWrite(RIGHT_DIR, dir ? HIGH : LOW);
  analogWrite(RIGHT_PWM, min(abs((int)pwm), 255));
}

// Quadrature encoder ISRs
void leftEncoderISR() {
  uint8_t state = (digitalRead(LEFT_ENC_CH1) << 1) | digitalRead(LEFT_ENC_CH2);
  if ((left_last_state == 0b00 && state == 0b01) ||
      (left_last_state == 0b01 && state == 0b11) ||
      (left_last_state == 0b11 && state == 0b10) ||
      (left_last_state == 0b10 && state == 0b00)) {
    left_encoder_ticks++;
  } else if ((left_last_state == 0b00 && state == 0b10) ||
             (left_last_state == 0b10 && state == 0b11) ||
             (left_last_state == 0b11 && state == 0b01) ||
             (left_last_state == 0b01 && state == 0b00)) {
    left_encoder_ticks--;
  }
  left_last_state = state;
}

void rightEncoderISR() {
  uint8_t state = (digitalRead(RIGHT_ENC_CH1) << 1) | digitalRead(RIGHT_ENC_CH2);
  if ((right_last_state == 0b00 && state == 0b01) ||
      (right_last_state == 0b01 && state == 0b11) ||
      (right_last_state == 0b11 && state == 0b10) ||
      (right_last_state == 0b10 && state == 0b00)) {
    right_encoder_ticks++;
  } else if ((right_last_state == 0b00 && state == 0b10) ||
             (right_last_state == 0b10 && state == 0b11) ||
             (right_last_state == 0b11 && state == 0b01) ||
             (right_last_state == 0b01 && state == 0b00)) {
    right_encoder_ticks--;
  }
  right_last_state = state;
}


void sendENC(int left_ticks, int right_ticks) {
  Serial.print("ENC ");
  Serial.print(left_ticks); Serial.print(" ");
  Serial.println(right_ticks);
}

// Print IMU data
void sendIMU(float qx, float qy, float qz, float qw, float wx, float wy, float wz, float ax, float ay, float az) {
  Serial.print("IMU ");
  Serial.print(qx, 6); Serial.print(" ");
  Serial.print(qy, 6); Serial.print(" ");
  Serial.print(qz, 6); Serial.print(" ");
  Serial.print(qw, 6); Serial.print(" ");
  Serial.print(wx, 6); Serial.print(" ");
  Serial.print(wy, 6); Serial.print(" ");
  Serial.print(wz, 6); Serial.print(" ");
  Serial.print(ax, 6); Serial.print(" ");
  Serial.print(ay, 6); Serial.print(" ");
  Serial.println(az, 6);
}