#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define RESET_PIN 22
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
//M1A->8
//M1B->9

//M1A->10
//M1B->11

const unsigned long BAUD = 115200;
const int TICKS_PER_REV = 400;
const float WHEEL_RADIUS = 0.0325;  // meters
// const float MAX_SPEED =150*2*3.141592653*0.0325/60.0;

#define LEFT_MOTOR_B  11 //  9
#define LEFT_MOTOR_A  10 //  8

#define RIGHT_MOTOR_B 9 //  11
#define RIGHT_MOTOR_A 8 //  10

#define LEFT_ENC_CH1 2
#define LEFT_ENC_CH2 3
#define RIGHT_ENC_CH1 18//20
#define RIGHT_ENC_CH2 19//21

// Encoder tick counters
volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;
long prev_left_ticks = 0, prev_right_ticks = 0;

// PID
float Kp_l = 2.5, Ki_l = 3.5, Kd_l = 1.1;
float Kp_r = 40.0, Ki_r = 0.0, Kd_r = 10.7;
float left_integral = 0, right_integral = 0;
float prev_left_error = 0, prev_right_error = 0;

// Velocity targets and feedback
float target_left_vel = 0.0, target_right_vel = 0.0;
float measured_left_vel = 0.0, measured_right_vel = 0.0;

// Performance metrics
float peak_left = 0.0, peak_right = 0.0;
bool left_settled = false, right_settled = false;
unsigned long settle_time_left = 0, settle_time_right = 0;
unsigned long start_time_ms = 0;

// Simulated IMU
// float qx, qy, qz, qw, ax, ay, az, wx, wy, wz;
// qx = qy = qz = qw = ax = ay = az = wx = wy = wz=0.0;

// Time
unsigned long last_pid_time = 0;

void setup() {
  Serial.begin(BAUD);
  delay(1000);

  pinMode(LEFT_ENC_CH1, INPUT_PULLUP);
  pinMode(LEFT_ENC_CH2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_CH1), leftEncoderISR, CHANGE);

  pinMode(RIGHT_ENC_CH1, INPUT_PULLUP);
  pinMode(RIGHT_ENC_CH2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_CH1), rightEncoderISR, CHANGE);

  pinMode(LEFT_MOTOR_B, OUTPUT);
  pinMode(LEFT_MOTOR_A, OUTPUT);
  pinMode(RIGHT_MOTOR_B, OUTPUT);
  pinMode(RIGHT_MOTOR_A, OUTPUT);

  ///////////////////
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  if (!bno.begin()) 
  {
    Serial.println(F("BNO055 not detected. Check wiring or I2C address."));
    while (1);
  }

  bno.setExtCrystalUse(true);
  Serial.println(F("BNO055 initialized. Reading data..."));

  ///////////////////


  last_pid_time = millis();
}

void loop() {
  // Time delta

  /////////////////////////////////
  if (Serial.available()) 
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("CMD")) 
    {
      cmd.remove(0, 4);
      int space_index = cmd.indexOf(' ');
      if (space_index != -1) {
        float v_left = cmd.substring(0, space_index).toFloat();
        float v_right = cmd.substring(space_index + 1).toFloat();
        target_left_vel = v_left/WHEEL_RADIUS;
        target_right_vel = v_right/WHEEL_RADIUS;

        ///////
         peak_left = 0;
        peak_right = 0;
        left_settled = false;
        right_settled = false;
        settle_time_left = 0;
        settle_time_right = 0;
        start_time_ms = millis();
        //////
      }
    }

    if (cmd == "RESET") {
      noInterrupts();
      left_encoder_ticks = 0;
      right_encoder_ticks = 0;
      interrupts();


      digitalWrite(RESET_PIN, LOW);   // Set LOW to trigger reset
      delay(10);                      // Hold LOW briefly
      digitalWrite(RESET_PIN, HIGH);  // Return HIGH to end reset
      delay(650);

      if (!bno.begin()) 
      {
        Serial.println(F("BNO055 re-init failed after reset!"));
      } 
      else 
      {
        Serial.println(F("BNO055 re-initialized after reset."));
        bno.setExtCrystalUse(true);  // Re-enable external crystal
      }
    }
  }



  /////////////////////////////////////////

  unsigned long now = micros();
  float dt = (now - last_pid_time) / 1000000.0;

  if (dt < 0.05) return;

  // Encoder values
  long left_ticks, right_ticks;
  noInterrupts();
  left_ticks = left_encoder_ticks;
  right_ticks = right_encoder_ticks;
  interrupts();

  // Calculate wheel velocities (rad/s)
  measured_left_vel = ((left_ticks - prev_left_ticks) / (float)TICKS_PER_REV) * 2 * PI / dt;
  measured_right_vel = ((right_ticks - prev_right_ticks) / (float)TICKS_PER_REV) * 2 * PI / dt;
  prev_left_ticks = left_ticks;
  prev_right_ticks = right_ticks;

  // PID control
  
  float error_left = target_left_vel - measured_left_vel;
  float error_right = target_right_vel - measured_right_vel;

  left_integral += error_left * dt;
  right_integral += error_right * dt;

  float d_left = (error_left - prev_left_error) / dt;
  float d_right = (error_right - prev_right_error) / dt;

  float output_left = Kp_l * error_left + Ki_l * left_integral + Kd_l * d_left;
  float output_right = Kp_r * error_right + Ki_r * right_integral + Kd_r * d_right;

  ///////
  // Track peak velocity for overshoot
if (abs(measured_left_vel) > peak_left) peak_left = abs(measured_left_vel);
if (abs(measured_right_vel) > peak_right) peak_right = abs(measured_right_vel);

// Check 2% settling band (and if not already settled)
float err_thresh_left = abs(target_left_vel) * 0.02;
float err_thresh_right = abs(target_right_vel) * 0.02;

if (!left_settled && abs(measured_left_vel - target_left_vel) < err_thresh_left) {
  left_settled = true;
  settle_time_left = millis() - start_time_ms;
}

if (!right_settled && abs(measured_right_vel - target_right_vel) < err_thresh_right) {
  right_settled = true;
  settle_time_right = millis() - start_time_ms;
}
if (left_settled && right_settled) {
  // Calculate overshoot %
  float overshoot_left = 100.0 * (peak_left - abs(target_left_vel)) / abs(target_left_vel);
  float overshoot_right = 100.0 * (peak_right - abs(target_right_vel)) / abs(target_right_vel);

  

  // Reset metrics to avoid re-printing
  peak_left = 0;
  peak_right = 0;
  left_settled = right_settled = false;
  settle_time_left = settle_time_right = 0;
  start_time_ms = millis();
}

  //////

  // Serial.print("current Vel ");
  // Serial.print(measured_left_vel);
  // Serial.print(" ");
  // Serial.print(measured_right_vel);
  // Serial.print("  Target Vel ");
  // Serial.print(target_left_vel);
  // Serial.print(" ");
  // Serial.print(target_right_vel);
  // Serial.print("  error ");
  // Serial.print(error_left);
  // Serial.print(" ");
  // Serial.println(error_right);

  


  int pwm_l=map(target_left_vel,-15.7079,15.7079,-255,255);
  int pwm_r=map(target_right_vel,-15.7079,15.7079,-255,255);
  //int pwm_l = constrain((int)output_left, -255, 255);
  //int pwm_r = constrain((int)output_right, -255, 255);


  

  

  prev_left_error = error_left;
  prev_right_error = error_right;

  setMotorSpeedLeft(pwm_l);
  setMotorSpeedRight(pwm_r);

  // Serial Output ENC
  // sendENC(left_ticks,right_ticks);

  // Serial Output IMU
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // sendIMU(quat.x(),quat.y(),quat.z(),quat.w(),gyro.x(),gyro.y(),gyro.z(),linAccel.x(),linAccel.y(),linAccel.z());



  

  last_pid_time = now;
  // delay(20);
}

// Motor Drivers
void setMotorSpeedLeft(float pwm) {
  bool dir = pwm >= 0;
  if(pwm<0){
    analogWrite(LEFT_MOTOR_B, min(abs((int)pwm), 255));
    digitalWrite(LEFT_MOTOR_A, 0);
  }
  else{
    analogWrite(LEFT_MOTOR_A, min(abs((int)pwm), 255));
    digitalWrite(LEFT_MOTOR_B, 0);
  }
}

void setMotorSpeedRight(float pwm) {
  bool dir = pwm >= 0;
  if(pwm<0){
    analogWrite(RIGHT_MOTOR_B, min(abs((int)pwm), 255));
    digitalWrite(RIGHT_MOTOR_A, 0);
  }
  else{
    analogWrite(RIGHT_MOTOR_A, min(abs((int)pwm), 255));
    digitalWrite(RIGHT_MOTOR_B, 0);
  }
}

// Encoders
void leftEncoderISR() {
  bool ch1 = digitalRead(LEFT_ENC_CH1);
  bool ch2 = digitalRead(LEFT_ENC_CH2);
  left_encoder_ticks += (ch1 == ch2) ? 1 : -1;
}

void rightEncoderISR() {
  bool ch1 = digitalRead(RIGHT_ENC_CH1);
  bool ch2 = digitalRead(RIGHT_ENC_CH2);
  right_encoder_ticks += (ch1 == ch2) ? 1 : -1;
}

void sendENC(int left_ticks,int right_ticks)
{
  Serial.print("ENC ");
  Serial.print(left_ticks); Serial.print(" ");
  Serial.println(right_ticks);
}

void sendIMU(float qx, float qy, float qz, float qw, float wx, float wy, float wz, float ax, float ay, float az)
{
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