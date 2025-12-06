#include <Wire.h>
#include <Servo.h>
#include "AHRSProtocol.h"

/* --- SETTINGS --- */

int DISPLAY_VERSION = 2;
int THROTTLE_MINIMUM = 1000;
int THROTTLE_MAXIMUM = 1800;

float COMPLEMENTARY_FILTER = 0.98;

float throttle = 1000;
float angle_desired[3] = { 0.0, 0.0, 0.0 };

float gain_p[3] = { 1.5, 1.5, 1.5 };
float gain_i[3] = { 0, 0, 0 };
float gain_d[3] = { 0.4, 0.4, 0.4 };

float filter = 0.9;

int mode = 0;

byte navx_data[512];

/* --- CONSTANTS --- */

#define PITCH 1
#define ROLL 2
#define YAW 0

#define MPU_ADDRESS 0x68

#define ITERATION_DELAY_MS 10
#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT 0x32
#define NUM_BYTES_TO_READ 8

#define MAX_PID_OUTPUT 500  // Limit PID output to prevent motor saturation
#define MAX_INTEGRAL_SUM 200  // Limit integral windup

/* --- VARIABLES --- */

float error_current[3] = { 0, 0, 0 };
float error_prev[3] = { 0, 0, 0 };

float pid_current[3] = { 0, 0, 0 };
float pid_p[3] = { 0, 0, 0 };
float pid_i[3] = { 0, 0, 0 };
float pid_d[3] = { 0, 0, 0 };

float integral_sum[3] = { 0, 0, 0 };  // Added: track integral accumulation

float angle_current[3];
float angle_acc[3];
float angle_gyro[3];
float angle_acc_offset[3] = { 0.0, 0.0, 0.0 };
float angle_gyro_offset[3] = { 0.0, 0.0, 0.0 };

float angle_acc_raw[3];
int16_t angle_gyro_raw[3];

float time_current;
float time_prev;
double time_elapsed;

Servo motor_1;  // Motor front right
Servo motor_2;  // Motor front left
Servo motor_3;  // Motor back left
Servo motor_4;  // Motor back right

float rad_to_deg = 180 / 3.141592654;

float blink_counter = 0;
bool blink_status = false;

float lastCommand = 0;

int sendDataCounter = 0;

void setup() {

  Serial.begin(115200);

  Wire.begin();

  Serial.println("SETUP: Start");

  for (int i = 0; i < sizeof(navx_data); i++) {
    navx_data[i] = 0;
  }

  time_current = millis();

  /* Attach the motors to different pins to avoid conflicts */
  motor_1.attach(3);   // Changed from 13
  motor_2.attach(5);   // Changed from 11
  motor_3.attach(6);   // Changed from 9
  motor_4.attach(10);  // Changed from 8

  Serial.println("SETUP: Motors attached");

  Serial.println("SETUP: Calibrating motors");

  calibrateMotors();

  pinMode(13, OUTPUT);  // Now safe for LED

  Serial.println("SETUP: Motors calibrated");
  Serial.println("SETUP: Finished");
  
}


void loop() {
  time_prev = time_current;
  time_current = millis();
  time_elapsed = (time_current - time_prev) / 1000;

  // Prevent division by zero issues
  if (time_elapsed <= 0) {
    time_elapsed = 0.01;
  }

  getDataFromNAVX();

  receiveControl();

  calculatePid();

  if (throttle > 1010) {
    if (millis() > lastCommand + 1000) {
      emergencyLanding();
    }

    setMotorPids();
  } else {
    setSpeedForAllMotors(THROTTLE_MINIMUM);
  }

  //if (sendDataCounter > 150) {
  //  sendData();
  //  sendDataCounter = 0;
  //}

  //sendDataCounter += 1;

  blinkLED();
}

/**
 * Calculates all PID values with proper integral term
 */
void calculatePid() {
  error_prev[PITCH] = error_current[PITCH];
  error_prev[ROLL] = error_current[ROLL];
  error_prev[YAW] = error_current[YAW];

  error_current[PITCH] = angle_current[PITCH] - angle_desired[PITCH];
  error_current[ROLL] = angle_current[ROLL] - angle_desired[ROLL];
  error_current[YAW] = angle_current[YAW] - angle_desired[YAW];

  /* Proportional term */
  pid_p[PITCH] = gain_p[PITCH] * error_current[PITCH];
  pid_p[ROLL] = gain_p[ROLL] * error_current[ROLL];
  pid_p[YAW] = gain_p[YAW] * error_current[YAW];

  /* Integral term - accumulate error over time with anti-windup */
  integral_sum[PITCH] += error_current[PITCH] * time_elapsed;
  integral_sum[ROLL] += error_current[ROLL] * time_elapsed;
  integral_sum[YAW] += error_current[YAW] * time_elapsed;

  // Anti-windup: clamp integral accumulation
  for (int i = 0; i < 3; i++) {
    if (integral_sum[i] > MAX_INTEGRAL_SUM) {
      integral_sum[i] = MAX_INTEGRAL_SUM;
    } else if (integral_sum[i] < -MAX_INTEGRAL_SUM) {
      integral_sum[i] = -MAX_INTEGRAL_SUM;
    }
  }

  pid_i[PITCH] = gain_i[PITCH] * integral_sum[PITCH];
  pid_i[ROLL] = gain_i[ROLL] * integral_sum[ROLL];
  pid_i[YAW] = gain_i[YAW] * integral_sum[YAW];

  /* Derivative term */
  float pid_d_new[3];

  pid_d_new[PITCH] = gain_d[PITCH] * (error_current[PITCH] - error_prev[PITCH]) / time_elapsed;
  pid_d_new[ROLL] = gain_d[ROLL] * (error_current[ROLL] - error_prev[ROLL]) / time_elapsed;
  pid_d_new[YAW] = gain_d[YAW] * (error_current[YAW] - error_prev[YAW]) / time_elapsed;

  pid_d[PITCH] = filter * pid_d[PITCH] + (1 - filter) * pid_d_new[PITCH];
  pid_d[ROLL] = filter * pid_d[ROLL] + (1 - filter) * pid_d_new[ROLL];
  pid_d[YAW] = filter * pid_d[YAW] + (1 - filter) * pid_d_new[YAW];

  /* Sum with bounds checking */
  pid_current[PITCH] = pid_p[PITCH] + pid_i[PITCH] + pid_d[PITCH];
  pid_current[ROLL] = pid_p[ROLL] + pid_i[ROLL] + pid_d[ROLL];
  pid_current[YAW] = pid_p[YAW] + pid_i[YAW] + pid_d[YAW];

  // Clamp PID output
  for (int i = 0; i < 3; i++) {
    if (pid_current[i] > MAX_PID_OUTPUT) {
      pid_current[i] = MAX_PID_OUTPUT;
    } else if (pid_current[i] < -MAX_PID_OUTPUT) {
      pid_current[i] = -MAX_PID_OUTPUT;
    }
  }
}


void setMotorPids() {
  float m1 = throttle + pid_current[PITCH] + pid_current[ROLL] + pid_current[YAW];
  float m2 = throttle + pid_current[PITCH] - pid_current[ROLL] - pid_current[YAW];
  float m3 = throttle - pid_current[PITCH] - pid_current[ROLL] + pid_current[YAW];
  float m4 = throttle - pid_current[PITCH] + pid_current[ROLL] - pid_current[YAW];

  // Bounds checking for motor outputs
  m1 = constrain(m1, THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
  m2 = constrain(m2, THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
  m3 = constrain(m3, THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
  m4 = constrain(m4, THROTTLE_MINIMUM, THROTTLE_MAXIMUM);

  if (mode == 1 || mode == 0) {
    motor_1.writeMicroseconds((int)m1);
    motor_3.writeMicroseconds((int)m3);
  }

  if (mode == 2 || mode == 0) {
    motor_2.writeMicroseconds((int)m2);
    motor_4.writeMicroseconds((int)m4);
  }
}

void emergencyLanding() {
  throttle = throttle - 0.25;

  angle_desired[0] = 0.0;
  angle_desired[1] = 0.0;
  angle_desired[2] = 0.0;
}

void calibrateMotors() {
  setSpeedForAllMotors(THROTTLE_MINIMUM);
  delay(7000);
}

void setSpeedForAllMotors(double speed) {
  motor_1.writeMicroseconds((int)speed);
  motor_2.writeMicroseconds((int)speed);
  motor_3.writeMicroseconds((int)speed);
  motor_4.writeMicroseconds((int)speed);
}

void blinkLED() {
  if (blink_counter > 20) {
    blink_status = !blink_status;

    if (blink_status) {
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
    }

    blink_counter = 0;
  }

  blink_counter = blink_counter + 1;
}

void getDataFromNAVX() {
  int i = 0;
  Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT);
  Wire.write(NAVX_REG_YAW_L);
  Wire.write(NUM_BYTES_TO_READ);
  Wire.endTransmission();

  Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT);
  Wire.requestFrom(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT, NUM_BYTES_TO_READ);
  delay(1);
  while (Wire.available()) {
    navx_data[i++] = Wire.read();
  }
  Wire.endTransmission();

  float yaw = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&navx_data[0]);
  float pitch = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&navx_data[2]);
  float roll = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&navx_data[4]);

  angle_current[0] = yaw;
  angle_current[1] = pitch;
  angle_current[2] = roll;
}

void receiveControl() {
  if (Serial.available()) {
    String command = Serial.readStringUntil(';');

    if (command.length() == 0) return;  // Safety check

    if (command[0] == '.') {
      if (command.length() < 3) return;  // Safety: ensure enough characters

      if (command[1] == 't') {
        throttle = command.substring(2).toInt();
        throttle = constrain(throttle, THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
      } else if (command[1] == 'p') {
        angle_desired[PITCH] = command.substring(2).toFloat();
      } else if (command[1] == 'r') {
        angle_desired[ROLL] = command.substring(2).toFloat();
      } else if (command[1] == 'y') {
        angle_desired[YAW] = command.substring(2).toFloat();
      } else if (command[1] == 's'){
        sendData();
      }

      lastCommand = millis();
    } else {
      if (command == "throttle+") {
        throttle = constrain(throttle + 50, THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
      } else if (command == "throttle-") {
        throttle = constrain(throttle - 50, THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
      } else if (command == "stop") {
        throttle = THROTTLE_MINIMUM;
      } else if (command == "calibrateAngles") {
        //calibrateAngleOffsets();
      } else if (command == "gainP+") {
        gain_p[PITCH] += 0.1;
        gain_p[ROLL] += 0.1;
      } else if (command == "gainP-") {
        gain_p[PITCH] -= 0.1;
        gain_p[ROLL] -= 0.1;
      } else if (command == "gainD+") {
        gain_d[PITCH] += 0.05;
        gain_d[ROLL] += 0.05;
      } else if (command == "gainD-") {
        gain_d[PITCH] -= 0.05;
        gain_d[ROLL] -= 0.05;
      } else if (command == "right") {
        angle_desired[ROLL] += 2;
      } else if (command == "left") {
        angle_desired[ROLL] -= 2;
      } else if (command == "filter+") {
        filter = constrain(filter + 0.005, 0.0, 1.0);
      } else if (command == "filter-") {
        filter = constrain(filter - 0.005, 0.0, 1.0);
      } else if (command == "mode0") {
        mode = 0;
      } else if (command == "mode1") {
        mode = 1;
      } else if (command == "mode2") {
        mode = 2;
      }
    }
  }
}

unsigned char *floatToPaddedByteArray(float value) {
  static unsigned char byteArray[8];

  for (int i = 0; i < 8; i++) {
    byteArray[i] = 0;
  }

  unsigned char *floatBytes = (unsigned char *)&value;
  for (int i = 0; i < 4; i++) {
    byteArray[i] = floatBytes[i];
  }

  return byteArray;
}

void sendData() {
  Serial.write("B");

  unsigned char *bytes = floatToPaddedByteArray(throttle);
  for (int i = 0; i < 4; i++) {
    Serial.write(bytes[i]);
  }
  Serial.write(0x7C);

  bytes = floatToPaddedByteArray(angle_current[YAW]);
  for (int i = 0; i < 4; i++) {
    Serial.write(bytes[i]);
  }
  Serial.write(0x7C);

  bytes = floatToPaddedByteArray(angle_current[PITCH]);
  for (int i = 0; i < 4; i++) {
    Serial.write(bytes[i]);
  }
  Serial.write(0x7C);

  bytes = floatToPaddedByteArray(angle_current[ROLL]);
  for (int i = 0; i < 4; i++) {
    Serial.write(bytes[i]);
  }
  Serial.write(0x7C);

  bytes = floatToPaddedByteArray(angle_desired[YAW]);
  for (int i = 0; i < 4; i++) {
    Serial.write(bytes[i]);
  }
  Serial.write(0x7C);

  bytes = floatToPaddedByteArray(angle_desired[PITCH]);
  for (int i = 0; i < 4; i++) {
    Serial.write(bytes[i]);
  }
  Serial.write(0x7C);

  bytes = floatToPaddedByteArray(angle_desired[ROLL]);
  for (int i = 0; i < 4; i++) {
    Serial.write(bytes[i]);
  }
  Serial.write(0x7C);

  bytes = floatToPaddedByteArray(pid_current[YAW]);
  for (int i = 0; i < 4; i++) {
    Serial.write(bytes[i]);
  }
  Serial.write(0x7C);

  bytes = floatToPaddedByteArray(pid_current[PITCH]);
  for (int i = 0; i < 4; i++) {
    Serial.write(bytes[i]);
  }
  Serial.write(0x7C);

  bytes = floatToPaddedByteArray(pid_current[ROLL]);
  for (int i = 0; i < 4; i++) {
    Serial.write(bytes[i]);
  }

  Serial.write("E");
}