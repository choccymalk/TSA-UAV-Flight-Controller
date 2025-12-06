#include <Wire.h>
#include <Servo.h>
#include <cstring>
#include <array>
#include <iostream>
#include "AHRSProtocol.h"             // navX-Sensor Register Definition header file

/* --- SETTINGS --- */

int DISPLAY_VERSION = 2;                            // Which display should be used for the remote controller? 1D or 2D version?

int THROTTLE_MINIMUM = 1000;                        // Minimum throttle of a motor
int THROTTLE_MAXIMUM = 1800;                        // Maximum throttle of a motor

float COMPLEMENTARY_FILTER = 0.98;                   // Complementary filter for combining acc and gyro

float throttle = 1000;                             // Desired throttle
float angle_desired[3] = {0.0, 0.0, 0.0};           // Desired angle

float gain_p[3] = {1.5, 1.5, 1.5};                    // Gain proportional
float gain_i[3] = {0, 0, 0};                        // Gain integral
float gain_d[3] = {0.4, 0.4, 0.4};                    // Gain derivetive

float filter = 0.9;                                 // Complementary filter for pid

int mode = 0;                                       // Mode for testing purpose: 0 = all motors | 1 = motor 1 & 3 | 2 = motor 2 & 4

byte navx_data[512];

/* --- CONSTANTS --- */

#define PITCH 1                                     // Rotation forward/backward
#define ROLL 3                                      // Rotation left/right
#define YAW 0                                       // Rotation around center

#define MPU_ADDRESS 0x68

#define ITERATION_DELAY_MS                   10
#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT  0x32
#define NUM_BYTES_TO_READ                    8

/* --- VARIABLES --- */

float error_current[3] = {0, 0, 0};                 // Current error
float error_prev[3] = {0, 0, 0};                    // Previous error

float pid_current[3] = {0, 0, 0};                   // PID weighted (!) sum of proportional, integral and derivitive error
float pid_p[3] = {0, 0, 0};                         // PID proportional error     
float pid_i[3] = {0, 0, 0};                         // PID integral error
float pid_d[3] = {0, 0, 0};                         // PID derivitive error

float angle_current[3];                             // Angle measured after filtering
float angle_acc[3];                                 // Angle measured using accelerometer
float angle_gyro[3];                                // Angle measured using gyro
float angle_acc_offset[3] = {0.0,0.0,0.0};          // Offsets for gyro angle measurement
float angle_gyro_offset[3] = {0.0,0.0,0.0};         // Offsets for acc angle measurement

float angle_acc_raw[3];                             // Accelerator raw data
int16_t angle_gyro_raw[3];                          // Gyro raw data

float time_current;                                 // Current time
float time_prev;                                    // Previous time
double time_elapsed;                                // Elapsed time during the last loop

Servo motor_1;                                      // Motor front right
Servo motor_2;                                      // Motor front left
Servo motor_3;                                      // Motor back left
Servo motor_4;                                      // Motor back right

float rad_to_deg = 180/3.141592654;                 // Constant for convert radian to degrees

float blink_counter = 0;                            // The blink counter is used to let the build-in LED blink every x loops to see whether the programm is still running or not
bool blink_status = false;                             // Is the build-in LED currently on or off?

float lastCommand = 0;                              // Time when the last comment has been recieved

int sendDataCounter = 0;                            // Counts when data has been sent the last time to reduce amount of data

void setup() {

  /* Begin serial communication for remote control */
  Serial.begin(115200);

  Wire.begin();

  //while(!Serial) {}
  
  Serial.println("SETUP: Start");

  for ( int i = 0; i < sizeof(navx_data); i++ ) {
      navx_data[i] = 0;
  }

  time_current = millis();

  /* Attach the motors */
  motor_1.attach(13);                                
  motor_2.attach(11);
  motor_3.attach(9);
  motor_4.attach(8);

  Serial.println("SETUP: Motors attached");

  Serial.println("SETUP: Calibrating motors");

  /* Calibrate the motors */
  calibrateMotors();

  pinMode(13, OUTPUT);

  Serial.println("SETUP: Motors calibrated");
  Serial.println("SETUP: Finished");
}


void loop() {
  /* Calculate elapsed time */
  time_prev = time_current;
  time_current = millis();
  time_elapsed = (time_current - time_prev) / 1000;

  /* Get IMU data */
  getDataFromNAVX();

  /* Receive the remote controller's commands */
  receiveControl();

  /* Calculate PID */
  calculatePid();

  
  if(throttle > 1010) {
    /* Emergency landing if no command has been recieved for more then a second */
    if(millis() > lastCommand + 1000) {
      emergencyLanding();
    }

    /* Apply PID to all motors */
    setMotorPids();
  } else {
    /* Turn off all motors */
    setSpeedForAllMotors(THROTTLE_MINIMUM);
  }

  if(sendDataCounter > 3) {
    sendData();
    sendDataCounter = 0;
  }
  
  sendDataCounter += 1;  

  blinkLED();
}

/**
 * Calculates all PID values
 */
void calculatePid() {
  /* Save previous errors */
  error_prev[PITCH] = error_current[PITCH];
  error_prev[ROLL] = error_current[ROLL];
  error_prev[YAW] = error_current[YAW];

  /* Calculate current error */
  error_current[PITCH] = angle_current[PITCH] - angle_desired[PITCH];
  error_current[ROLL] = angle_current[ROLL] - angle_desired[ROLL];
  error_current[YAW] = angle_current[YAW] - angle_desired[YAW];

  /* Calculate weighted proportional error */
  pid_p[PITCH] = gain_p[PITCH] * error_current[PITCH];
  pid_p[ROLL] = gain_p[ROLL] * error_current[ROLL];
  pid_p[YAW] = gain_p[YAW] * error_current[YAW];

  /* Calculated weighted derivitive error */
  float pid_d_new[3];

  pid_d_new[PITCH] = gain_d[PITCH] * (error_current[PITCH] - error_prev[PITCH]) / time_elapsed;
  pid_d_new[ROLL] = gain_d[ROLL] * (error_current[ROLL] - error_prev[ROLL]) / time_elapsed;
  pid_d_new[YAW] = gain_d[YAW] * (error_current[YAW] - error_prev[YAW]) / time_elapsed;

  pid_d[PITCH] = filter * pid_d[PITCH] + (1 - filter) * pid_d_new[PITCH];
  pid_d[ROLL] = filter * pid_d[ROLL] + (1 - filter) * pid_d_new[ROLL];
  pid_d[YAW] = filter * pid_d[YAW] + (1 - filter) * pid_d_new[YAW];

  /* Calculate weighted sum of the PID */
  pid_current[PITCH] = pid_p[PITCH] + pid_i[PITCH] + pid_d[PITCH];
  pid_current[ROLL] = pid_p[ROLL] + pid_i[ROLL] + pid_d[ROLL];
  pid_current[YAW] = pid_p[YAW] + pid_i[YAW] + pid_d[YAW];
}


/* 
* Sets the PID values to motors 
* THERE MIGHT BE SOME ERRORS WITH THE SIGNS of the pid variables
* TODO: Write down the orientation 
*/
void setMotorPids() {
  if(mode == 1 || mode == 0) {
    motor_1.writeMicroseconds(throttle + pid_current[PITCH] + pid_current[ROLL] + pid_current[YAW]);      // Set PID for front right motor
    motor_3.writeMicroseconds(throttle - pid_current[PITCH] - pid_current[ROLL] + pid_current[YAW]);      // Set PID for back left motor
  }

  if(mode == 2 || mode == 0) {
    motor_2.writeMicroseconds(throttle + pid_current[PITCH] - pid_current[ROLL] - pid_current[YAW]);      // Set PID for front left motor
    motor_4.writeMicroseconds(throttle - pid_current[PITCH] + pid_current[ROLL] - pid_current[YAW]);      // Set PID for back right motor
  }
}

/*
 * Automatic emergency landing:
 * Decrease throttle slowly and set desired angle to 0
 */
void emergencyLanding() {
  throttle = throttle - 0.25;
      
  angle_desired[0] = 0.0;
  angle_desired[1] = 0.0;
  angle_desired[2] = 0.0;
}

/**
 * Calibrates all 4 motors
 */
void calibrateMotors() {
  setSpeedForAllMotors(THROTTLE_MINIMUM);
  delay(7000);
}

/**
 * Sets the given speed for all motors
 */
void setSpeedForAllMotors(double speed) {
  motor_1.writeMicroseconds(speed);
  motor_2.writeMicroseconds(speed);
  motor_3.writeMicroseconds(speed);
  motor_4.writeMicroseconds(speed);
}

/**
 * Let the build-in status LED blink to make sure the main loop is running and not stuck
 */
void blinkLED() {
  if(blink_counter > 20) {
    blink_status = !blink_status;

    // Toggle LED
    if(blink_status) {
      digitalWrite(13, HIGH);
    }
    else {
      digitalWrite(13, LOW);
    }
    
    blink_counter = 0;
  }

  blink_counter = blink_counter + 1;  
}

void getDataFromNAVX(){
  int i = 0;
  /* Transmit I2C data request */
  Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT); // Begin transmitting to navX-Sensor
  Wire.write(NAVX_REG_YAW_L);                                // Sends starting register address
  Wire.write(NUM_BYTES_TO_READ);                               // Send number of bytes to read
  Wire.endTransmission();                                      // Stop transmitting
  
  /* Receive the echoed value back */
  Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT); // Begin transmitting to navX-Sensor
  Wire.requestFrom(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT, NUM_BYTES_TO_READ);    // Send number of bytes to read
  delay(1);
  while(Wire.available()) {                                    // Read data (slave may send less than requested)
     data[i++] = Wire.read();
  }
  Wire.endTransmission();                                      // Stop transmitting

  /* Decode received data to floating-point orientation values */
  float yaw =     IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[0]);   // The cast is needed on arduino
  float pitch =   IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[2]);   // The cast is needed on arduino
  float roll =    IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[4]);   // The cast is needed on arduino
  float heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat((char *)&data[6]); // The cast is needed on arduino  
  
  angle_current = {yaw, pitch, roll};
}

/**
 * Low pass filter the gyro's data using a complementary filter, unneeded as navx does this
 */
void filterAngle() {
  float angle_new[3];

  angle_new[PITCH] = -(COMPLEMENTARY_FILTER * (-angle_current[PITCH] + angle_gyro[PITCH] * time_elapsed) + (1 - COMPLEMENTARY_FILTER) * angle_acc[PITCH]);    // Positive angle -> forward
  angle_new[ROLL] = COMPLEMENTARY_FILTER * (angle_current[ROLL] + angle_gyro[ROLL] * time_elapsed) + (1 - COMPLEMENTARY_FILTER) * angle_acc[ROLL];            // Positive angle -> right
  angle_new[YAW] = COMPLEMENTARY_FILTER * (angle_current[YAW] + angle_gyro[YAW] * time_elapsed) + (1 - COMPLEMENTARY_FILTER) * angle_acc[YAW];                // Calculated by chris

  float value = 0.5; // some weird stuff is going on here, this is we we use the value variable

  angle_current[PITCH] = value * angle_current[PITCH] + (1 - value) * angle_new[PITCH];
  angle_current[ROLL] = value * angle_current[ROLL] + (1 - value) * angle_new[ROLL];
  angle_current[YAW] = value * angle_current[YAW] + (1 - value) * angle_new[YAW];
}


/**
 * Reads the gyro and saves the values, unneeded as getDataFromNavx does this
 */
void readGyro() {
  /* Ask gyro for gyro data */
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true);

  /* Save received answer */
  angle_gyro_raw[PITCH] = Wire.read()<<8|Wire.read();
  angle_gyro_raw[ROLL] = Wire.read()<<8|Wire.read();
  angle_gyro_raw[YAW] = Wire.read()<<8|Wire.read();         // Added, did not check if that works

  /* Convert the data to degrees */
  angle_gyro[PITCH] = angle_gyro_raw[PITCH] / 131.0;
  angle_gyro[ROLL] = angle_gyro_raw[ROLL] / 131.0;
  angle_gyro[YAW] = angle_gyro_raw[YAW] / 131.0;              // Added, did not check if that works

  /* Subtract gyro offset value, this is done here, because the total angle is calculated by integration */
  angle_gyro[PITCH] = angle_gyro[PITCH] - angle_gyro_offset[PITCH];
  angle_gyro[ROLL] = angle_gyro[ROLL] - angle_gyro_offset[ROLL];
  angle_gyro[YAW] = angle_gyro[YAW] - angle_gyro_offset[YAW];
}


/**
 *  Reads the accelerometer and saves the values, unneeded as getDataFromNavx does this
 */
void readAccelerometer() {
  /* Ask gyro for acceleration data */
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true);

  /* Save received answer */
  angle_acc_raw[PITCH] = (Wire.read()<<8|Wire.read()) / 16384.0;
  angle_acc_raw[ROLL] = (Wire.read()<<8|Wire.read()) / 16384.0;
  angle_acc_raw[YAW] = (Wire.read()<<8|Wire.read()) / 16384.0;

  /* Convert the data to g */
  angle_acc[PITCH] = atan(angle_acc_raw[ROLL] / sqrt(pow(angle_acc_raw[PITCH], 2) + pow(angle_acc_raw[YAW], 2))) * rad_to_deg;
  angle_acc[ROLL] = atan(-1 * angle_acc_raw[PITCH] / sqrt(pow(angle_acc_raw[ROLL], 2) + pow(angle_acc_raw[YAW], 2))) * rad_to_deg;
  angle_acc[YAW] = atan(angle_acc_raw[PITCH] / angle_acc_raw[ROLL]);
  //angle_acc[YAW] = angle_acc_raw[YAW] / 16384.0;                   // Calculated by my own, don't know if its correct...

  /* Subtract Acc angle offsets, this is done here, since the total angle is calculated by integration and offsets there interfere with integration */
  angle_acc[PITCH] = angle_acc[PITCH] - angle_acc_offset[PITCH];
  angle_acc[ROLL] = angle_acc[ROLL] - angle_acc_offset[ROLL];
  angle_acc[YAW] = angle_acc[YAW] - angle_acc_offset[YAW];
}


/**
 *  Takes 100 samples of current angle measurement and takes average as new offset, unneeded
 */
void calibrateAngleOffsets() {
  float num = 100.0;
  float gyro_avg[3] = {0.0,0.0,0.0};
  float acc_avg[3] = {0.0,0.0,0.0};
  for(int i = 0; i < num; i++){
    // Read Gyro angles and add to average
    readGyro();
    gyro_avg[PITCH] += angle_gyro[PITCH];
    gyro_avg[ROLL] += angle_gyro[ROLL];
    gyro_avg[YAW] += angle_gyro[YAW];
    
    // Read ACC angles and add to average
    readAccelerometer();
    acc_avg[PITCH] += angle_acc[PITCH];
    acc_avg[ROLL] += angle_acc[ROLL];
    acc_avg[YAW] += angle_acc[YAW];
  }

  // divide sums by number of measurements to get average
  angle_gyro_offset[PITCH] = gyro_avg[PITCH] / num;
  angle_gyro_offset[ROLL] = gyro_avg[ROLL] / num;
  angle_gyro_offset[YAW] = gyro_avg[YAW] / num;
  angle_acc_offset[PITCH] = acc_avg[PITCH] / num;
  angle_acc_offset[ROLL] = acc_avg[ROLL] / num;
  angle_acc_offset[YAW] = acc_avg[YAW] / num;
}


/**
 * Receive remote control's command
 * 
 * There are two kind of commands: increase/decrease some value or it's direct value.
 * Direct value commands start with a '.'
 * Each command is determinated with ';'
 */
void receiveControl() {
  if(Serial.available()) {
    // Read until end of command
    String command = Serial.readStringUntil(';');

    if(command[0] == '.') {
      // Receive direct value command

      if(command[1] == 't') {
        throttle = command.substring(2).toInt();                    // Set throttle to value
      } else if(command[1] == 'p') {
        angle_desired[PITCH] = command.substring(2).toFloat();      // Set desired PITCH
      } else if(command[1] == 'r') {
        angle_desired[ROLL] = command.substring(2).toFloat();       // Set Desired ROLL
      } else if(command[1] == 'y') {
        angle_desired[YAW] = command.substring(2).toFloat();        // Set desired YAW
      }

      lastCommand = millis();
    } else {
      // Receive increase/decrease command

      if(command == "throttle+") {
        throttle += 50;                                             // Increase throttle

        if(throttle >= THROTTLE_MAXIMUM) {
          throttle = THROTTLE_MAXIMUM;
        }
      } else if(command == "throttle-") {
        throttle -= 50;                                             // Decrease throttle

        if(throttle <= THROTTLE_MINIMUM) {
          throttle = THROTTLE_MINIMUM;
        }
      } else if(command == "stop") {
        throttle = THROTTLE_MINIMUM;                                // Turn off all motors
      } else if(command == "calibrateAngles") {
        calibrateAngleOffsets();                                    // Calibrate gyro and accelerometer offsets
      } else if(command == "gainP+") {
        gain_p[PITCH] = gain_p[PITCH] + 0.1;                        // Increase P gain for pitch
        gain_p[ROLL] = gain_p[ROLL] + 0.1;                          // Increase P gain for roll
      } else if(command == "gainP-") {
        gain_p[PITCH] = gain_p[PITCH] - 0.1;                        // Decrease P gain for pitch
        gain_p[ROLL] = gain_p[ROLL] - 0.1;                          // Decrease P gain for roll
      } else if(command == "gainD+") {
        gain_d[PITCH] = gain_d[PITCH] + 0.05;                       // Increase D gain for pitch
        gain_d[ROLL] = gain_d[ROLL] + 0.05;                         // Increase D gain for roll
      } else if(command == "gainD-") {
        gain_d[PITCH] = gain_d[PITCH] - 0.05;                       // Decrease D gain for pitch
        gain_d[ROLL] = gain_d[ROLL] - 0.05;                         // Decrease D gain for roll
      } else if(command == "right") {
        angle_desired[ROLL] = angle_desired[ROLL] + 2;              // Move right
      } else if(command == "left") {
        angle_desired[ROLL] = angle_desired[ROLL] - 2;              // Move left
      } else if(command == "filter+") {
        filter = filter + 0.005;                                    // Increase complementary filter
      } else if(command == "filter-") {
        filter = filter - 0.005;                                    // Decrease complementary filter
      } else if(command == "mode0") {
        mode = 0;                                                   // Activate all motors
      } else if(command == "mode1") {
        mode = 1;                                                   // Activate motor 1 & 3
      } else if(command == "mode2") {
        mode = 2;                                                   // Activate motor 2 & 4
      }
    }
  }
}

/**
 * Utility to convert a float to a padded byte array, for serial communication
 */
std::array<unsigned char, 8> floatToPaddedByteArray(float value) {
    std::array<unsigned char, 8> byteArray;
    // Initialize all bytes to 0 (or any other desired padding value)
    byteArray.fill(0); 

    // Copy the 4 bytes of the float into the beginning of the 8-byte array
    std::memcpy(byteArray.data(), &value, sizeof(float)); 
    return byteArray;
}

/**
 * Sends the controller's settings and measured data for three dimensions to the remote controller
 */
void sendData() {
  // Serial.println("B" +
  //   String(throttle) + "|" + 
  //   String(angle_current[YAW]) + "|" + 
  //   String(angle_current[PITCH]) + "|" + 
  //   String(angle_current[ROLL]) + "|" + 
  //   String(angle_desired[YAW]) + "|" +
  //   String(angle_desired[PITCH]) + "|" +
  //   String(angle_desired[ROLL]) + "|" +
  //   String(pid_current[YAW]) + "|" + 
  //   String(pid_current[PITCH]) + "|" + 
  //   String(pid_current[ROLL]) + "E"
  // );
  // start of message, "B"
  Serial.write(0x42);
  // convert to 8 byte array and send each byte
  for (unsigned char byte : floatToPaddedByteArray(throttle)) {
    Serial.write(byte);
  }
  // the pipe "|" character separates each part, 0x7C
  Serial.write(0x7C);
  for (unsigned char byte : floatToPaddedByteArray(angle_current[YAW])) {
    Serial.write(byte);
  }
  Serial.write(0x7C);
  for (unsigned char byte : floatToPaddedByteArray(angle_current[PITCH])) {
    Serial.write(byte);
  }
  Serial.write(0x7C);
  for (unsigned char byte : floatToPaddedByteArray(angle_current[ROLL])) {
    Serial.write(byte);
  }
  Serial.write(0x7C);
  for (unsigned char byte : floatToPaddedByteArray(angle_desired[YAW])) {
    Serial.write(byte);
  }
  Serial.write(0x7C);
  for (unsigned char byte : floatToPaddedByteArray(angle_desired[PITCH])) {
    Serial.write(byte);
  }
  Serial.write(0x7C);
  for (unsigned char byte : floatToPaddedByteArray(angle_desired[ROLL])) {
    Serial.write(byte);
  }
  Serial.write(0x7C);
  for (unsigned char byte : floatToPaddedByteArray(pid_current[YAW])) {
    Serial.write(byte);
  }
  Serial.write(0x7C);
  for (unsigned char byte : floatToPaddedByteArray(pid_current[PITCH])) {
    Serial.write(byte);
  }
  Serial.write(0x7C);
  for (unsigned char byte : floatToPaddedByteArray(pid_current[ROLL])) {
    Serial.write(byte);
  }
  // "E" ends the message
  Serial.write(0x45);
}