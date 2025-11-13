/**
 * The software is provided "as is", without any warranty of any kind.
 * Feel free to edit it if needed.
 *
 * MODIFIED VERSION:
 * - Replaced MPU-6050 (I2C) with ICM-20948 (SPI)
 * - Replaced Direct Port PWM with PCA9685 I2C PWM Driver
 *
 * REQUIRED LIBRARIES:
 * - <SPI.h>
 * - <Wire.h>
 * - <SparkFun_ICM-20948_ArduinoLibrary.h>
 * - <Adafruit_PWMServoDriver.h>
 */

// ---------------------------------------------------------------------------
#include <Wire.h>     // For I2C communication with PCA9685
#include <SPI.h>      // For SPI communication with ICM-20948
#include <SparkFun_ICM-20948_ArduinoLibrary.h> // SparkFun Library for ICM-20948
#include <Adafruit_PWMServoDriver.h>         // Adafruit Library for PCA9685

// ------------------- Define some constants for convenience -----------------
#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define X 0 // X axis
#define Y 1 // Y axis
#define Z 2 // Z axis

// --- PCA9685 Setup ---
// NOTE: Assumes PCA9685, not PCA9531. PCA9531 is an LED driver and will not work.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Default I2C address 0x40
// Motor Mapping to PCA9685 channels
#define MOTOR_A 0 // Front-Left (ESC 1)
#define MOTOR_B 1 // Front-Right (ESC 2)
#define MOTOR_C 2 // Rear-Left (ESC 3)
#define MOTOR_D 3 // Rear-Right (ESC 4)

// --- ICM-20948 Setup ---
#define SPI_CS_PIN 10 // Chip Select pin for SPI
ICM_20948 myICM;      // Create an instance of the ICM_20948 class

#define FREQ 250 // Sampling frequency (Hz)
// #define SSF_GYRO 65.5 // Sensitivity Scale Factor (NO LONGER NEEDED - Library returns dps)

#define STOPPED 0
#define STARTING 1
#define STARTED 2
// ---------------- Receiver variables ---------------------------------------
// Previous state of each channel (HIGH or LOW)
volatile byte previous_state[4];

// Duration of the pulse on each channel of the receiver in µs (must be within 1000µs & 2000µs)
volatile unsigned int pulse_length[4] = {1500, 1500, 1000, 1500};

// Used to calculate pulse duration on each channel
volatile unsigned long current_time;
volatile unsigned long timer[4]; // Timer of each channel

// Used to configure which control (yaw, pitch, roll, throttle) is on which channel
int mode_mapping[4];
// ----------------------- IMU variables -------------------------------------
// The RAW values got from gyro (in °/sec) in that order: X, Y, Z
// Changed to float as library returns processed values, not raw ints
float gyro_raw[3] = {0, 0, 0};

// Average gyro offsets of each axis in that order: X, Y, Z
float gyro_offset[3] = {0, 0, 0};

// Calculated angles from gyro's values in that order: X, Y, Z
float gyro_angle[3] = {0, 0, 0};

// The RAW values got from accelerometer (in m/sec² or g's) in that order: X, Y, Z
// Changed to float as library returns processed values
float acc_raw[3] = {0, 0, 0};

// Calculated angles from accelerometer's values in that order: X, Y, Z
float acc_angle[3] = {0, 0, 0};

// Total 3D acceleration vector
float acc_total_vector;

// Calculated angular motion on each axis: Yaw, Pitch, Roll
float angular_motions[3] = {0, 0, 0};

/**
 * Real measures on 3 axis calculated from gyro AND accelerometer in that order : Yaw, Pitch, Roll
 * - Left wing up implies a positive roll
 * - Nose up implies a positive pitch
 * - Nose right implies a positive yaw
 */
float measures[3] = {0, 0, 0};

// MPU's temperature (now float)
float temperature;

// Init flag set to TRUE after first loop
boolean initialized;
// ----------------------- Variables for servo signal generation -------------
unsigned int period;      // Sampling period (in µs)
unsigned long loop_timer; // Timer to keep main loop at FREQ

// Pulse lengths for ESCs (1000-2000µs)
unsigned long pulse_length_esc1 = 1000,
              pulse_length_esc2 = 1000,
              pulse_length_esc3 = 1000,
              pulse_length_esc4 = 1000;

// ------------- Global variables used for PID controller --------------------
float pid_set_points[3] = {0, 0, 0}; // Yaw, Pitch, Roll
// Errors
float errors[3];                       // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float delta_err[3] = {0, 0, 0};      // Error deltas in that order   : Yaw, Pitch, Roll
float error_sum[3] = {0, 0, 0};      // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
// PID coefficients
float Kp[3] = {4.0, 1.3, 1.3};   // P coefficients in that order : Yaw, Pitch, Roll
float Ki[3] = {0.02, 0.04, 0.04}; // I coefficients in that order : Yaw, Pitch, Roll
float Kd[3] = {0, 18, 18};       // D coefficients in that order : Yaw, Pitch, Roll
// ---------------------------------------------------------------------------
/**
 * Status of the quadcopter:
 * - 0 : stopped
 * - 1 : starting
 * - 2 : started
 *
 * @var int
 */
int status = STOPPED;
// ---------------------------------------------------------------------------
int battery_voltage;
// ---------------------------------------------------------------------------

/**
 * Setup configuration
 */
void setup() {
    // Start Serial for debugging
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("Flight Controller Setup...");

    // Start I2C communication (for PCA9685)
    Wire.begin();
    TWBR = 12; // Set the I2C clock speed to 400kHz.

    // Start SPI communication (for ICM-20948)
    SPI.begin();

    // Turn LED on during setup
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    // Set up the PCA9685 PWM driver
    setupPCA9685();

    // Set up the ICM-20948 IMU
    setupICM20948();

    // Calibrate the IMU
    Serial.println("Calibrating Gyro. Keep the quadcopter level and still...");
    calibrateICM20948();
    Serial.println("Gyro calibration done.");

    configureChannelMapping();

    // Configure interrupts for receiver
    PCICR |= (1 << PCIE0);  // Set PCIE0 to enable PCMSK0 scan
    PCMSK0 |= (1 << PCINT0); // Set PCINT0 (digital input 8) to trigger an interrupt on state change
    PCMSK0 |= (1 << PCINT1); // Set PCINT1 (digital input 9) to trigger an interrupt on state change
    PCMSK0 |= (1 << PCINT2); // Set PCINT2 (digital input 10)to trigger an interrupt on state change
    PCMSK0 |= (1 << PCINT3); // Set PCINT3 (digital input 11)to trigger an interrupt on state change

    period = (1000000 / FREQ); // Sampling period in µs

    // Arm ESCs by sending 1000µs pulse
    Serial.println("Arming ESCs. Make sure throttle is low.");
    Serial.println("Sending 1000us pulse for 2 seconds...");
    stopAll(); // Sets all pulse lengths to 1000
    
    // Send 1000us for a few seconds to arm
    unsigned long arm_timer = millis();
    while (millis() - arm_timer < 2000) {
        // Send 1000us pulses
        pwm.writeMicroseconds(MOTOR_A, 1000);
        pwm.writeMicroseconds(MOTOR_B, 1000);
        pwm.writeMicroseconds(MOTOR_C, 1000);
        pwm.writeMicroseconds(MOTOR_D, 1000);
        delay(5); // Send commands steadily
    }
    Serial.println("Arming complete. Ready to start.");


    // Initialize loop_timer
    loop_timer = micros();

    // Turn LED off now setup is done
    digitalWrite(13, LOW);
}

/**
 * Main program loop
 */
void loop() {
    // 1. Maintain 250Hz loop rate (4000µs period)
    // This blocking loop ensures all calculations and actions finish within the time slice
    while ((micros() - loop_timer) < period);
    loop_timer = micros();

    // 2. First, read raw values from ICM-20948
    readSensor();

    // 3. Calculate angles from gyro & accelerometer's values
    calculateAngles();

    // 4. Calculate set points of PID controller
    calculateSetPoints();

    // 5. Calculate errors comparing angular motions to set points
    calculateErrors();

    if (isStarted()) {
        // 6. Calculate motors speed with PID controller
        pidController();

        // 7. Compensate for battery voltage drop
        compensateBatteryDrop();
    }
    // isStarted() function calls stopAll() when transitioning to STOPPED state,
    // which sets pulse_length_esc* variables to 1000.

    // 8. Apply motors speed (sends pulses via I2C to PCA9685)
    applyMotorSpeed();
}

/**
 * Setup the PCA9685 PWM driver
 */
void setupPCA9685() {
    pwm.begin();
    // This is the typical internal oscillator frequency for the PCA9685
    pwm.setOscillatorFrequency(27000000); 
    // Set PWM frequency to 250Hz (4000µs period)
    pwm.setPWMFreq(FREQ); 
}

/**
 * Send pulse width commands to the PCA9685.
 * This function is now non-blocking.
 */
void applyMotorSpeed() {
    // pulse_length_esc* values are set by pidController() or stopAll()
    pwm.writeMicroseconds(MOTOR_A, pulse_length_esc1);
    pwm.writeMicroseconds(MOTOR_B, pulse_length_esc2);
    pwm.writeMicroseconds(MOTOR_C, pulse_length_esc3);
    pwm.writeMicroseconds(MOTOR_D, pulse_length_esc4);
}

/**
 * Request values from ICM-20948.
 */
void readSensor() {
    // Check if new data is available
    if (myICM.dataReady()) {
        myICM.getAGMT(); // Get all sensor data (Accel, Gyro, Mag, Temp)

        // Assign to our global variables
        // Library returns processed floats (dps, g's)
        acc_raw[X] = myICM.accX();
        acc_raw[Y] = myICM.accY();
        acc_raw[Z] = myICM.accZ();

        gyro_raw[X] = myICM.gyrX();
        gyro_raw[Y] = myICM.gyrY();
        gyro_raw[Z] = myICM.gyrZ();

        // temperature = myICM.temp(); // Available if needed
    }
    // If data is not ready, the old values will be used for this loop iteration
}


/**
 * Calculate real angles from gyro and accelerometer's values
 */
void calculateAngles() {
    calculateGyroAngles();
    calculateAccelerometerAngles();

    if (initialized) {
        // Correct the drift of the gyro with the accelerometer
        gyro_angle[X] = gyro_angle[X] * 0.9996 + acc_angle[X] * 0.0004;
        gyro_angle[Y] = gyro_angle[Y] * 0.9996 + acc_angle[Y] * 0.0004;
    } else {
        // At very first start, init gyro angles with accelerometer angles
        resetGyroAngles();
        initialized = true;
    }

    // To dampen the pitch and roll angles a complementary filter is used
    measures[ROLL] = measures[ROLL] * 0.9 + gyro_angle[X] * 0.1;
    measures[PITCH] = measures[PITCH] * 0.9 + gyro_angle[Y] * 0.1;
    // Store the angular motion for this axis (direct dps value from gyro)
    measures[YAW] = -gyro_raw[Z]; 

    // Apply low-pass filter (10Hz cutoff frequency)
    // No longer need to divide by SSF_GYRO as gyro_raw is already in dps
    angular_motions[ROLL] = 0.7 * angular_motions[ROLL] + 0.3 * gyro_raw[X];
    angular_motions[PITCH] = 0.7 * angular_motions[PITCH] + 0.3 * gyro_raw[Y];
    angular_motions[YAW] = 0.7 * angular_motions[YAW] + 0.3 * gyro_raw[Z];
}

/**
 * Calculate pitch & roll angles using only the gyro.
 */
void calculateGyroAngles() {
    // Subtract offsets
    gyro_raw[X] -= gyro_offset[X];
    gyro_raw[Y] -= gyro_offset[Y];
    gyro_raw[Z] -= gyro_offset[Z];

    // Angle calculation using integration
    // No longer need to divide by SSF_GYRO
    gyro_angle[X] += (gyro_raw[X] / FREQ);
    gyro_angle[Y] += (-gyro_raw[Y] / FREQ); // Change sign to match the accelerometer's one

    // Transfer roll to pitch if IMU has yawed
    // gyro_raw[Z] is now in dps. (dps / FREQ) = degrees per tick. (deg/tick) * (PI/180) = rads/tick
    float yaw_rads_per_tick = gyro_raw[Z] * (1.0 / FREQ) * (PI / 180.0);
    gyro_angle[Y] += gyro_angle[X] * sin(yaw_rads_per_tick);
    gyro_angle[X] -= gyro_angle[Y] * sin(yaw_rads_per_tick);
}

/**
 * Calculate pitch & roll angles using only the accelerometer.
 */
void calculateAccelerometerAngles() {
    // Calculate total 3D acceleration vector : √(X² + Y² + Z²)
    // acc_raw[] values are floats
    acc_total_vector = sqrt(pow(acc_raw[X], 2) + pow(acc_raw[Y], 2) + pow(acc_raw[Z], 2));

    // To prevent asin to produce a NaN, make sure the input value is within [-1;+1]
    // The library returns g's or m/s^2, but the ratio is unitless and valid.
    if (abs(acc_raw[Y]) < acc_total_vector) {
        acc_angle[X] = asin(acc_raw[Y] / acc_total_vector) * (180.0 / PI); // asin gives angle in radian.
    }

    if (abs(acc_raw[X]) < acc_total_vector) {
        acc_angle[Y] = asin(acc_raw[X] / acc_total_vector) * (180.0 / PI);
    }
}


/**
 * Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
 * by applying PID control.
 *
 * (A) (B)     x
 * \ /     z ↑
 * X        |
 * / \       +----→ y
 * (C) (D)
 *
 * Motors A & D run clockwise. (Connected to PCA9685 channels 0, 3)
 * Motors B & C run counter-clockwise. (Connected to PCA9685 channels 1, 2)
 *
 * Each motor output is 1000µs to 2000µs
 */
void pidController() {
    float yaw_pid = 0;
    float pitch_pid = 0;
    float roll_pid = 0;
    int throttle = pulse_length[mode_mapping[THROTTLE]];

    // Initialize motor commands with throttle
    pulse_length_esc1 = throttle;
    pulse_length_esc2 = throttle;
    pulse_length_esc3 = throttle;
    pulse_length_esc4 = throttle;

    // Do not calculate anything if throttle is too low
    if (throttle >= 1012) {
        // PID = e.Kp + ∫e.Ki + Δe.Kd
        yaw_pid = (errors[YAW] * Kp[YAW]) + (error_sum[YAW] * Ki[YAW]) + (delta_err[YAW] * Kd[YAW]);
        pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
        roll_pid = (errors[ROLL] * Kp[ROLL]) + (error_sum[ROLL] * Ki[ROLL]) + (delta_err[ROLL] * Kd[ROLL]);

        // Keep values within acceptable range.
        yaw_pid = minMax(yaw_pid, -400, 400);
        pitch_pid = minMax(pitch_pid, -400, 400);
        roll_pid = minMax(roll_pid, -400, 400);

        // Calculate pulse duration for each ESC
        // Motor A (Front-Left, CW)
        pulse_length_esc1 = throttle - roll_pid - pitch_pid + yaw_pid;
        // Motor B (Front-Right, CCW)
        pulse_length_esc2 = throttle + roll_pid - pitch_pid - yaw_pid;
        // Motor C (Rear-Left, CCW)
        pulse_length_esc3 = throttle - roll_pid + pitch_pid - yaw_pid;
        // Motor D (Rear-Right, CW)
        pulse_length_esc4 = throttle + roll_pid + pitch_pid + yaw_pid;
    }

    // Prevent out-of-range-values
    pulse_length_esc1 = minMax(pulse_length_esc1, 1100, 2000);
    pulse_length_esc2 = minMax(pulse_length_esc2, 1100, 2000);
    pulse_length_esc3 = minMax(pulse_length_esc3, 1100, 2000);
    pulse_length_esc4 = minMax(pulse_length_esc4, 1100, 2000);
}

/**
 * Calculate errors used by PID controller
 */
void calculateErrors() {
    // Calculate current errors
    errors[YAW] = angular_motions[YAW] - pid_set_points[YAW];
    errors[PITCH] = angular_motions[PITCH] - pid_set_points[PITCH];
    errors[ROLL] = angular_motions[ROLL] - pid_set_points[ROLL];

    // Calculate sum of errors : Integral coefficients
    error_sum[YAW] += errors[YAW];
    error_sum[PITCH] += errors[PITCH];
    error_sum[ROLL] += errors[ROLL];

    // Keep values in acceptable range
    error_sum[YAW] = minMax(error_sum[YAW], -400 / Ki[YAW], 400 / Ki[YAW]);
    error_sum[PITCH] = minMax(error_sum[PITCH], -400 / Ki[PITCH], 400 / Ki[PITCH]);
    error_sum[ROLL] = minMax(error_sum[ROLL], -400 / Ki[ROLL], 400 / Ki[ROLL]);

    // Calculate error delta : Derivative coefficients
    delta_err[YAW] = errors[YAW] - previous_error[YAW];
    delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
    delta_err[ROLL] = errors[ROLL] - previous_error[ROLL];

    // Save current error as previous_error for next time
    previous_error[YAW] = errors[YAW];
    previous_error[PITCH] = errors[PITCH];
    previous_error[ROLL] = errors[ROLL];
}

/**
 * Customize mapping of controls: set here which command is on which channel and call
 * this function in setup() routine.
 */
void configureChannelMapping() {
    mode_mapping[YAW] = CHANNEL4;
    mode_mapping[PITCH] = CHANNEL2;
    mode_mapping[ROLL] = CHANNEL1;
    mode_mapping[THROTTLE] = CHANNEL3;
}

/**
 * Configure ICM-20948 gyro and accelerometer settings.
 *
 * Gyro: ±500°/s (Matches original SSF_GYRO of 65.5)
 * Accel: ±8g (Matches original 0x10)
 * DLPF: ~45Hz (Matches original 0x03)
 */
void setupICM20948() {
    Serial.println("Initializing ICM-20948 IMU...");
    
    // Attempt to initialize the IMU
    myICM.begin(SPI_CS_PIN, SPI);

    int max_retries = 5;
    while (myICM.status != ICM_20948_Stat_Ok && max_retries-- > 0) {
        Serial.println("ICM-20948 initialization failed. Retrying...");
        delay(500);
        myICM.begin(SPI_CS_PIN, SPI);
    }

    if (myICM.status != ICM_20948_Stat_Ok) {
        Serial.println("ICM-20948 initialization failed. Halting.");
        digitalWrite(13, LOW); // Turn off LED
        while (1); // Freeze
    }
    Serial.println("ICM-20948 successfully initialized.");

    // Set Gyro Full-Scale Range to ±500dps
    myICM.setGyroFSR(ICM_20948_GYRO_FSR_500DPS);
    // Set Accel Full-Scale Range to ±8g
    myICM.setAccelFSR(ICM_20948_ACCEL_FSR_8G);

    // Set Gyro Digital Low Pass Filter
    myICM.setGyroDLPF(ICM_20948_DLPF_CFG_3); // ~45Hz bandwidth
    // Set Accel Digital Low Pass Filter
    myICM.setAccelDLPF(ICM_20948_DLPF_CFG_3); // ~45Hz bandwidth
}


/**
 * Calibrate ICM-20948: take 2000 samples to calculate average gyro offsets.
 * During this step, the quadcopter needs to be static and on a horizontal surface.
 */
void calibrateICM20948() {
    int max_samples = 2000;

    for (int i = 0; i < max_samples; i++) {
        // Wait for data to be ready
        while (!myICM.dataReady());
        myICM.getAGMT(); // Get all sensor data

        gyro_offset[X] += myICM.gyrX();
        gyro_offset[Y] += myICM.gyrY();
        gyro_offset[Z] += myICM.gyrZ();

        // Short delay before next sample
        delay(3);

        if (i % 200 == 0) {
            Serial.print(".");
        }
    }
    Serial.println();

    // Calculate average offsets
    gyro_offset[X] /= max_samples;
    gyro_offset[Y] /= max_samples;
    gyro_offset[Z] /= max_samples;
}

/**
 * Make sure that given value is not over min_value/max_value range.
 */
float minMax(float value, float min_value, float max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }
    return value;
}

/**
 * Return whether the quadcopter is started.
 * To start the quadcopter, move the left stick in bottom left corner then, move it back in center position.
 * To stop the quadcopter move the left stick in bottom right corner.
 *
 * @return bool
 */
bool isStarted() {
    // When left stick is moved in the bottom left corner
    if (status == STOPPED && pulse_length[mode_mapping[YAW]] <= 1012 && pulse_length[mode_mapping[THROTTLE]] <= 1012) {
        status = STARTING;
    }

    // When left stick is moved back in the center position
    if (status == STARTING && pulse_length[mode_mapping[YAW]] == 1500 && pulse_length[mode_mapping[THROTTLE]] <= 1012) {
        status = STARTED;
        Serial.println("STATUS: STARTED");

        // Reset PID controller's variables to prevent bump start
        resetPidController();
        resetGyroAngles();
    }

    // When left stick is moved in the bottom right corner
    if (status == STARTED && pulse_length[mode_mapping[YAW]] >= 1988 && pulse_length[mode_mapping[THROTTLE]] <= 1012) {
        status = STOPPED;
        Serial.println("STATUS: STOPPED");
        // Make sure to always stop motors when status is STOPPED
        stopAll();
    }

    return status == STARTED;
}

/**
 * Reset gyro's angles with accelerometer's angles.
 */
void resetGyroAngles() {
    gyro_angle[X] = acc_angle[X];
    gyro_angle[Y] = acc_angle[Y];
}

/**
 * Reset motors' pulse length to 1000µs to totally stop them.
 */
void stopAll() {
    pulse_length_esc1 = 1000;
    pulse_length_esc2 = 1000;
    pulse_length_esc3 = 1000;
    pulse_length_esc4 = 1000;
}

/**
 * Reset all PID controller's variables.
 */
void resetPidController() {
    errors[YAW] = 0;
    errors[PITCH] = 0;
    errors[ROLL] = 0;

    error_sum[YAW] = 0;
    error_sum[PITCH] = 0;
    error_sum[ROLL] = 0;

    previous_error[YAW] = 0;
    previous_error[PITCH] = 0;
    previous_error[ROLL] = 0;
}

/**
 * Calculate PID set points on axis YAW, PITCH, ROLL
 */
void calculateSetPoints() {
    pid_set_points[YAW] = calculateYawSetPoint(pulse_length[mode_mapping[YAW]], pulse_length[mode_mapping[THROTTLE]]);
    pid_set_points[PITCH] = calculateSetPoint(measures[PITCH], pulse_length[mode_mapping[PITCH]]);
    pid_set_points[ROLL] = calculateSetPoint(measures[ROLL], pulse_length[mode_mapping[ROLL]]);
}

/**
 * Calculate the PID set point in °/s
 *
 * @param float angle        Measured angle (in °) on an axis
 * @param int   channel_pulse Pulse length of the corresponding receiver channel
 * @return float
 */
float calculateSetPoint(float angle, int channel_pulse) {
    float level_adjust = angle * 15; // Value 15 limits maximum angle value to ±32.8°
    float set_point = 0;

    // Need a dead band of 16µs for better result
    if (channel_pulse > 1508) {
        set_point = channel_pulse - 1508;
    } else if (channel_pulse < 1492) {
        set_point = channel_pulse - 1492;
    }

    set_point -= level_adjust;
    set_point /= 3;

    return set_point;
}

/**
 * Calculate the PID set point of YAW axis in °/s
 *
 * @param int yaw_pulse      Receiver pulse length of yaw's channel
 * @param int throttle_pulse Receiver pulse length of throttle's channel
 * @return float
 */
float calculateYawSetPoint(int yaw_pulse, int throttle_pulse) {
    float set_point = 0;

    // Do not yaw when turning off the motors
    if (throttle_pulse > 1050) {
        // There is no notion of angle on this axis as the quadcopter can turn on itself
        set_point = calculateSetPoint(0, yaw_pulse);
    }

    return set_point;
}

/**
 * Compensate battery drop applying a coefficient on output values
 */
void compensateBatteryDrop() {
    if (isBatteryConnected()) {
        pulse_length_esc1 += pulse_length_esc1 * ((1240 - battery_voltage) / (float)3500);
        pulse_length_esc2 += pulse_length_esc2 * ((1240 - battery_voltage) / (float)3500);
        pulse_length_esc3 += pulse_length_esc3 * ((1240 - battery_voltage) / (float)3500);
        pulse_length_esc4 += pulse_length_esc4 * ((1240 - battery_voltage) / (float)3500);
    }
}

/**
 * Read battery voltage & return whether the battery seems connected
 *
 * @return boolean
 */
bool isBatteryConnected() {
    // Reduce noise with a low-pass filter (10Hz cutoff frequency)
    battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

    return battery_voltage < 1240 && battery_voltage > 800;
}

/**
 * This Interrupt Sub Routine is called each time input 8, 9, 10 or 11 changed state.
 * Read the receiver signals in order to get flight instructions.
 */
ISR(PCINT0_vect) {
    current_time = micros();

    // Channel 1 -------------------------------------------------
    if (PINB & B00000001) { // Is input 8 high ?
        if (previous_state[CHANNEL1] == LOW) { // Input 8 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL1] = HIGH;   // Save current state
            timer[CHANNEL1] = current_time;  // Save current time
        }
    } else if (previous_state[CHANNEL1] == HIGH) { // Input 8 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL1] = LOW;            // Save current state
        pulse_length[CHANNEL1] = current_time - timer[CHANNEL1]; // Calculate pulse duration & save it
    }

    // Channel 2 -------------------------------------------------
    if (PINB & B00000010) { // Is input 9 high ?
        if (previous_state[CHANNEL2] == LOW) { // Input 9 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL2] = HIGH;   // Save current state
            timer[CHANNEL2] = current_time;  // Save current time
        }
    } else if (previous_state[CHANNEL2] == HIGH) { // Input 9 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL2] = LOW;            // Save current state
        pulse_length[CHANNEL2] = current_time - timer[CHANNEL2]; // Calculate pulse duration & save it
    }

    // Channel 3 -------------------------------------------------
    if (PINB & B00000100) { // Is input 10 high ?
        if (previous_state[CHANNEL3] == LOW) { // Input 10 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL3] = HIGH;   // Save current state
            timer[CHANNEL3] = current_time;  // Save current time
        }
    } else if (previous_state[CHANNEL3] == HIGH) { // Input 10 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL3] = LOW;            // Save current state
        pulse_length[CHANNEL3] = current_time - timer[CHANNEL3]; // Calculate pulse duration & save it
    }

    // Channel 4 -------------------------------------------------
    if (PINB & B00001000) { // Is input 11 high ?
        if (previous_state[CHANNEL4] == LOW) { // Input 11 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL4] = HIGH;   // Save current state
            timer[CHANNEL4] = current_time;  // Save current time
        }
    } else if (previous_state[CHANNEL4] == HIGH) { // Input 11 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL4] = LOW;            // Save current state
        pulse_length[CHANNEL4] = current_time - timer[CHANNEL4]; // Calculate pulse duration & save it
    }
}