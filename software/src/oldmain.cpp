// flight_controller_linux.cpp
// Linux-port of Arduino flight controller (PID + IMU + PCA9685 abstraction).
// Exposes four control inputs via methods: setYawPulse/setPitchPulse/setRollPulse/setThrottlePulse.
//
// Build: g++ -std=c++17 flight_controller_linux.cpp -pthread -o flight_controller_linux

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ---------------- Constants (kept from original) ----------------
constexpr int CHANNEL1 = 0;
constexpr int CHANNEL2 = 1;
constexpr int CHANNEL3 = 2;
constexpr int CHANNEL4 = 3;

constexpr int YAW = 0;
constexpr int PITCH = 1;
constexpr int ROLL = 2;
constexpr int THROTTLE = 3;

constexpr int X_AXIS = 0;
constexpr int Y_AXIS = 1;
constexpr int Z_AXIS = 2;

constexpr int MOTOR_A = 0; // FL
constexpr int MOTOR_B = 1; // FR
constexpr int MOTOR_C = 2; // RL
constexpr int MOTOR_D = 3; // RR

constexpr int FREQ = 250; // Hz -> 4000us period

// ---------------- Interfaces for hardware abstraction ----------------
struct IIMU {
    virtual ~IIMU() = default;
    virtual bool dataReady() = 0;
    virtual void getAGMT() = 0;
    virtual float accX() = 0;
    virtual float accY() = 0;
    virtual float accZ() = 0;
    virtual float gyrX() = 0;
    virtual float gyrY() = 0;
    virtual float gyrZ() = 0;
    virtual float temperature() = 0;
};

struct IPWMDriver {
    virtual ~IPWMDriver() = default;
    virtual void begin() = 0;
    virtual void setOscillatorFrequency(unsigned long freq) = 0;
    virtual void setPWMFreq(int freq) = 0;
    virtual void writeMicroseconds(int channel, unsigned int pulse_us) = 0;
};

// ---------------- Mock implementations (useful for testing) ----------------
struct MockIMU : public IIMU {
    // Simple mock that produces zeroed sensors
    bool dataReady() override { return true; }
    void getAGMT() override {}
    float accX() override { return 0.0f; }
    float accY() override { return 0.0f; }
    float accZ() override { return 1.0f; } // gravity
    float gyrX() override { return 0.0f; }
    float gyrY() override { return 0.0f; }
    float gyrZ() override { return 0.0f; }
    float temperature() override { return 25.0f; }
};

struct MockPWMDriver : public IPWMDriver {
    void begin() override { std::cout << "[MockPWM] begin()\n"; }
    void setOscillatorFrequency(unsigned long f) override { (void)f; }
    void setPWMFreq(int f) override { std::cout << "[MockPWM] set freq " << f << "Hz\n"; }
    void writeMicroseconds(int channel, unsigned int pulse_us) override {
        std::cout << "[MockPWM] ch" << channel << " -> " << pulse_us << "us\n";
    }
};

// ---------------- FlightController ----------------
class FlightController {
public:
    FlightController(std::unique_ptr<IIMU> imu,
                     std::unique_ptr<IPWMDriver> pwm,
                     std::function<int()> battery_reader = [](){ return -1; })
        : imu_(std::move(imu)), pwm_(std::move(pwm)), battery_reader_(battery_reader)
    {
        initDefaults();
    }

    ~FlightController() {
        stop();
    }

    // Exposed control input setters (thread-safe)
    void setYawPulse(unsigned int us)      { setPulseAtomic(CHANNEL4, us); }
    void setPitchPulse(unsigned int us)    { setPulseAtomic(CHANNEL2, us); }
    void setRollPulse(unsigned int us)     { setPulseAtomic(CHANNEL1, us); }
    void setThrottlePulse(unsigned int us) { setPulseAtomic(CHANNEL3, us); }

    // Start/stop the control loop
    void start() {
        bool expected = false;
        if (!running_.compare_exchange_strong(expected, true)) return;
        pwm_->begin();
        pwm_->setOscillatorFrequency(27000000);
        pwm_->setPWMFreq(FREQ);
        // Arm ESCs by sending 1000us for 2 seconds (non-blocking here)
        stopAll(); // sets esc pulses to 1000
        armLoop(); // send initial pulses synchronously
        worker_ = std::thread(&FlightController::loopThread, this);
    }

    void stop() {
        bool expected = true;
        if (!running_.compare_exchange_strong(expected, false)) {
            if (worker_.joinable()) worker_.join();
            return;
        }
        if (worker_.joinable()) worker_.join();
    }

    // Get last computed ESC pulses (useful for testing)
    std::array<unsigned int,4> getMotorPulses() const {
        std::lock_guard<std::mutex> lk(motor_mutex_);
        return {pulse_length_esc1_, pulse_length_esc2_, pulse_length_esc3_, pulse_length_esc4_};
    }

private:
    // Hardware
    std::unique_ptr<IIMU> imu_;
    std::unique_ptr<IPWMDriver> pwm_;
    std::function<int()> battery_reader_;

    // Receiver pulses (like volatile pulse_length[] in original)
    std::array<std::atomic<unsigned int>,4> pulse_length_{ {1500,1500,1000,1500} };

    // Mapping (same as original configureChannelMapping)
    std::array<int,4> mode_mapping_;

    // IMU-related state
    std::array<float,3> gyro_raw_{0,0,0};
    std::array<float,3> gyro_offset_{0,0,0};
    std::array<float,3> gyro_angle_{0,0,0};
    std::array<float,3> acc_raw_{0,0,0};
    std::array<float,3> acc_angle_{0,0,0};
    float acc_total_vector_{0.0f};
    std::array<float,3> angular_motions_{0,0,0};
    std::array<float,3> measures_{0,0,0};
    float temperature_{0.0f};
    bool initialized_ = false;

    // ESC output pulses (protected by motor_mutex_)
    mutable std::mutex motor_mutex_;
    unsigned int pulse_length_esc1_ = 1000,
                 pulse_length_esc2_ = 1000,
                 pulse_length_esc3_ = 1000,
                 pulse_length_esc4_ = 1000;

    // PID vars
    std::array<float,3> pid_set_points_{0,0,0};
    std::array<float,3> errors_{0,0,0};
    std::array<float,3> delta_err_{0,0,0};
    std::array<float,3> error_sum_{0,0,0};
    std::array<float,3> previous_error_{0,0,0};
    std::array<float,3> Kp_{4.0f,1.3f,1.3f};
    std::array<float,3> Ki_{0.02f,0.04f,0.04f};
    std::array<float,3> Kd_{0.0f,18.0f,18.0f};

    // Status
    enum { STOPPED=0, STARTING=1, STARTED=2 };
    std::atomic<int> status_{STOPPED};

    // Battery
    int battery_voltage_ = 0;

    // Threading
    std::atomic<bool> running_{false};
    std::thread worker_;

    // Timing helper
    static long micros_now() {
        using namespace std::chrono;
        return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
    }

    // Initialize default mapping & variables
    void initDefaults() {
        mode_mapping_[YAW] = CHANNEL4;
        mode_mapping_[PITCH] = CHANNEL2;
        mode_mapping_[ROLL] = CHANNEL1;
        mode_mapping_[THROTTLE] = CHANNEL3;

        // default pulses are already set in atomic initialization
    }

    void setPulseAtomic(int channel, unsigned int us) {
        if (channel < 0 || channel > 3) return;
        pulse_length_[channel].store(us, std::memory_order_relaxed);
    }

    // Arm loop: sends 1000us for 2 seconds on the mock/real driver (blocking at start)
    void armLoop() {
        using namespace std::chrono;
        auto start = steady_clock::now();
        while (steady_clock::now() - start < seconds(2)) {
            pwm_->writeMicroseconds(MOTOR_A, 1000);
            pwm_->writeMicroseconds(MOTOR_B, 1000);
            pwm_->writeMicroseconds(MOTOR_C, 1000);
            pwm_->writeMicroseconds(MOTOR_D, 1000);
            std::this_thread::sleep_for(milliseconds(5));
        }
    }

    // Main loop thread
    void loopThread() {
        using namespace std::chrono;
        const long period_us = 1000000 / FREQ;
        auto last_loop = steady_clock::now();

        while (running_.load()) {
            // enforce loop rate
            auto now = steady_clock::now();
            auto elapsed = duration_cast<microseconds>(now - last_loop).count();
            if (elapsed < period_us) {
                std::this_thread::sleep_for(microseconds(period_us - elapsed));
            }
            last_loop = steady_clock::now();

            // 1. Read sensors
            readSensor();

            // 2. Calculate angles
            calculateAngles();

            // 3. Calculate set points
            calculateSetPoints();

            // 4. Calculate errors
            calculateErrors();

            if (isStarted()) {
                // 5. PID controller
                pidController();

                // 6. Battery compensation
                compensateBatteryDrop();
            }
            // 7. Apply motor speed
            applyMotorSpeed();
        }
        // On exit, ensure motors are stopped
        stopAll();
        applyMotorSpeed();
    }

    // ---------------- Sensor / IMU conversions ---------------
    void readSensor() {
        if (imu_->dataReady()) {
            imu_->getAGMT();
            acc_raw_[X_AXIS] = imu_->accX();
            acc_raw_[Y_AXIS] = imu_->accY();
            acc_raw_[Z_AXIS] = imu_->accZ();
            gyro_raw_[X_AXIS] = imu_->gyrX();
            gyro_raw_[Y_AXIS] = imu_->gyrY();
            gyro_raw_[Z_AXIS] = imu_->gyrZ();
            temperature_ = imu_->temperature();
        }
    }

    void calculateAngles() {
        calculateGyroAngles();
        calculateAccelerometerAngles();

        if (initialized_) {
            gyro_angle_[X_AXIS] = gyro_angle_[X_AXIS] * 0.9996f + acc_angle_[X_AXIS] * 0.0004f;
            gyro_angle_[Y_AXIS] = gyro_angle_[Y_AXIS] * 0.9996f + acc_angle_[Y_AXIS] * 0.0004f;
        } else {
            resetGyroAngles();
            initialized_ = true;
        }

        measures_[ROLL] = measures_[ROLL] * 0.9f + gyro_angle_[X_AXIS] * 0.1f;
        measures_[PITCH] = measures_[PITCH] * 0.9f + gyro_angle_[Y_AXIS] * 0.1f;
        measures_[YAW] = -gyro_raw_[Z_AXIS];

        angular_motions_[ROLL] = 0.7f * angular_motions_[ROLL] + 0.3f * gyro_raw_[X_AXIS];
        angular_motions_[PITCH] = 0.7f * angular_motions_[PITCH] + 0.3f * gyro_raw_[Y_AXIS];
        angular_motions_[YAW] = 0.7f * angular_motions_[YAW] + 0.3f * gyro_raw_[Z_AXIS];
    }

    void calculateGyroAngles() {
        gyro_raw_[X_AXIS] -= gyro_offset_[X_AXIS];
        gyro_raw_[Y_AXIS] -= gyro_offset_[Y_AXIS];
        gyro_raw_[Z_AXIS] -= gyro_offset_[Z_AXIS];

        gyro_angle_[X_AXIS] += (gyro_raw_[X_AXIS] / FREQ);
        gyro_angle_[Y_AXIS] += (-gyro_raw_[Y_AXIS] / FREQ);

        float yaw_rads_per_tick = gyro_raw_[Z_AXIS] * (1.0f / FREQ) * (M_PI / 180.0f);
        gyro_angle_[Y_AXIS] += gyro_angle_[X_AXIS] * sinf(yaw_rads_per_tick);
        gyro_angle_[X_AXIS] -= gyro_angle_[Y_AXIS] * sinf(yaw_rads_per_tick);
    }

    void calculateAccelerometerAngles() {
        acc_total_vector_ = std::sqrt(acc_raw_[X_AXIS]*acc_raw_[X_AXIS] +
                                      acc_raw_[Y_AXIS]*acc_raw_[Y_AXIS] +
                                      acc_raw_[Z_AXIS]*acc_raw_[Z_AXIS]);
        if (std::abs(acc_raw_[Y_AXIS]) < acc_total_vector_) {
            acc_angle_[X_AXIS] = asinf(acc_raw_[Y_AXIS] / acc_total_vector_) * (180.0f / M_PI);
        }
        if (std::abs(acc_raw_[X_AXIS]) < acc_total_vector_) {
            acc_angle_[Y_AXIS] = asinf(acc_raw_[X_AXIS] / acc_total_vector_) * (180.0f / M_PI);
        }
    }

    void resetGyroAngles() {
        gyro_angle_[X_AXIS] = acc_angle_[X_AXIS];
        gyro_angle_[Y_AXIS] = acc_angle_[Y_AXIS];
    }

    // ---------------- PID / motor control ---------------
    void pidController() {
        float yaw_pid = 0.0f;
        float pitch_pid = 0.0f;
        float roll_pid = 0.0f;

        unsigned int throttle = pulse_length_[mode_mapping_[THROTTLE]].load(std::memory_order_relaxed);

        {
            std::lock_guard<std::mutex> lk(motor_mutex_);
            pulse_length_esc1_ = throttle;
            pulse_length_esc2_ = throttle;
            pulse_length_esc3_ = throttle;
            pulse_length_esc4_ = throttle;
        }

        if (throttle >= 1012) {
            yaw_pid = (errors_[YAW] * Kp_[YAW]) + (error_sum_[YAW] * Ki_[YAW]) + (delta_err_[YAW] * Kd_[YAW]);
            pitch_pid = (errors_[PITCH] * Kp_[PITCH]) + (error_sum_[PITCH] * Ki_[PITCH]) + (delta_err_[PITCH] * Kd_[PITCH]);
            roll_pid = (errors_[ROLL] * Kp_[ROLL]) + (error_sum_[ROLL] * Ki_[ROLL]) + (delta_err_[ROLL] * Kd_[ROLL]);

            yaw_pid = minMax(yaw_pid, -400.0f, 400.0f);
            pitch_pid = minMax(pitch_pid, -400.0f, 400.0f);
            roll_pid = minMax(roll_pid, -400.0f, 400.0f);

            unsigned int a = clampToUnsignedInt((float)throttle - roll_pid - pitch_pid + yaw_pid);
            unsigned int b = clampToUnsignedInt((float)throttle + roll_pid - pitch_pid - yaw_pid);
            unsigned int c = clampToUnsignedInt((float)throttle - roll_pid + pitch_pid - yaw_pid);
            unsigned int d = clampToUnsignedInt((float)throttle + roll_pid + pitch_pid + yaw_pid);

            std::lock_guard<std::mutex> lk(motor_mutex_);
            pulse_length_esc1_ = minMaxUint(a, 1100u, 2000u);
            pulse_length_esc2_ = minMaxUint(b, 1100u, 2000u);
            pulse_length_esc3_ = minMaxUint(c, 1100u, 2000u);
            pulse_length_esc4_ = minMaxUint(d, 1100u, 2000u);
        } else {
            // if below threshold, ensure outputs are safe-limited
            std::lock_guard<std::mutex> lk(motor_mutex_);
            pulse_length_esc1_ = minMaxUint(pulse_length_esc1_, 1000u, 2000u);
            pulse_length_esc2_ = minMaxUint(pulse_length_esc2_, 1000u, 2000u);
            pulse_length_esc3_ = minMaxUint(pulse_length_esc3_, 1000u, 2000u);
            pulse_length_esc4_ = minMaxUint(pulse_length_esc4_, 1000u, 2000u);
        }
    }

    void calculateErrors() {
        errors_[YAW] = angular_motions_[YAW] - pid_set_points_[YAW];
        errors_[PITCH] = angular_motions_[PITCH] - pid_set_points_[PITCH];
        errors_[ROLL] = angular_motions_[ROLL] - pid_set_points_[ROLL];

        error_sum_[YAW] += errors_[YAW];
        error_sum_[PITCH] += errors_[PITCH];
        error_sum_[ROLL] += errors_[ROLL];

        if (Ki_[YAW] != 0.0f) error_sum_[YAW] = minMax(error_sum_[YAW], -400.0f / Ki_[YAW], 400.0f / Ki_[YAW]);
        if (Ki_[PITCH] != 0.0f) error_sum_[PITCH] = minMax(error_sum_[PITCH], -400.0f / Ki_[PITCH], 400.0f / Ki_[PITCH]);
        if (Ki_[ROLL] != 0.0f) error_sum_[ROLL] = minMax(error_sum_[ROLL], -400.0f / Ki_[ROLL], 400.0f / Ki_[ROLL]);

        delta_err_[YAW] = errors_[YAW] - previous_error_[YAW];
        delta_err_[PITCH] = errors_[PITCH] - previous_error_[PITCH];
        delta_err_[ROLL] = errors_[ROLL] - previous_error_[ROLL];

        previous_error_[YAW] = errors_[YAW];
        previous_error_[PITCH] = errors_[PITCH];
        previous_error_[ROLL] = errors_[ROLL];
    }

    void calculateSetPoints() {
        pid_set_points_[YAW] = calculateYawSetPoint(
            static_cast<int>(pulse_length_[mode_mapping_[YAW]].load()), 
            static_cast<int>(pulse_length_[mode_mapping_[THROTTLE]].load())
        );
        pid_set_points_[PITCH] = calculateSetPoint(measures_[PITCH],
            static_cast<int>(pulse_length_[mode_mapping_[PITCH]].load()));
        pid_set_points_[ROLL] = calculateSetPoint(measures_[ROLL],
            static_cast<int>(pulse_length_[mode_mapping_[ROLL]].load()));
    }

    float calculateSetPoint(float angle, int channel_pulse) {
        float level_adjust = angle * 15.0f;
        float set_point = 0.0f;
        if (channel_pulse > 1508) {
            set_point = channel_pulse - 1508;
        } else if (channel_pulse < 1492) {
            set_point = channel_pulse - 1492;
        }
        set_point -= level_adjust;
        set_point /= 3.0f;
        return set_point;
    }

    float calculateYawSetPoint(int yaw_pulse, int throttle_pulse) {
        float set_point = 0.0f;
        if (throttle_pulse > 1050) {
            set_point = calculateSetPoint(0.0f, yaw_pulse);
        }
        return set_point;
    }

    void compensateBatteryDrop() {
        int v = battery_reader_();
        if (v > 0) {
            battery_voltage_ = static_cast<int>(battery_voltage_ * 0.92 + v * 0.09853);
            std::lock_guard<std::mutex> lk(motor_mutex_);
            pulse_length_esc1_ += static_cast<unsigned int>(pulse_length_esc1_ * ((1240 - battery_voltage_) / 3500.0f));
            pulse_length_esc2_ += static_cast<unsigned int>(pulse_length_esc2_ * ((1240 - battery_voltage_) / 3500.0f));
            pulse_length_esc3_ += static_cast<unsigned int>(pulse_length_esc3_ * ((1240 - battery_voltage_) / 3500.0f));
            pulse_length_esc4_ += static_cast<unsigned int>(pulse_length_esc4_ * ((1240 - battery_voltage_) / 3500.0f));
        }
    }

    bool isStarted() {
        int s = status_.load();
        unsigned int yaw_pulse = pulse_length_[mode_mapping_[YAW]].load();
        unsigned int throttle_pulse = pulse_length_[mode_mapping_[THROTTLE]].load();

        if (s == STOPPED && yaw_pulse <= 1012 && throttle_pulse <= 1012) {
            status_.store(STARTING);
        }
        if (s == STARTING && yaw_pulse == 1500 && throttle_pulse <= 1012) {
            status_.store(STARTED);
            std::cout << "STATUS: STARTED\n";
            resetPidController();
            resetGyroAngles();
        }
        if (s == STARTED && yaw_pulse >= 1988 && throttle_pulse <= 1012) {
            status_.store(STOPPED);
            std::cout << "STATUS: STOPPED\n";
            stopAll();
        }

        return status_.load() == STARTED;
    }

    void stopAll() {
        std::lock_guard<std::mutex> lk(motor_mutex_);
        pulse_length_esc1_ = 1000;
        pulse_length_esc2_ = 1000;
        pulse_length_esc3_ = 1000;
        pulse_length_esc4_ = 1000;
    }

    void resetPidController() {
        errors_.fill(0.0f);
        error_sum_.fill(0.0f);
        previous_error_.fill(0.0f);
        delta_err_.fill(0.0f);
    }

    void applyMotorSpeed() {
        std::lock_guard<std::mutex> lk(motor_mutex_);
        pwm_->writeMicroseconds(MOTOR_A, pulse_length_esc1_);
        pwm_->writeMicroseconds(MOTOR_B, pulse_length_esc2_);
        pwm_->writeMicroseconds(MOTOR_C, pulse_length_esc3_);
        pwm_->writeMicroseconds(MOTOR_D, pulse_length_esc4_);
    }

    // ---------------- Utility helpers ----------------
    static float minMax(float value, float min_value, float max_value) {
        if (value > max_value) return max_value;
        if (value < min_value) return min_value;
        return value;
    }
    static unsigned int minMaxUint(unsigned int value, unsigned int min_value, unsigned int max_value) {
        if (value > max_value) return max_value;
        if (value < min_value) return min_value;
        return value;
    }
    static unsigned int clampToUnsignedInt(float v) {
        if (v < 0.0f) return 0u;
        if (v > 65535.0f) return 65535u;
        return static_cast<unsigned int>(v);
    }
};

// ---------------- Example usage ----------------
int main() {
    // Create controller with mock IMU + mock PWM. Replace with real implementations for hardware.
    auto imu = std::make_unique<MockIMU>();
    auto pwm = std::make_unique<MockPWMDriver>();

    // Optional battery reader lambda (read from ADC/sysfs in a real driver)
    auto battery_reader = []() -> int {
        return -1; // disabled in this mock example
    };

    FlightController fc(std::move(imu), std::move(pwm), battery_reader);

    // Start controller (starts worker thread)
    fc.start();

    // Example: feed control inputs (the API you asked for)
    fc.setThrottlePulse(1200);
    fc.setYawPulse(1500);
    fc.setPitchPulse(1500);
    fc.setRollPulse(1500);

    // Let it run a short time
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Simulate stick move to "start" (as original logic expects yaw low + throttle low -> then center)
    fc.setYawPulse(1000);
    fc.setThrottlePulse(1000);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    fc.setYawPulse(1500); // back to center
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Query motor pulses
    auto motors = fc.getMotorPulses();
    std::cout << "Motor pulses: " << motors[0] << ", " << motors[1] << ", " << motors[2] << ", " << motors[3] << "\n";

    // Stop controller and exit
    fc.stop();
    return 0;
}
