#include <iostream>
#include <fcntl.h> // For open()
#include <unistd.h> // For close(), read(), write()
#include <sys/ioctl.h> // For ioctl()
#include <linux/i2c-dev.h> // For I2C_SLAVE

// PCA9531 I2C address (adjust if different)
#define PCA9531_ADDRESS 0x60

// PCA9531 Register addresses
#define PCA9531_INPUT_REG 0x00
#define PCA9531_PSC0_REG 0x01
#define PCA9531_PWM0_REG 0x02
#define PCA9531_PSC1_REG 0x03
#define PCA9531_PWM1_REG 0x04
#define PCA9531_LS0_REG 0x05 // LED Output State 0-3
#define PCA9531_LS1_REG 0x06 // LED Output State 4-7

int main() {
    int file_i2c;
    char buffer[2];

    // Open the I2C device
    file_i2c = open("/dev/i2c-1", O_RDWR);
    if (file_i2c < 0) {
        std::cerr << "Failed to open I2C bus." << std::endl;
        return 1;
    }

    // Set the I2C slave address
    if (ioctl(file_i2c, I2C_SLAVE, PCA9531_ADDRESS) < 0) {
        std::cerr << "Failed to acquire I2C bus access and/or talk to slave." << std::endl;
        return 1;
    }

    // Example: Set PWM0 frequency (Prescaler 0)
    buffer[0] = PCA9531_PSC0_REG;
    buffer[1] = 0x05; // Example prescaler value
    if (write(file_i2c, buffer, 2) != 2) {
        std::cerr << "Failed to write to PSC0 register." << std::endl;
    }

    // Example: Set PWM0 duty cycle (PWM0 register)
    buffer[0] = PCA9531_PWM0_REG;
    buffer[1] = 0x80; // Example duty cycle (mid-brightness)
    if (write(file_i2c, buffer, 2) != 2) {
        std::cerr << "Failed to write to PWM0 register." << std::endl;
    }

    // Example: Set LED output 0 to use PWM0
    buffer[0] = PCA9531_LS0_REG;
    buffer[1] = 0x01; // Set LS0[1:0] to 01 (use PWM0 for LED0)
    if (write(file_i2c, buffer, 2) != 2) {
        std::cerr << "Failed to write to LS0 register." << std::endl;
    }

    // Close the I2C device
    close(file_i2c);

    return 0;
}
