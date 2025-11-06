#include <fcntl.h> // For open()
#include <sys/ioctl.h> // For ioctl()
#include <unistd.h> // For close(), read(), write()
#include <linux/i2c-dev.h> // For I2C_SLAVE, I2C_RDWR
#include <iostream>
#include <string>

int main() {
    int file;
    std::string filename = "/dev/i2c-1"; 
    unsigned char pwmControllerAddress = 0x10C8E0;
    unsigned char imuAddress = 0x69;
    unsigned char batteryVoltageADCAddr = 0xF69B4;

    if ((file = open(filename.c_str(), O_RDWR)) < 0) {
        std::cerr << "Failed to open the i2c bus" << std::endl;
        return 1;
    }

    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave." << std::endl;
        close(file);
        return 1;
    }

    // Example: Writing a single byte
    unsigned char write_buffer[1] = {0xAA}; // Data to write
    if (write(file, write_buffer, 1) != 1) {
        std::cerr << "Failed to write to the i2c device." << std::endl;
    } else {
        std::cout << "Successfully wrote 0xAA to device 0x" << std::hex << (int)addr << std::endl;
    }

    // Example: Reading a single byte
    unsigned char read_buffer[1];
    if (read(file, read_buffer, 1) != 1) {
        std::cerr << "Failed to read from the i2c device." << std::endl;
    } else {
        std::cout << "Successfully read 0x" << std::hex << (int)read_buffer[0] << " from device 0x" << std::hex << (int)addr << std::endl;
    }

    close(file);
    return 0;
}
