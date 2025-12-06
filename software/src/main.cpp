#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <thread>
#include <iostream>
#include "ceSerial.h"
#include "httplib.h"
#include <chrono>
#include <ctime>
#include <iomanip> // For std::hex, std::setfill, std::setw
#include <sstream> // For std::ostringstream

ceSerial com("/dev/ttyACM0", 115200, 8, 'N', 1);
httplib::Server svr;

long long getTimestampMilliseconds() {
    auto now = std::chrono::system_clock::now();
    auto duration_since_epoch = now.time_since_epoch();
    auto milliseconds_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(duration_since_epoch).count();
    return milliseconds_since_epoch;  // Return positive value
}

std::string charToHexString(char c) {
    std::ostringstream oss;
    oss << std::hex << std::uppercase << std::setfill('0') << std::setw(2) 
        << static_cast<int>(static_cast<unsigned char>(c));
    return oss.str();
}

std::string readSerialData() {
    bool readSuccess = true;
    char initialChar = com.ReadChar(readSuccess);
    
    if (!readSuccess) {
        return std::to_string(getTimestampMilliseconds()) + ": Error reading from serial port";
    }
    
    // Implement timeout to avoid infinite recursion
    int retries = 0;
    while (initialChar != 'B' && retries < 100) {
        std::cerr << std::to_string(getTimestampMilliseconds()) << ": Invalid start character: " << charToHexString(initialChar) << std::endl;
        initialChar = com.ReadChar(readSuccess);
        if (!readSuccess) {
            return std::to_string(getTimestampMilliseconds()) + ": Error reading from serial port";
        }
        retries++;
    }
    
    if (initialChar != 'B') {
        return std::to_string(getTimestampMilliseconds()) + ": Failed to find valid start character after retries";
    }

    std::vector<char> buffer(800);
    int i = 0;

    while (true) {
        char nextChar = com.ReadChar(readSuccess);
        if (!readSuccess) {
            return std::to_string(getTimestampMilliseconds()) + ": Error reading from serial port";
        }
        if (nextChar == 'E') {
            break;
        }
        if (i < buffer.size()) {
            buffer[i++] = nextChar;
        } else {
            break;
        }
    }

    return std::string(buffer.data(), i);
}

std::vector<char> readSerialDataBuffer() {
    bool readSuccess = true;
    char initialChar = com.ReadChar(readSuccess);
    
    if (!readSuccess) {
        std::cerr << std::to_string(getTimestampMilliseconds()) << ": Error reading from serial port" << std::endl;
        return std::vector<char>();
    }
    
    // Implement timeout to avoid infinite recursion
    int retries = 0;
    while (initialChar != 'B' && retries < 100) {
        std::cerr << std::to_string(getTimestampMilliseconds()) << ": Invalid start character: " << charToHexString(initialChar) << std::endl;
        initialChar = com.ReadChar(readSuccess);
        if (!readSuccess) {
            std::cerr << std::to_string(getTimestampMilliseconds()) << ": Error reading from serial port" << std::endl;
            return std::vector<char>();
        }
        retries++;
    }
    
    if (initialChar != 'B') {
        std::cerr << std::to_string(getTimestampMilliseconds()) << ": Failed to find valid start character after retries" << std::endl;
        return std::vector<char>();
    }

    std::vector<char> buffer(800);
    int i = 0;

    while (true) {
        char nextChar = com.ReadChar(readSuccess);
        if (!readSuccess) {
            std::cerr << std::to_string(getTimestampMilliseconds()) << ": Error reading from serial port" << std::endl;
            return std::vector<char>();
        }
        if (nextChar == 'E') {
            break;
        }
        if (i < buffer.size()) {
            buffer[i++] = nextChar;
        } else {
            break;
        }
    }

    return std::vector<char>(buffer.begin(), buffer.begin() + i);
}

std::string parseMessage(std::vector<char> data) {
    // if (data.empty() || data[0] != 'B') {
    //     std::cout << std::to_string(getTimestampMilliseconds()) + ": " + data[0] << std::endl;
    //     return std::to_string(getTimestampMilliseconds()) + ": Invalid start character";
    // }

    size_t pos = 0;  // 'B' gets removed before parsing
    std::string fullMessage;
    
    while (pos < data.size()) {
        if (data[pos] == 'E') break;
        
        // Check if we have enough bytes for a float
        if (pos + sizeof(float) > data.size()) break;
        
        if (data[pos] == '|') {
            fullMessage += "|";
            pos++;
            continue;
        }

        // Extract 8-byte double
        double value;
        std::memcpy(&value, &data[pos], sizeof(double));
        std::cout << std::to_string(getTimestampMilliseconds()) << ": Parsed double: " << value << std::endl;
        fullMessage += std::to_string(value);
        pos += sizeof(double);
    }
    
    return fullMessage;
}

int main() {
    printf("Opening port %s.\n", com.GetPort().c_str());
    if (com.Open() == 0) {
        std::cout << std::to_string(getTimestampMilliseconds()) << ": Serial comms with arduino ok.\n";
    } else {
        std::cout << std::to_string(getTimestampMilliseconds()) << ": Serial comms with arduino not ok.\n";
        return 1;
    }

    svr.Get("/", [](const httplib::Request &, httplib::Response &res) {
        res.set_file_content("index.html", "text/html");
    });

    svr.Get("/get_raw_serial_data", [](const httplib::Request &, httplib::Response &res) {
        res.set_content(readSerialData(), "text/plain");
    });

    svr.Get("/get_parsed_serial_data", [](const httplib::Request &, httplib::Response &res) {
        res.set_content(parseMessage(readSerialDataBuffer()), "text/plain");
    });

    svr.listen("0.0.0.0", 8008);

    return 0;
}