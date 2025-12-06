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
#include <string.h>
#include "ceSerial.h"
#include "httplib.h"
#include <chrono>
#include <ctime>

ceSerial com("/dev/ttyACM0",115200,8,'N',1);
httplib::Server svr;

int getTimestampSeconds() {
    return static_cast<int>(std::time(nullptr));
}

std::string readSerialData() {
    bool readSuccess = true; // assume serial will read successfully, com.readChar will update this if it wasn't
    char initialChar = com.ReadChar(readSuccess);
    if(!readSuccess) {
        return std::to_string(getTimestampSeconds()) + ": Error reading from serial port";
    }
    if (initialChar != 'B') {
        std::cout << std::to_string(getTimestampSeconds()) << ": Invalid start character: %c\n", initialChar;
        // retry, eventually we will get a message
        // TODO: implement a timeout here to avoid infinite loops
        readSerialData();
    }

    std::vector<char> buffer(800);  // Use vector for safe memory management
    int i = 0;

    while (true) {
        char nextChar = com.ReadChar(readSuccess);
        if(!readSuccess) {
            return std::to_string(getTimestampSeconds()) + ": Error reading from serial port";
        }
        if (nextChar == 'E') {
            break;  // End of message
        }
        if (i < buffer.size()) {
            buffer[i++] = nextChar;
        } else {
            break;  // Buffer full
        }
    }

    // Convert buffer to string
    return std::string(buffer.data(), i);
}

std::vector<char> readSerialDataBuffer(){
    bool readSuccess = true; // assume serial will read successfully, com.readChar will update this if it wasn't
    char initialChar = com.ReadChar(readSuccess);
    if(!readSuccess) {
        std::cout << std::to_string(getTimestampSeconds()) << ": Error reading from serial port";
        return std::vector<char>();
    }
    if (initialChar != 'B') {
        std::cout << std::to_string(getTimestampSeconds()) << ": Invalid start character: %c\n", initialChar;
        // retry, eventually we will get a message
        // TODO: implement a timeout here to avoid infinite loops
        readSerialData();
    }

    std::vector<char> buffer(800);  // Use vector for safe memory management
    int i = 0;

    while (true) {
        char nextChar = com.ReadChar(readSuccess);
        if(!readSuccess) {
            std::cout << std::to_string(getTimestampSeconds()) << ": Error reading from serial port";
            return std::vector<char>();
        }
        if (nextChar == 'E') {
            break;  // End of message
        }
        if (i < buffer.size()) {
            buffer[i++] = nextChar;
        } else {
            break;  // Buffer full
        }
    }

    // just return the buffer
    return std::vector<char>(buffer.begin(), buffer.begin() + i);
}

std::string parseMessage(std::vector<char> data) {
    size_t pos = 0;
    size_t sizeOfCurrentBlock = 0;  // iterator for current block size, each block is 8 bytes
    std::string fullMessage;
    if (data[0] != 'B') return std::to_string(getTimestampSeconds()) + ": Invalid start character";

    pos = 1;  // Skip 'B'
    
    while (pos < data.size()) {
        if (data[pos] == 'E') break;
        sizeOfCurrentBlock++;
        if (data[pos] == '|' && sizeOfCurrentBlock == 7) {
            pos++;  // Skip '|'
            sizeOfCurrentBlock = 0;
            continue;
        }

        // Extract 8-byte float (assuming 4 bytes for float, 4 padding)
        float value;
        std::memcpy(&value, data.at(pos), sizeof(float));
        std::cout << std::to_string(getTimestampSeconds()) << ": Parsed float: " << value << std::endl;
        fullMessage += std::to_string(value) + (data[pos + 8] == '|' ? "|" : "");
        pos += 8;  // Move to next 8-byte block
    }
    return fullMessage;
}

int main(){

    printf("Opening port %s.\n",com.GetPort().c_str());
	if (com.Open() == 0) {
		std::cout << std::to_string(getTimestampSeconds()) << ": Serial comms with arduino ok.\n";
	}
	else {
		std::cout << std::to_string(getTimestampSeconds()) << ": Serial comms with arduino not ok.\n";
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

}
