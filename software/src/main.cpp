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

ceSerial com("/dev/ttyACM0",115200,8,'N',1);
httplib::Server svr;

std::string readSerialData() {
    bool readSuccess = true; // assume serial will read successfully, com.readChar will update this if it wasn't
    char initialChar = com.ReadChar(readSuccess);
    if(!readSuccess) {
        return "Error reading from serial port";
    }
    if (initialChar != 'B') {
        return "Not a valid message";  // Not a valid message
    }

    std::vector<char> buffer(800);  // Use vector for safe memory management
    int i = 0;

    while (true) {
        char nextChar = com.ReadChar(readSuccess);
        if(!readSuccess) {
            return "Error reading from serial port";
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

std::string parseMessage(const std::string& data) {
    size_t pos = 0;
    size_t sizeOfCurrentBlock = 0;  // iterator for current block size, each block is 8 bytes
    if (data[0] != 'B') return "Invalid start character";

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
        std::memcpy(&value, data.c_str() + pos, sizeof(float));
        std::cout << "Parsed float: " << value << std::endl;

        pos += 8;  // Move to next 8-byte block
    }
}

int main(){

    printf("Opening port %s.\n",com.GetPort().c_str());
	if (com.Open() == 0) {
		printf("Serial comms with arduino ok.\n");
	}
	else {
		printf("Serial comms with arduino not ok.\n");
		return 1;
	}

    svr.Get("/", [](const httplib::Request &, httplib::Response &res) {
        res.set_file_content("index.html", "text/html");
    });

    svr.Get("/get_serial_data", [](const httplib::Request &, httplib::Response &res) {
        res.set_content(readSerialData(), "text/plain");
    });

    svr.listen("0.0.0.0", 8008);

}
