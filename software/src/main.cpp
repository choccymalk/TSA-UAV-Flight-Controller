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
#include <set>
#include <memory>
#include "ceSerial.h"
#include "httplib.h"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

// WebSocket++ headers
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

typedef websocketpp::server<websocketpp::config::asio> ws_server;
typedef websocketpp::connection_hdl connection_hdl;

ceSerial com("/dev/ttyACM0", 115200, 8, 'N', 1);
httplib::Server svr;
ws_server wsServer;
std::mutex g_serial_mutex;

long long getTimestampMilliseconds() {
    auto now = std::chrono::system_clock::now();
    auto duration_since_epoch = now.time_since_epoch();
    auto milliseconds_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(duration_since_epoch).count();
    return milliseconds_since_epoch;
}

std::string charToHexString(char c) {
    std::ostringstream oss;
    oss << std::hex << std::uppercase << std::setfill('0') << std::setw(2) 
        << static_cast<int>(static_cast<unsigned char>(c));
    return oss.str();
}

std::vector<char> readSerialDataBuffer() {
    std::lock_guard<std::mutex> lock(g_serial_mutex);
    bool readSuccess = true;
    com.WriteChar('.');
    com.WriteChar('s');
    char initialChar = com.ReadChar(readSuccess);
    
    if (!readSuccess) {
        std::cerr << std::to_string(getTimestampMilliseconds()) << ": Error reading from serial port" << std::endl;
        return std::vector<char>();
    }
    
    int retries = 0;
    while (initialChar != 'B' && retries < 100) {
        std::cerr << std::to_string(getTimestampMilliseconds()) << ": Invalid start character: " << charToHexString(initialChar) << std::endl;
        com.WriteChar('.');
        com.WriteChar('s');
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
        com.WriteChar('.');
        com.WriteChar('s');
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
    size_t pos = 0;
    std::string fullMessage;
    
    while (pos < data.size()) {
        if (data[pos] == 'E') break;
        
        if (pos + sizeof(float) > data.size()) break;
        
        if (data[pos] == '|') {
            fullMessage += "|";
            pos++;
            continue;
        }
        
        float value;
        std::memcpy(&value, &data[pos], sizeof(float));
        std::cout << std::to_string(getTimestampMilliseconds()) << ": Parsed float: " << value << std::endl;
        fullMessage += std::to_string(value);
        pos += sizeof(float);
    }
    
    return fullMessage;
}

void onWSOpen(connection_hdl hdl) {
    std::cout << std::to_string(getTimestampMilliseconds()) << ": WebSocket client connected" << std::endl;
}

void onWSClose(connection_hdl hdl) {
    std::cout << std::to_string(getTimestampMilliseconds()) << ": WebSocket client disconnected" << std::endl;
}

void onWSMessage(connection_hdl hdl, ws_server::message_ptr msg) {
    std::string payload = msg->get_payload();
    
    if (payload == "get_data") {
        // Client requests serial data
        std::cout << std::to_string(getTimestampMilliseconds()) << ": Telemetry request received, waiting 1.5 seconds for response..." << std::endl;
        
        {
            std::lock_guard<std::mutex> lock(g_serial_mutex);
            // Send request to flight controller
            com.WriteChar('.');
            com.WriteChar('s');
        }
        
        // Flight controller takes ~1.5 seconds to respond
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        
        std::vector<char> rawData = readSerialDataBuffer();
        if (!rawData.empty()) {
            std::string parsedData = parseMessage(rawData);
            if (!parsedData.empty()) {
                try {
                    wsServer.send(hdl, parsedData, websocketpp::frame::opcode::text);
                    std::cout << std::to_string(getTimestampMilliseconds()) << ": Telemetry sent to client" << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "Error sending data: " << e.what() << std::endl;
                }
            }
        }
    } else if (payload.find("send:") == 0) {
        // Client sends command to serial device
        std::string dataToSend = payload.substr(5);
        {
            std::lock_guard<std::mutex> lock(g_serial_mutex);
            for (char c : dataToSend) {
                com.WriteChar(c);
            }
        }
        // Small delay to let flight controller process command
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        std::cout << std::to_string(getTimestampMilliseconds()) << ": Sent data via WebSocket: " << dataToSend << std::endl;
    }
}

int main() {
    printf("Opening port %s.\n", com.GetPort().c_str());
    if (com.Open() == 0) {
        std::cout << std::to_string(getTimestampMilliseconds()) << ": Serial comms with arduino ok.\n";
    } else {
        std::cout << std::to_string(getTimestampMilliseconds()) << ": Serial comms with arduino not ok.\n";
        return 1;
    }
    
    // Configure WebSocket server
    try {
        wsServer.set_access_channels(websocketpp::log::alevel::all);
        wsServer.clear_access_channels(websocketpp::log::alevel::frame_payload);
        
        wsServer.init_asio();
        wsServer.set_open_handler(&onWSOpen);
        wsServer.set_close_handler(&onWSClose);
        wsServer.set_message_handler(&onWSMessage);
        
        wsServer.listen(8009);
        wsServer.start_accept();
    } catch (websocketpp::exception const & e) {
        std::cout << e.what() << std::endl;
        return 1;
    }
    
    // Start WebSocket server in separate thread
    std::thread wsThread([](){ wsServer.run(); });
    wsThread.detach();
    
    // HTTP server for serving static files
    svr.Get("/", [](const httplib::Request &, httplib::Response &res) {
        res.set_file_content("index.html", "text/html");
    });
    
    svr.listen("0.0.0.0", 8008);
    return 0;
}