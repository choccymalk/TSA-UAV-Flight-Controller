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
std::set<connection_hdl, std::owner_less<connection_hdl>> g_connections;
std::mutex g_connections_mutex;

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

void broadcastData(const std::string& message) {
    std::lock_guard<std::mutex> lock(g_connections_mutex);
    for (auto hdl : g_connections) {
        try {
            wsServer.send(hdl, message, websocketpp::frame::opcode::text);
        } catch (const std::exception& e) {
            std::cerr << "Error sending message: " << e.what() << std::endl;
        }
    }
}

void serialDataThread() {
    int i = 0;
    while (true) {
        if(i == 100){
            try {
                i = 0;
                std::vector<char> rawData = readSerialDataBuffer();
                if (!rawData.empty()) {
                    std::string parsedData = parseMessage(rawData);
                    if (!parsedData.empty()) {
                        broadcastData(parsedData);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } catch (const std::exception& e) {
                std::cerr << "Error in serialDataThread: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        } else {
            i++;
        }
    }
}

void onWSOpen(connection_hdl hdl) {
    std::lock_guard<std::mutex> lock(g_connections_mutex);
    g_connections.insert(hdl);
    std::cout << std::to_string(getTimestampMilliseconds()) << ": WebSocket opened, total connections: " << g_connections.size() << std::endl;
}

void onWSClose(connection_hdl hdl) {
    std::lock_guard<std::mutex> lock(g_connections_mutex);
    g_connections.erase(hdl);
    std::cout << std::to_string(getTimestampMilliseconds()) << ": WebSocket closed, total connections: " << g_connections.size() << std::endl;
}

void onWSMessage(connection_hdl hdl, ws_server::message_ptr msg) {
    std::string payload = msg->get_payload();
    
    if (payload.find("send:") == 0) {
        std::string dataToSend = payload.substr(5);
        for (char c : dataToSend) {
            com.WriteChar(c);
        }
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
    
    // Start serial data reading thread
    std::thread serialThread(serialDataThread);
    serialThread.detach();
    
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