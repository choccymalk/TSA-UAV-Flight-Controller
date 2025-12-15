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
#include "mjpeg_streamer.hpp"
#include "opencv2/opencv.hpp" 
// WebSocket++ headers
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

typedef websocketpp::server<websocketpp::config::asio> ws_server;
typedef websocketpp::connection_hdl connection_hdl;

using MJPEGStreamer = nadjieb::MJPEGStreamer;
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
    
    std::vector<char> buffer(1024);
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
        std::cout << std::to_string(getTimestampMilliseconds()) << ": Telemetry request received, waiting 150 ms for response..." << std::endl;
        
        {
            std::lock_guard<std::mutex> lock(g_serial_mutex);
            // Send request to flight controller
            com.WriteChar('.');
            com.WriteChar('s');
            // termination character, required for response
            com.WriteChar(';');
        }
        
        // Flight controller takes ~150 ms to respond
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        
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

// draws a horizon line on the video feed based on roll and pitch, like a heads up display in a plane
cv::Mat drawHorizonOnFeed(cv::Mat frame, float roll, float pitch) {
    int width = frame.cols;
    int height = frame.rows;
    cv::Point center(width / 2, height / 2);
    
    // Debug: Mark center point
    cv::circle(frame, center, 5, cv::Scalar(255, 0, 0), -1);
    
    float horizonY = center.y + (pitch / 90.0f) * (height / 2);
    float angle = -roll * CV_PI / 180.0f;
    
    // Check for extreme angles (avoid tan(90°))
    if (abs(roll) >= 89.0f) {
        roll = (roll > 0) ? 88.0f : -88.0f;
        angle = -roll * CV_PI / 180.0f;
    }
    
    float tanAngle = std::tan(angle);
    cv::Point pt1(0, static_cast<int>(horizonY - (width * tanAngle / 2)));
    cv::Point pt2(width, static_cast<int>(horizonY + (width * tanAngle / 2)));
    
    // Debug: Mark the calculated points
    cv::circle(frame, pt1, 5, cv::Scalar(0, 0, 255), -1);
    cv::circle(frame, pt2, 5, cv::Scalar(0, 0, 255), -1);
    
    // Check if points are within frame bounds
    cv::Rect frameRect(0, 0, width, height);
    if (frameRect.contains(pt1) && frameRect.contains(pt2)) {
        cv::line(frame, pt1, pt2, cv::Scalar(0, 255, 0), 2);
    } else {
        // Use cv::clipLine to ensure line is visible
        cv::Point clippedPt1 = pt1;
        cv::Point clippedPt2 = pt2;
        if (cv::clipLine(frameRect, clippedPt1, clippedPt2)) {
            cv::line(frame, clippedPt1, clippedPt2, cv::Scalar(0, 255, 0), 2);
        }
    }
    
    // Add text for debugging
    //std::string debugText = "Roll: " + std::to_string(roll) + 
    //                       " Pitch: " + std::to_string(pitch);
    //cv::putText(frame, debugText, cv::Point(10, 30), 
    //            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    
    return frame;
}

std::array<double, 2> getRollPitchFromSerialData(const std::vector<char>& data) {
    double roll = 0.0;
    double pitch = 0.0;
    size_t pos = 0;
    int floatCount = 0;

    // Debug: Show what we're receiving
    //std::cout << "getRollPitchFromSerialData: data size = " << data.size() << std::endl;
    
    while (pos < data.size()) {
        // Check for end marker first (matching parseMessage)
        if (data[pos] == 'E') {
            //std::cout << "Found end marker 'E' at position " << pos << std::endl;
            break;
        }
        
        // Check if we have enough bytes for a float
        if (pos + sizeof(float) > data.size()) {
            //std::cout << "Not enough bytes for a float at position " << pos << std::endl;
            break;
        }
        
        if (data[pos] == '|') {
            // Skip the delimiter
            pos++;
            continue;
        }
        
        // Parse the float
        float value;
        std::memcpy(&value, &data[pos], sizeof(float));
        
        //std::cout << "Parsed float " << floatCount << ": " << value << std::endl;
        
        // According to your comment: 
        // 0: throttle, 1: yaw, 2: pitch, 3: roll
        if (floatCount == 2) {
            pitch = static_cast<double>(value);
        } else if (floatCount == 3) {
            roll = static_cast<double>(value);
            break; // We got what we need
        }
        
        floatCount++;
        pos += sizeof(float);
    }
    
    //std::cout << "Returning roll=" << roll << ", pitch=" << pitch << std::endl;
    return {roll, pitch};
}

void mjpegStreamThread(MJPEGStreamer& streamer) {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "VideoCapture not opened\n";
    }

    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 50};

    while (streamer.isRunning()) {
        cv::Mat frame;
        cap >> frame;
        com.WriteChar('.');
        com.WriteChar('s');
        // termination character, required for response
        com.WriteChar(';');
        std::array<double, 2> rollPitch = getRollPitchFromSerialData(readSerialDataBuffer());
        //std::cout << "Roll: " << rollPitch[0] << ", Pitch: " << rollPitch[1] << std::endl;
        frame = drawHorizonOnFeed(frame, rollPitch[0], rollPitch[1]);
        if (frame.empty()) {
            std::cerr << "frame not grabbed\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        std::vector<uchar> buff_bgr;
        cv::imencode(".jpg", frame, buff_bgr, params);
        streamer.publish("/stream", std::string(buff_bgr.begin(), buff_bgr.end()));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

/*std::string getWifiStrength(){
    std::string strength;
    std::ifstream wireless_file("/proc/net/wireless");
    if (!wireless_file.is_open()) {
        return "Error";
    }

    std::string line;
    // Skip header lines
    std::getline(wireless_file, line);
    std::getline(wireless_file, line);

    while (std::getline(wireless_file, line)) {
        // Parse the line to extract signal quality (and other info if needed)
        // Example: wlan0: 0000 65. - 45. - 256 0 0 0 8 169 0
        // The third column (65 in this example) is typically the link quality.
        // You would need to parse this string to extract the relevant number.
        std::cout << std::to_string(getTimestampMilliseconds()) << ": Wireless info: " << line << std::endl;
        return line;
        // Implement parsing logic here to extract signal strength
    }
    wireless_file.close();
    return "Error";
}*/

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

    // Start MJPEG streamer
    MJPEGStreamer streamer;
    streamer.start(8010);

    // Start MJPEG streaming in separate thread
    std::thread mjpegThread([&streamer](){ mjpegStreamThread(std::ref(streamer)); });
    mjpegThread.detach();

    // HTTP server for serving static files
    svr.Get("/", [](const httplib::Request &, httplib::Response &res) {
        res.set_file_content("index.html", "text/html");
    });

    svr.Get("/script.js", [](const httplib::Request &, httplib::Response &res) {
        res.set_file_content("script.js", "text/javascript");
    });

    svr.Get("/get_wifi_strength", [](const httplib::Request &, httplib::Response &res) {
        res.set_file_content("getWifiStrength()", "text/plain");
    });
    
    svr.listen("0.0.0.0", 8008);

    return 0;
}