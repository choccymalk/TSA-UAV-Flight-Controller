cmake_minimum_required(VERSION 3.10)
project(quadcopter VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
#set(CMAKE_CXX_FLAGS "-fpermissive")

# Define a list of source files
set(SOURCES
    src/main.cpp
)

# Define a list of header files (optional for basic compilation, but good for IDEs)
set(HEADERS
    include/ceSerial.h
    include/httplib.h
    include/mjpeg_streamer.hpp
)


find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)

include_directories(${OpenCV_INCLUDE_DIRS})

# Add an executable target
add_executable(quadcopter ${SOURCES} ${HEADERS})

# Specify include directories for headers
target_include_directories(quadcopter PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    #/usr/include/opencv4
)

target_link_libraries(quadcopter ${OpenCV_LIBS})

# (Optional) Install the executable
install(TARGETS quadcopter DESTINATION bin)