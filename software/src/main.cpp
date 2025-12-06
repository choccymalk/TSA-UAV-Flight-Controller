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

ceSerial com("/dev/ttyS0",115200,8,'N',1);
httplib::Server svr;

std::string readSerialData(){
    char initialChar = com.ReadChar(true);
    char nextChar;
    char charsFromMessage = new char[800];
    //std::string message;
    int i = 0;
    if(initialChar == "B"){
        while(true){
            nextChar = com.ReadChar(true);
            if(nextChar == "E"){
                break;
            } else {
                //message.append(nextChar);
                charsFromMessage[i] = nextChar;
                i++;
            }
        }
    } else {
        readSerialData();
    }
    return str(charsFromMessage[0], charsFromMessage[800]);
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
        com.ReadChar(true);
        res.set_content("Hello World!", "text/plain");
    });

    svr.listen("0.0.0.0", 8008);

}
