#include "tcp_ip.h"

tcp_ip::tcp_ip(){}

tcp_ip::~tcp_ip(){}

void tcp_ip::setSocket(std::string serverIp, int port) {

        serverIp_ = serverIp; 
        port_ = port;
}

bool tcp_ip::establishConnection_() {

	// Creating socket file descriptor 
    std::cout << "client" << std::endl;
    CreateSocket_ = 0; 
    int n = 0;
    char dataReceived[1024];
    struct sockaddr_in ipOfServer;
 
    memset(dataReceived, '0' ,sizeof(dataReceived));
     
    if((CreateSocket_ = socket(AF_INET, SOCK_STREAM, 0))< 0)
    {
        printf("Socket not created \n");
        return false;
    }
     
    ipOfServer.sin_family = AF_INET;
    ipOfServer.sin_port = htons(port_);
    ipOfServer.sin_addr.s_addr = inet_addr(serverIp_.c_str());

    if (connect(CreateSocket_, (struct sockaddr *)&ipOfServer, sizeof(ipOfServer))<0)
    {
        printf("Connection failed due to port and ip problems\n");
        return false;
    }

    return true;  
}

void tcp_ip::sendData(std::string data) {

	int errorCheck = establishConnection_();

	if (errorCheck < 0)
	{
		std::cout << "Error could not establish connection" << std::endl;
		tcpSuccess = false;
	}
	else
	{
		errorCheck = write(CreateSocket_, data.c_str(), strlen(data.c_str()));

		if (errorCheck < 0) {
			std::cout << "Error occured in sendData()" << std::endl;
			tcpSuccess = false;
		}
		else
		{
			tcpSuccess = true;
		}
	}
}

std::string tcp_ip::readData() {

	int errorCheck = establishConnection_();

	if (errorCheck < 0)
	{
		std::cout << "Error could not establish connection" << std::endl;
		tcpSuccess = false;
		return "";
	}
	else
	{
		char buffer[1024] = {0};
		errorCheck = read(CreateSocket_ , buffer, 1024);

		if (errorCheck < 0) {
			std::cout << "Error occured in readData()" << std::endl;
			tcpSuccess = false;
			return "";
		}
		else
		{
			tcpSuccess = true;
			return buffer;
		}
	}
}

