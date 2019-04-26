#ifndef TCP_IP_H
#define TCP_IP_H

#include <iostream>
#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <string.h> 

class tcp_ip
{
    private:
    	bool establishConnection_();
        
        std::string serverIp_ = "";
        int port_ = 0;
        int CreateSocket_ = 0;
	   	
    public:
        tcp_ip();
        ~tcp_ip();
        void setSocket(std::string serverIp, int port);
        void sendData(std::string data);
        std::string readData();

        bool tcpSuccess = true;
        
};

#endif