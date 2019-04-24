#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

class logger
{
	private:
		std::string getTimeStamp_();
		std::string fileName_ = "";

    public:
        logger(std::string name);
        ~logger();
        int write(char state, std::string input);

};

#endif