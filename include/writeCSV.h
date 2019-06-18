#ifndef WRITECSV_H
#define WRITECSV_H

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

class writeCSV
{
	private:
		std::string getTimeStamp_();
		std::string name_ = "";

    public:
        writeCSV(std::string name);
        ~writeCSV();
        int write(std::string input);

};

#endif