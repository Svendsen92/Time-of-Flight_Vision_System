#include "writeCSV.h"

writeCSV::writeCSV(std::string name){
	name_ = name;
}

writeCSV::~writeCSV(){}


int writeCSV::write(std::string input) {

	std::ofstream myfile(name_, std::ios::app);

	if (myfile.is_open())
	{
	    myfile << input;

	    myfile.close();
	    return 0;
	}
	else
	{
		std::cout << "Unable to open file" << std::endl;
		return 1;
	}
}