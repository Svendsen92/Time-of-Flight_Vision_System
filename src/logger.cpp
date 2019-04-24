#include "logger.h"

logger::logger(std::string name){

	fileName_ = name;

	std::ofstream myfile(fileName_, std::ios::trunc);

	if (myfile.is_open())
	{
	    myfile << "";

	    myfile.close();
	}
	else
	{
		std::cout << "Unable to open file" << std::endl;
	}

}

logger::~logger(){}

std::string logger::getTimeStamp_() {
	// current date/time based on current system
   std::time_t now = time(0);
   
   // convert now to string form
   char* dt = ctime(&now);

   std::cout << "The local date and time is: " << dt << std::endl;
   return dt;
}

int logger::write(char state, std::string input) {

	std::ofstream myfile("log-file", std::ios::app);

	std::string msg = "";
	if (state == 'm')
	{
		msg = "Message: ";
	}
	else if (state == 'w')
	{
		msg = "Warning: ";
	}
	else if (state == 'e')
	{
		msg = "Error: ";
	}
	else
	{
		std::cout << "Invalid log type" << std::endl;
		return 1;
	}

	std::string timeStamp = ". Timestamp: " + getTimeStamp_();

	if (myfile.is_open())
	{
	    myfile << msg + input + timeStamp + "\n";

	    myfile.close();
	    return 0;
	}
	else
	{
		std::cout << "Unable to open file" << std::endl;
		return 1;
	}
}