#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <fstream>

#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>


class config
{
    private:
	    double getMedian_(std::vector<double> vec); // finds the median value of a vector
	    int setParameter_(std::string param, std::string newValue); // writes a new paramater value into the config file 
	    double* imgCoeff_(double *coeffArr); // calculates coefficients for a linear equation
	    void setBackgrundDist_(); // finds the distance to the background scene 
	    void selectionMenu_(); // gives the user input chooses to change different things


    public:
        config();
        ~config();
        void changeParameters(); // this calls all the functions needed to change a parameter
        std::string getParameter(std::string param); // retrevies a parameter from the config file

};

#endif