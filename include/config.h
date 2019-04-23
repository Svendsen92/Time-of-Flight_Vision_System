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
	    double getMedian(std::vector<double> vec);
	    int setParameter(std::string param, std::string newValue);
	    double* imgCoeff(double *coeffArr);
	    void setBackgrundDist();
	    void selectionMenu();


    public:
        config();
        ~config();
        void changeParameters();
        std::string getParameter(std::string param);

};

#endif