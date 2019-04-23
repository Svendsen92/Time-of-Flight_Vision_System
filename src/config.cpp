#include "config.h"

config::config(){}

config::~config(){}
		



double config::getMedian(std::vector<double> vec) {

	double median = 0;
	int vecLen = vec.size();
	std::sort(vec.begin(), vec.end());

	if (vecLen % 2 > 0)
	{
		median = (vec[(vecLen+1)/2] + vec[(vecLen-1)/2])/2;	
	}
	else
	{
  		median = vec[vecLen/2];
	}
	return median;
}

std::string config::getParameter(std::string param) {

	std::string str;
	std::ifstream myfile("config");
	if (myfile.is_open())
	{
		getline(myfile, str);
		myfile.close();
	}
	else
	{
		std::cout << "Unable to open file" << "\n";
		return NULL;
	}

	std::string str2 = param + ":";
	std::size_t idx = str.find(str2) + str2.length();
	if (idx != std::string::npos) {
		str2 = "";
		for (size_t i = idx; i < str.length(); i++)
		{
			if (str[i] == ',')
			{
				break;
			}
			else
			{
				str2 += str[i];
			}
		}
	}
	return str2;
}

int config::setParameter(std::string param, std::string newValue) {

	std::string str;
	std::ifstream myfile("config");
	if (myfile.is_open())
	{
		getline(myfile, str);
		myfile.close();
	}
	else
	{
		std::cout << "Unable to open file" << "\n";
		return 1;
	}

	std::string str2 = param + ":";
	std::size_t idx = str.find(str2) + str2.length();
	if (idx != std::string::npos) {
		bool first = true;
		std::string strValue = newValue;
		for (size_t i = idx; i < idx + str.length(); i++)
		{
			
			if (first)
			{
				first = false;
				int count = 0;
				for (i = idx; i < idx + strValue.length(); i++)
				{
					if (str[i] == ',')
					{
						break;
					}
					else
					{
						str[i] = strValue[count];
						count++;
					}
				}
			}

			if (str[i] == ',')
			{
				break;
			}
			else
			{
				str[i] = '0';
			}
		}

		std::ofstream myfile("config");
		if (myfile.is_open())
		{
			myfile << str;
			myfile.close();
		}
		else
		{
			std::cout << "Unable to open file" << "\n";
			return 1;
		}
	}
	return 0;
}

double* config::imgCoeff(double *coeffArr) {

	double a, b = 1, c = 0.5, d = 1, e = 0, f = 255;

	/* 
		we solve the linear systems using Cramer's Rule.
    	-  ax+by=e
    	-  cx+dy=f
    */
	a = coeffArr[0];

	double determinant = a*d - b*c;
    if(determinant != 0) {
        double x = (e*d - b*f)/determinant;
        double y = (a*f - e*c)/determinant;

        coeffArr[0] = x;
        coeffArr[1] = y;
        //printf("Cramer equations system: result, x = %f, y = %f\n", x, y);
    } 
    else 
    {
        printf("Cramer equations system: determinant is zero\n"
                "there are either no solutions or many solutions exist.\n"); 
    }
    return coeffArr;
}

void config::setBackgrundDist() { 

	std::cerr << "setBackgrundDist()" << std::endl;

	auto cam = ifm3d::Camera::MakeShared();
	ifm3d::ImageBuffer::Ptr img = std::make_shared<ifm3d::ImageBuffer>();
	ifm3d::FrameGrabber::Ptr fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::IMG_AMP|ifm3d::IMG_CART|ifm3d::IMG_RDIS);
	
	if (! fg->WaitForFrame(img.get(), 1000))
	{
      	std::cerr << "Timeout waiting for camera!" << std::endl;
      	exit(-1);
    }

	cv::Mat xyz = img->XYZImage(); 

	int width = 20;
	int height = 20;
	int x1 = int((xyz.cols - width)/2);
	int y1 = int((xyz.rows - height)/2);
	int widthEnd = x1 + width;
	int heightEnd = y1 + height;

  	std::vector<double> maxDistArr;
  	for (int x = x1; x < widthEnd; ++x)
  	{
  		for (int y = y1; y < heightEnd; ++y)
  		{
  			maxDistArr.push_back(xyz.at<cv::Vec3s>(cv::Point(x,y))[0]);
  		}
  	}

	double medianDist = getMedian(maxDistArr) - 5;  
  	std::cout << "medianDist: " << medianDist << std::endl;

    double coeffArr[2];
    coeffArr[0] = medianDist;
    double* coeff = imgCoeff(coeffArr);

    std::cout << "coeffArr[0]: " << coeffArr[0] << std::endl;
    std::cout << "coeffArr[1]: " << coeffArr[1] << std::endl;

    setParameter("dist", std::to_string(medianDist));
    setParameter("coeff_A", std::to_string(coeffArr[0]));
    setParameter("coeff_B", std::to_string(coeffArr[1]));
}

void config::selectionMenu() {

	std::string c = "";
	std::cout << "Choose the action to be taken.\n\n";
	std::cout << "Press 1) To recalibrate the camera distance to the conveyor.\n";
	std::cout << "Press 2) To change the region of interest.\n";
	std::cin >> c;


	if (c == "1")
	{
		setBackgrundDist();
	}
	else if (c == "2")
	{
		std::string roi_x = "";
		std::cout << "\ninput new RoI width: ";
		std::cin >> roi_x;
		setParameter("roi_x", roi_x);

		std::string roi_y = "";
		std::cout << "\ninput new RoI height: ";
		std::cin >> roi_y;
		setParameter("roi_y", roi_y);
	}
	else
	{
		std::cout << "\nAn invalid action was chosen...";
	}
}

void config::changeParameters(){

	std::cout << "Do you want to reset image parameters?? (yes/no): ";
	std::string c;
	std::cin >> c;
	if (c == "yes") {
		bool run = true;
		while (run) 
		{
			std::string c = "";

			selectionMenu();

			std::cout << "\nDo you wnat to change another parameter?? (yes/no): ";
			std::cin >> c;
			std::cout << "\n";
			if (c != "yes") 
			{
				run = false;
			} 
		}
	}
}