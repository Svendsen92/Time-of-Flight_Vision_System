#include <iostream>
#include <memory>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>


#include <thread>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <chrono>  // for high_resolution_clock
#include <fstream>
#include <vector>

#include "config.h"
#include "imgProcess.h"
#include "geometricalData.h"
#include "imgConstruct.h"


struct thread_args {
  	int theta;
  	cv::Mat src, xyz;

  	// locks for the different threads
  	pthread_mutex_t img_mutex, theta_mutex, dimensions_mutex;
  	pthread_cond_t  img_cond, theta_cond, dimensions_cond;

  	cv::Point2f objCorners[4];
};


void* structInit(void* a){

	std::cerr << "structInit()" << std::endl;
	thread_args *b;
	b = (thread_args*)a;

	config c;

	// Asks if you want to reset the conveyor distance or change parameters in the config file
	c.changeParameters();

	try
	{
		b->theta = 0;

	  	b->img_mutex 		= PTHREAD_MUTEX_INITIALIZER;
	  	b->img_cond 		= PTHREAD_COND_INITIALIZER;
	  	b->theta_mutex 		= PTHREAD_MUTEX_INITIALIZER;
	  	b->theta_cond 		= PTHREAD_COND_INITIALIZER;
	  	b->dimensions_mutex = PTHREAD_MUTEX_INITIALIZER;
	  	b->dimensions_cond 	= PTHREAD_COND_INITIALIZER; 	

	}
	catch(const char* msg)
	{
		std::cerr << "ERROR... failed to initialize Struct variables!!" << std::endl;
		exit(-1);
	}
}


void* getImage(void* a){ 

	std::cerr << "getImage()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	config c;
	imgConstruct construct;

	auto cam = ifm3d::Camera::MakeShared();
	ifm3d::ImageBuffer::Ptr img = std::make_shared<ifm3d::ImageBuffer>();
	ifm3d::FrameGrabber::Ptr fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::IMG_AMP|ifm3d::IMG_CART|ifm3d::IMG_RDIS);
	
	cv::Mat xyz, conf;

	double coeff[2];
	coeff[0] = std::atof(c.getParameter("coeff_A").c_str());
	coeff[1] = std::atof(c.getParameter("coeff_B").c_str());

	double roi_width = std::atof(c.getParameter("roi_x").c_str());
	double roi_height = std::atof(c.getParameter("roi_y").c_str());
	double conf_dist = std::atof(c.getParameter("dist").c_str());

	while (true){

		//auto get_start = std::chrono::high_resolution_clock::now();

		if (! fg->WaitForFrame(img.get(), 1000))
		{
	      	std::cerr << "Timeout waiting for camera!" << std::endl;
	      	exit(-1);
	    }

		xyz = img->XYZImage();
		conf = img->ConfidenceImage();


		// The locks protects shared data.
    	pthread_mutex_lock(&b->img_mutex);

    	construct.generateImage(xyz, conf, coeff, conf_dist, roi_width, roi_height);

		b->xyz = cv::Mat(construct.xyz_rec.rows, construct.xyz_rec.cols, CV_16SC3);
		b->xyz = construct.xyz_rec;
		b->src = construct.cloud_rec;

	    pthread_mutex_unlock(&b->img_mutex);
	    pthread_cond_signal(&b->img_cond);

	    //auto get_finish = std::chrono::high_resolution_clock::now();
	    //printf("get image ms: %d\n", std::chrono::duration_cast<std::chrono::milliseconds>(get_finish - get_start).count());
	}
}


void* pointCloudMethod(void* a){

	std::cerr << "pointCloudMethod()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	imgProcess img;

	//int globalCounter = 0;

	while (true){

		pthread_cond_wait(&b->img_cond, &b->img_mutex);

		//auto process_start = std::chrono::high_resolution_clock::now();

		cv::Mat src = b->src;
		cv::Mat morph, thres;

		//medianBlur(src, src, 3);
		GaussianBlur(src, src, cv::Size(3,3), 0, 0, 1);
		//blur(src, src, cv::Size(3, 3), cv::Point(-1,-1));
		
		img.threshold(src, thres, 5); 
		cv::namedWindow("binary", cv::WINDOW_NORMAL);
		imshow("binary", thres);

		/*
		globalCounter++;
	    std::string str = std::to_string(globalCounter);
	    std::string binstr = str;
	    binstr += "_thres.png";
	    imwrite(binstr, thres);
		*/

	    img.morphing(thres, morph, 5, 3); 
		cv::namedWindow("morph", cv::WINDOW_NORMAL);
		imshow("morph", morph);

		/*
		std::string morphstr = str;
	    morphstr += "_morph.png";
	    imwrite(morphstr, morph);
		*/

	    cv::RotatedRect boundingBox = img.getControur(morph);

		// draw the rotated rect
		cv::Point2f corners[4];
		boundingBox.points(corners);

		img.drawRect(src, src, corners, boundingBox);

		cv::namedWindow("Base Image", cv::WINDOW_NORMAL);
		imshow("Base Image", src);
		cv::waitKey(1);

		/*
		std::string cloudstr = str;
	    cloudstr += "_cloud.png";
	    imwrite(cloudstr, src);

	    printf("globalCounter: %i\n", globalCounter);
	    char i;
	    std::cin >> i;
		*/

		// put in thread wait here
		pthread_mutex_lock(&b->theta_mutex);
		pthread_mutex_lock(&b->dimensions_mutex);

		b->objCorners[0] = corners[0];
		b->objCorners[1] = corners[1];
		b->objCorners[2] = corners[2];
		b->objCorners[3] = corners[3];


	    //b->theta = getRotateAngle(corners);
	    //std::cout << "theta: " << b->theta << std::endl;
		
	    pthread_mutex_unlock(&b->theta_mutex);
	    pthread_mutex_unlock(&b->dimensions_mutex);
	    pthread_cond_signal(&b->theta_cond);
	    pthread_cond_signal(&b->dimensions_cond);

	    //auto process_finish = std::chrono::high_resolution_clock::now();
	    //printf("image processing ms: %d\n", std::chrono::duration_cast<std::chrono::milliseconds>(process_finish - process_start).count());
	}
}


void* getRotateAngle(void* a) {

	std::cerr << "getRotateAngle()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	const float PI = 3.14159265359;

	while (true) {

		pthread_cond_wait(&b->theta_cond, &b->theta_mutex);

		auto process_start = std::chrono::high_resolution_clock::now();

		int tempMod = 0, tempHos = 0; 
		double hyp = 0, hos = 0, mod = 0, tempHyp = 0;

		for (size_t i = 0; i < 3; i++)
		{
			tempMod = int(b->objCorners[1 + i].y) - int(b->objCorners[0 + i].y);
			tempHos = int(b->objCorners[1 + i].x) - int(b->objCorners[0 + i].x);
			tempHyp = sqrt(pow(tempMod, 2) + pow(tempHos, 2));

			if (tempHyp > hyp) {
				hyp = tempHyp;
				hos = tempHos;
				mod = tempMod;
			}
		}

		b->theta = (atan(mod/hos) * 180 / PI);
		std::cout << "theta: " << b->theta << std::endl; 

		//auto process_finish = std::chrono::high_resolution_clock::now();
	    //printf("get rotation ms: %d\n", std::chrono::duration_cast<std::chrono::milliseconds>(process_finish - process_start).count());
	}
}


void* getDimensions(void* a) {

	std::cerr << "getDimensions()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	config c;
	geometricalData geoData;

	double conf_dist = std::atof(c.getParameter("dist").c_str());

	while (true)
	{

		pthread_cond_wait(&b->dimensions_cond, &b->dimensions_mutex);

		//auto dim_start = std::chrono::high_resolution_clock::now();


		double LWH[3];
		double* L_W_H = geoData.getDimensions(b->xyz, b->src, LWH, b->objCorners, conf_dist);

		//auto dim_finish = std::chrono::high_resolution_clock::now();
	    //printf("get dimensions ms: %d\n", std::chrono::duration_cast<std::chrono::milliseconds>(dim_finish - dim_start).count());


		std::cout << "\nobject length: " << std::setprecision(3) << L_W_H[0]/10 << "cm" << std::endl;
		std::cout << "object width: " << std::setprecision(3) << L_W_H[1]/10 << "cm" << std::endl;		
		std::cout << "object height: " << std::setprecision(3) << L_W_H[2]/10 << "cm" << std::endl;
	}
}





int main(int argc, const char **argv){

	thread_args *args = new thread_args;


  	// Initializes the variables in the Struct
  	structInit((void*) args);
	
  	// Starts the threads
  	std::thread thread1(getImage, args);
  	sleep(2);
	std::thread thread2(pointCloudMethod, args);
	sleep(2);
	std::thread thread3(getDimensions, args);
	sleep(2);
	std::thread thread4(getRotateAngle, args);
	sleep(2);

	// Holds the main thread until the thread has finished
	thread1.join();
  	thread2.join();
  	thread3.join();
  	thread4.join();

  	return 0;
}



/*
void setRoI(cv::Mat &src, cv::Mat &dst, double width_scaler, double hight_scaler){

	int width = int(src.cols*width_scaler);
	int hight = int(src.rows*hight_scaler);

	int x = int((src.cols-width)/2);
	int y = int((src.rows-hight)/2);

	cv::Rect RoI = cv::Rect(x, y, width, hight);
	dst = src(RoI);
}
*/

/*
void* getThetaAvg(void* a){

	std::cerr << "getThetaAvg()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	while (true){

		auto theta_start = std::chrono::high_resolution_clock::now();

		pthread_cond_wait(&b->theta_cond, &b->theta_mutex);

		//auto theta_start = std::chrono::high_resolution_clock::now();

		b->arr_theta[b->counter] = b->theta;

		b->counter++;
		//if (b->arr_theta[0] > -1 && b->arr_theta[1] > -1 && 
		//	b->arr_theta[2] > -1 && b->arr_theta[3] > -1 && b->arr_theta[4] > -1)
		if (true)
		{
			int arr_size = (sizeof(b->arr_theta)/sizeof(*b->arr_theta));

			if (b->counter == 5)
			{
				b->counter = 0;
			}

			int sum = 0;
			for(int i = 0; i < arr_size -1; i++)
			{
	   			sum += b->arr_theta[i];
			}
						
			int avg_theta = sum / (arr_size-1); 
			//printf("avg theta: %d\n", avg_theta);
		}
		else
		{
			b->counter = 0;
		}
		auto theta_finish = std::chrono::high_resolution_clock::now();
	    printf("theta processing ms: %d\n", std::chrono::duration_cast<std::chrono::milliseconds>(theta_finish - theta_start).count());	 
	}
}
*/

/*
int setCamParam(){

	char p;
	char *args[256], *argsSeparator = (char*)p, *paramCmd = (char*)p;

	argsSeparator = ", ";
	paramCmd = "";


	// Initializes all the elements of args to be NULL
	for (int i = 0; i < (sizeof(args)/sizeof(*args)-1); ++i)
	{
		args[i] = NULL;
	}

	// Set new parametes using json formate as see below. 
	// Look up the parameters' names in cmd-prompt by using "$imf3d dump", 
	// which will give you the entire list of settings.... Note some are read-only!! 
	args[0] = "echo \'{\"Apps\":[{\"Index\":\"1\",\"Imager\":{";

	// insert new parameters here... Remember to seperate the arguments using agrsSeparator:
	args[1] = "\"ExposureTime\":\"5000\"";	
	args[2] = argsSeparator;
	args[3] = "\"FrameRate\":\"20\"";	
	args[4] = argsSeparator;
	args[5] = "\"MinimumAmplitude\":\"42\"";
	args[6] = argsSeparator;
	args[7] = "\"Type\":\"under5m_low\"";

	// Always end with the below statement 
	args[8] = "}}]}\' | ifm3d config";


	// Finds the number of none-NULL elements in the array 
	int arrLen = 0;
	while (1){
		if (args[arrLen] == '\0'){
			break;
		}
		arrLen++;
	}

	
	// Concatenates the parameters into one const char pointer
	for (int i = 0; i < arrLen; ++i)
	{
		paramCmd = concatStr(paramCmd, args[i]);
	}


	//printf("paramCmd: %s\n", paramCmd);

	// Tries to execute a system command to change the parameters
	if (system(paramCmd)){
		std::cerr << "Failed to set camera parameters!!" << std::endl;
		return 1;
	}
	else
	{
		std::cerr << "Camera parameters were succesfully set!!" << std::endl;
		return 0;	
	}
}
*/

/*
char *concatStr(char *str1, char *str2){

   char *str3 = (char *) malloc(1 + strlen(str1)+ strlen(str2));
   strcpy(str3, str1);
   strcat(str3, str2);

   return str3;
}
*/

/*
std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}
*/