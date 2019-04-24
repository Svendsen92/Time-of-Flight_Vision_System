#include <iostream>
#include <memory>
#include <thread>
#include <signal.h>
#include <chrono>  // for high_resolution_clock

#include <opencv2/opencv.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>

#include "config.h"
#include "imgProcess.h"
#include "geometricalData.h"
#include "imgConstruct.h"
#include "tcp_ip.h"
#include "logger.h"


struct thread_args {
  	double theta, length, width, height;
  	cv::Mat src, xyz;

  	// locks for the different threads
  	pthread_mutex_t img_mutex, theta_mutex, dimensions_mutex, thetaTrans_mutex, dimensionsTrans_mutex;
  	pthread_cond_t  img_cond, theta_cond, dimensions_cond, thetaTrans_cond, dimensionsTrans_cond;

  	cv::Point2f objCorners[4];
};

void* structInit(void* a) {

	std::cout << "structInit()" << std::endl;
	thread_args *b;
	b = (thread_args*)a;

	config c;

	// Asks if you want to reset the conveyor distance or change parameters in the config file
	c.changeParameters();

	try
	{
		b->theta = 0;
		b->length = 0;
		b->width = 0;
		b->height = 0;

	  	b->img_mutex 				= PTHREAD_MUTEX_INITIALIZER;
	  	b->img_cond 				= PTHREAD_COND_INITIALIZER;
	  	b->theta_mutex 				= PTHREAD_MUTEX_INITIALIZER;
	  	b->theta_cond 				= PTHREAD_COND_INITIALIZER;
	  	b->dimensions_mutex 		= PTHREAD_MUTEX_INITIALIZER;
	  	b->dimensions_cond 			= PTHREAD_COND_INITIALIZER;
	  	b->thetaTrans_mutex 		= PTHREAD_MUTEX_INITIALIZER;
	  	b->thetaTrans_cond			= PTHREAD_COND_INITIALIZER;
	  	b->dimensionsTrans_mutex 	= PTHREAD_MUTEX_INITIALIZER;
	  	b->dimensionsTrans_cond 	= PTHREAD_COND_INITIALIZER;
	}
	catch(const char* msg)
	{
		std::cout << "ERROR... failed to initialize Struct variables!!" << std::endl;
		exit(-1);
	}
}

void* getImage(void* a) { 

	std::cout << "getImage()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	config c;
	logger log("log-file");
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

	log.write('m', "coeff_A: " + std::to_string(coeff[0]));
	log.write('m', "coeff_B: " + std::to_string(coeff[1]));
	log.write('m', "roi_width: " + std::to_string(roi_width));
	log.write('m', "roi_height: " + std::to_string(roi_height));
	log.write('m', "conf_dist: " + std::to_string(conf_dist));

	while (true){

		//auto get_start = std::chrono::high_resolution_clock::now();

		if (! fg->WaitForFrame(img.get(), 1000))
		{
			log.write('e', "In getImage()... Timeout waiting for camera!");
	      	std::cout << "Timeout waiting for camera!" << std::endl;
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

void* pointCloudMethod(void* a) {

	std::cout << "pointCloudMethod()" << std::endl;

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

	geometricalData geoData;

	while (true) {

		pthread_cond_wait(&b->theta_cond, &b->theta_mutex);

		//auto process_start = std::chrono::high_resolution_clock::now();
		
		b->theta = geoData.getOrientation(b->objCorners);
		std::cout << "theta: " << int(b->theta) << std::endl; 

		pthread_cond_signal(&b->thetaTrans_cond);

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

		b->length = L_W_H[0];
		b->width = L_W_H[1];
		b->height = L_W_H[2];

		pthread_cond_signal(&b->dimensionsTrans_cond);

		//auto dim_finish = std::chrono::high_resolution_clock::now();
	    //printf("get dimensions ms: %d\n", std::chrono::duration_cast<std::chrono::milliseconds>(dim_finish - dim_start).count());


		std::cout << "\nobject length: " << std::setprecision(3) << b->length / 10 << "cm" << std::endl;
		std::cout << "object width: " << std::setprecision(3) << b->width / 10 << "cm" << std::endl;		
		std::cout << "object height: " << std::setprecision(3) << b->height / 10 << "cm" << std::endl;
	}
}

void* dataTransfer(void* a) {

	std::cerr << "sendData()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	logger log("log-file");

	tcp_ip t;
	int port = 12345;
	bool tcpFailure = true;

	while (true)
	{
		pthread_cond_wait(&b->thetaTrans_cond, &b->thetaTrans_mutex);
		pthread_cond_wait(&b->dimensionsTrans_cond, &b->dimensionsTrans_mutex);

		//auto dim_start = std::chrono::high_resolution_clock::now();

		std::string data;
		data = "theta: ";
		data += std::to_string(b->theta) + ", ";
		data += "length: ";
		data += std::to_string(b->length) + ", ";
		data += "width: ";
		data += std::to_string(b->width) + ", ";
		data += "height: ";
		data += std::to_string(b->height);

		while (tcpFailure) 
		{
			t.sendData(data, port);

			if (t.tcpSuccess)
			{
				tcpFailure = false;
				std::cout << "data has been sent" << std::endl;
			}
			else
			{
				log.write('e', "In dataTransfer()... failed to send the data");
				std::cout << "failed to send the data" << std::endl;	
			}
		}			

		//auto dim_finish = std::chrono::high_resolution_clock::now();
	    //printf("get dimensions ms: %d\n", std::chrono::duration_cast<std::chrono::milliseconds>(dim_finish - dim_start).count());
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
	//std::thread thread5(dataTransfer, args);
	//sleep(2);

	// Holds the main thread until the thread has finished
	thread1.join();
  	thread2.join();
  	thread3.join();
  	thread4.join();
  	//thread5.join();

  	return 0;
}

