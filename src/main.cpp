#include <iostream>
#include <memory>
#include <thread>
#include <signal.h>

#include <opencv2/opencv.hpp>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>

#include "config.h"
#include "imgProcess.h"
#include "geometricalData.h"
#include "imgConstruct.h"
#include "tcp_ip.h"
#include "logger.h"
#include "writeCSV.h"


struct thread_args {
	int errorFlag;
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
		b->errorFlag = 0;
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


	while (true)
	{
		if (fg->WaitForFrame(img.get(), 1000))
		{

			xyz = img->XYZImage();
			conf = img->ConfidenceImage();
			
			construct.generateImage(xyz, conf, coeff, conf_dist, roi_width, roi_height);

			// The locks protects shared data.
		    pthread_mutex_lock(&b->img_mutex);

			b->xyz = cv::Mat(construct.xyz_rec.rows, construct.xyz_rec.cols, CV_16SC3);
			b->xyz = construct.xyz_rec;
			b->src = construct.cloud_rec;
			
		    pthread_mutex_unlock(&b->img_mutex);
		    pthread_cond_signal(&b->img_cond);

		}
		else
		{
			log.write('m', "In getImage()... Waiting for trigger signal!");
	      	//std::cout << "In getImage()... Waiting for trigger signal!" << std::endl;
	      	//exit(-1);
		}			    
	}
}

void* imageProcessing(void* a) {

	std::cout << "imageProcessing()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	imgProcess img;

	cv::Mat src, thres, morph;

	while (true){

		pthread_cond_wait(&b->img_cond, &b->img_mutex);

		src = b->src;
		cv::Point2f corners[4];


		img.threshold(src, thres, 10);  
		cv::namedWindow("binary", cv::WINDOW_NORMAL);
		imwrite("thres_image.png", thres);
		imshow("binary", thres);
			
	    img.morphing(thres, morph, 17); 
	    cv::namedWindow("morph", cv::WINDOW_NORMAL);
		imwrite("morph_image.png", morph);
		imshow("morph", morph);

	    cv::RotatedRect boundingBox = img.getControur(morph);

		// draw the rotated rect
		boundingBox.points(corners);
		img.drawRect(src, src, corners, boundingBox);

		cv::namedWindow("Base Image", cv::WINDOW_NORMAL);
		imwrite("base_image.png", src);
		imshow("Base Image", src);
		cv::waitKey(1);


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

	}
}

void* getRotateAngle(void* a) {

	std::cerr << "getRotateAngle()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	geometricalData geoData;

	while (true) {

		pthread_cond_wait(&b->theta_cond, &b->theta_mutex);
		
		b->theta = geoData.getOrientation(b->objCorners);
		std::cout << "theta: " << std::setprecision(3) << b->theta << std::endl;
		
		pthread_cond_signal(&b->thetaTrans_cond);

	}
}

void* getDimensions(void* a) {

	std::cerr << "getDimensions()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	config c;
	geometricalData geoData;

	double conf_dist = std::atof(c.getParameter("dist").c_str()) +5;
	

	while (true)
	{

		pthread_cond_wait(&b->dimensions_cond, &b->dimensions_mutex);

		double LWH[3];
		double* L_W_H = geoData.getDimensions(b->xyz, b->src, LWH, b->objCorners, conf_dist);

		b->length = L_W_H[0];
		b->width = L_W_H[1];
		b->height = L_W_H[2];

		std::cout << "object length: " << std::setprecision(3) << b->length / 10 << "cm" << std::endl;
		std::cout << "object width: " << std::setprecision(3) << b->width / 10 << "cm" << std::endl;		
		std::cout << "object height: " << std::setprecision(3) << b->height / 10 << "cm\n" << std::endl;
		

		pthread_cond_signal(&b->dimensionsTrans_cond);

	}
}



void* getGeometricalData(void* a) {

	thread_args *b;
	b = (thread_args*)a;

	config c;
	double conf_dist = std::atof(c.getParameter("dist").c_str()) +5;

	geometricalData geoData;

	int numOfProducts = 5;
	int sampleSize = 10;
	
	writeCSV thetaCSV("T3-25-06_80m_dynamic_-51angle.csv");

	std::string thetaStr = "";
	thetaStr += "Water_Bottles,";
	thetaStr += "Lenor_pulver,";
	//thetaStr += "Gillette,";
	//thetaStr += "Adez_Soy,";
	thetaStr += "Cola_1.5L,";
	//thetaStr += "Freeway,";
	thetaStr += "Granini_juice,";
	//thetaStr += "Tonic_water,";
	//thetaStr += "Bleer,";
	thetaStr += "Pizza_sauce,";
	//thetaStr += "Edgel_salad,";
	//thetaStr += "Benco,";
	thetaStr += "\n";

	thetaCSV.write(thetaStr);

	double csvTheta[numOfProducts][sampleSize];
	std::string ThetaCsvStr = "";




	writeCSV dimCSV("T3-25-06_80m_dimensions_-51.csv");

	std::string dimStr = "";
	dimStr += "Water_Bottles Length,Water_Bottles Width,Water_Bottles Height,";
	dimStr += "Lenor_pulver Length,Lenor_pulver Width,Lenor_pulver Height,";
	//dimStr += "Gillette Length,Gillette Width,Gillette Height,";
	//dimStr += "Adez_Soy Length,Adez_Soy Width,Adez_Soy Height,";
	dimStr += "Cola_1.5L Length,Cola_1.5L Width,Cola_1.5L Height,";
	//dimStr += "Freeway Length,Freeway Width,Freeway Height,";
	dimStr += "Granini_juice Length,Granini_juice Width,Granini_juice Height,";
	//dimStr += "Tonic_water Length,Tonic_water Width,Tonic_water Height,";
	//dimStr += "Bleer Length,Bleer Width,Bleer Height,";
	dimStr += "Pizza_sauce Length,Pizza_sauce Width,Pizza_sauce Height,";
	//dimStr += "Edgel_salad Length,Edgel_salad Width,Edgel_salad Height,";
	//dimStr += "Benco Length,Benco Width,Benco Height,";
	dimStr += "\n";
	
	dimCSV.write(dimStr);

	double csvLength[numOfProducts][sampleSize];
	double csvWidth[numOfProducts][sampleSize];
	double csvHeight[numOfProducts][sampleSize];
	std::string dimCsvStr = "";


	int counter = 0;
	int cnt = 0;


	while (true) {

		pthread_cond_wait(&b->theta_cond, &b->theta_mutex);

		
		b->theta = geoData.getOrientation(b->objCorners) +1;
		std::cout << "theta: " << std::setprecision(3) << b->theta << std::endl;
		csvTheta[cnt][counter] = b->theta;



		double LWH[3];
		double* L_W_H = geoData.getDimensions(b->xyz, b->src, LWH, b->objCorners, conf_dist);

		b->length = L_W_H[0];
		b->width = L_W_H[1];
		b->height = L_W_H[2];

		std::cout << "object length: " << std::setprecision(3) << b->length / 10 << "cm" << std::endl;
		std::cout << "object width: " << std::setprecision(3) << b->width / 10 << "cm" << std::endl;		
		std::cout << "object height: " << std::setprecision(3) << b->height / 10 << "cm\n" << std::endl;
		
		csvLength[cnt][counter] = b->length;
		csvWidth[cnt][counter] = b->width;
		csvHeight[cnt][counter] = b->height; 


		counter++;
		if (counter == sampleSize)
		{ 
			std::cout << "Put in new product no." << cnt+2 << "\n";
			char c;
			sleep(30);
			std::cin >> c;

			cnt++;
			counter = 0;
			if (cnt+1 > numOfProducts) 
			{	
				int j;
				for (int i = 0; i < sampleSize; ++i)
				{
					for (j = 0; j < numOfProducts-1; ++j)
					{
						ThetaCsvStr += std::to_string(csvTheta[j][i]) + ",";
						dimCsvStr += std::to_string(csvLength[j][i]) + ",";
						dimCsvStr += std::to_string(csvWidth[j][i]) + ",";
						dimCsvStr += std::to_string(csvHeight[j][i]) + ",";
					}
					ThetaCsvStr += std::to_string(csvTheta[j][i]) + ",\n";
					dimCsvStr += std::to_string(csvLength[j][i]) + ",";
					dimCsvStr += std::to_string(csvWidth[j][i]) + ",";
					dimCsvStr += std::to_string(csvHeight[j][i]) + ",\n";
				}
				thetaCSV.write(ThetaCsvStr);
				dimCSV.write(dimCsvStr);
				exit(0);
			}
		}
		pthread_cond_signal(&b->thetaTrans_cond);
	}
}

void* dataTransfer(void* a) {

	std::cerr << "sendData()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	logger log("log-file");

	tcp_ip t;
	int port = 12345;
	std::string serAddress = "server address";

	t.setSocket(serAddress, port);

	while (true)
	{
		pthread_cond_wait(&b->thetaTrans_cond, &b->thetaTrans_mutex);
		pthread_cond_wait(&b->dimensionsTrans_cond, &b->dimensionsTrans_mutex);

		std::string data;
		data = "theta: ";
		data += std::to_string(b->theta) + ", ";
		data += "length: ";
		data += std::to_string(b->length) + ", ";
		data += "width: ";
		data += std::to_string(b->width) + ", ";
		data += "height: ";
		data += std::to_string(b->height); + ", ";
		data += "errorFlag: ";
		data += std::to_string(b->errorFlag);

		for (int i = 1; i < 6; ++i)
		{
		
			t.sendData(data);

			if (t.tcpSuccess)
			{
				std::cout << "data has been sent" << std::endl;
				i = 6;
			}
			else
			{
				log.write('e', "In dataTransfer()... " + std::to_string(i) + " failed attempt to send the data");
				std::cout << "failed to send the data" << std::endl;	
			}
		}			
	}
}



int main(int argc, const char **argv){

	thread_args *args = new thread_args;

  	// Initializes the variables in the Struct
  	structInit((void*) args);
	
  	// Starts the threads
  	std::thread thread1(getImage, args);
  	sleep(2);
	std::thread thread2(imageProcessing, args);
	sleep(2);
	//std::thread thread3(getDimensions, args);
	//sleep(2);
	std::thread thread4(getRotateAngle, args);
	sleep(2);
	///std::thread thread5(getGeometricalData, args);
	///sleep(2);
	//std::thread thread6(dataTransfer, args);
	//sleep(2);

	// Holds the main thread until the thread has finished
	thread1.join();
  	thread2.join();
  	//thread3.join();
  	thread4.join();
  	///thread5.join();
  	//thread6.join();

  	return 0;
}
