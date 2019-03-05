#include <iostream>
#include <memory>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>


#include <thread>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <chrono>  // for high_resolution_clock


struct thread_args {
  	int counter;
  	double arr_theta[5], theta, roi_width, roi_hight;
  	cv::Mat src, dst, background_src;

  	// locks for the different threads
  	pthread_mutex_t img_mutex, theta_mutex, cam_mutex;
  	pthread_cond_t  img_cond, theta_cond, cam_cond;

  	bool cam_busy;
};

void setRoI(cv::Mat &src, cv::Mat &dst, double width_scaler, double hight_scaler){

	int width = int(src.cols*width_scaler);
	int hight = int(src.rows*hight_scaler);

	int x = int((src.cols-width)/2);
	int y = int((src.rows-hight)/2);

	cv::Rect RoI = cv::Rect(x, y, width, hight);
	dst = src(RoI);
}



void grayScale(cv::Mat &src, cv::Mat &dst) {
	cv::Mat temp = cv::Mat(src.rows, src.cols, CV_8U);

	for (int x = 0; x < src.cols; x++) {
		for (int y = 0; y < src.rows; y++) {
			temp.at<uchar>(cv::Point(x, y)) = (src.at<cv::Vec3b>(cv::Point(x, y))[0] + src.at<cv::Vec3b>(cv::Point(x, y))[1] + src.at<cv::Vec3b>(cv::Point(x, y))[2]) / 3;
		}
	}
	dst = temp;
}

void imgSubstractGray(cv::Mat &src, cv::Mat &dst, int threshold) {

	for (int x = 0; x < src.cols; x++) {
		for (int y = 0; y < src.rows; y++) {
			if (dst.at<uchar>(cv::Point(x, y)) + threshold > src.at<uchar>(cv::Point(x, y)) && 
				dst.at<uchar>(cv::Point(x, y)) - threshold < src.at<uchar>(cv::Point(x, y))) 
			{
				dst.at<uchar>(cv::Point(x, y)) = 0;
			}
		}
	}
}

void threshold(cv::Mat &src, cv::Mat &dst, int threshold) {
	dst = cv::Mat(src.rows, src.cols, CV_8U);

	for (int x = 0; x < src.cols; x++) {
		for (int y = 0; y < src.rows; y++) {
			if (src.at<uchar>(cv::Point(x, y)) < threshold)
			{
				dst.at<uchar>(cv::Point(x, y)) = 0;
			}
			else
			{
				dst.at<uchar>(cv::Point(x, y)) = 255;
			}
		}
	}
}

void morphing(cv::Mat &src, cv::Mat &dst, int closeE, int openE) {
	
	cv::Mat closeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(closeE, closeE));
	morphologyEx(src, dst, cv::MORPH_CLOSE, closeElement);
	cv::Mat openElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(openE, openE));
	morphologyEx(dst, dst, cv::MORPH_OPEN, openElement);
}

cv::RotatedRect getControur(cv::Mat &img) {

	std::vector<std::vector<cv::Point>> contours;
	findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	int biggestContourIdx = -1;
	float biggestContourArea = 0;
	for (int i = 0; i < contours.size(); i++) {
		drawContours(img, contours, i, cv::Scalar(0, 0, 255), 1, 8);

		double ctArea = cv::contourArea(contours[i]);
		if (ctArea > biggestContourArea)
		{
			biggestContourArea = ctArea;
			biggestContourIdx = i;
		}
	}

	cv::RotatedRect boundingBox;
	if (biggestContourArea < 150)
	{
		return boundingBox;
	}else{
		boundingBox = cv::minAreaRect(contours[biggestContourIdx]);
		return boundingBox;
	}	
}

void drawRect(cv::Mat &src, cv::Mat &dst, cv::Point2f *corners, cv::RotatedRect boundingBox) {
	cv::Mat temp_img = src;

	for (int i = 0; i < 4; ++i)
	{
		corners[i].x = corners[i].x +30;
	}


	// draw the rotated rect
	cv::line(temp_img, corners[0], corners[1], cv::Scalar(0, 0, 0));
	cv::line(temp_img, corners[1], corners[2], cv::Scalar(0, 0, 0));
	cv::line(temp_img, corners[2], corners[3], cv::Scalar(0, 0, 0));
	cv::line(temp_img, corners[3], corners[0], cv::Scalar(0, 0, 0));

	dst = temp_img;
}

double getRotateAngle(cv::Point2f *corners) {
	int tempMod = 0, tempHos = 0; 
	double hyp = 0, hos = 0, mod = 0, tempHyp = 0;

	for (size_t i = 0; i < 4; i++)
	{
		tempMod = int(corners[1 + i].y) - int(corners[0 + i].y);
		tempHos = int(corners[1 + i].x) - int(corners[0 + i].x);
		tempHyp = sqrt(pow(tempMod, 2) + pow(tempHos, 2));

		if (tempHyp > hyp) {
			hyp = tempHyp;
			hos = tempHos;
			mod = tempMod;
		}
	}
	printf("theta1: %lf\n", (atan(hos/mod) * 180 / 3.14159265359));
	return (atan(hos/mod) * 180 / 3.14159265359);
}




void* structInit(void* a){

	std::cerr << "structInit()" << std::endl;
	thread_args *b;
	b = (thread_args*)a;

	try
	{
		b->theta = 0;
	  	b->counter = 0;
	  	for (int i = 0; i < 5; ++i)
	  	{
	  		b->arr_theta[i] = 0;
	  	}

	  	b->roi_width = 1;
	  	b->roi_hight = 1;


	  	auto cam = ifm3d::Camera::MakeShared();
  	 
	  	ifm3d::ImageBuffer::Ptr img = std::make_shared<ifm3d::ImageBuffer>();
	 	ifm3d::FrameGrabber::Ptr fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::IMG_AMP|ifm3d::IMG_CART|ifm3d::IMG_RDIS);

	  	b->img_mutex = PTHREAD_MUTEX_INITIALIZER;
	  	b->img_cond = PTHREAD_COND_INITIALIZER;
	  	b->theta_mutex = PTHREAD_MUTEX_INITIALIZER;
	  	b->theta_cond = PTHREAD_COND_INITIALIZER;
		b->cam_mutex = PTHREAD_MUTEX_INITIALIZER;
	  	b->cam_cond = PTHREAD_COND_INITIALIZER;	  	

	  	b->cam_busy = false;

	  	if (! fg->WaitForFrame(img.get(), 1000)){
      		std::cerr << "Timeout waiting for camera!" << std::endl;
      		exit(-1);
	    }

	    cv::Mat back_img, back_dst;
	  	//back_img = img->DistanceImage();

	  	imwrite("amplitude.png", img->AmplitudeImage());
  		back_img = cv::imread("//home//vm//project//build//file_io//amplitude.png");

	    grayScale(back_img, back_dst);

	    b->background_src = back_dst;

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

	cv::Mat src = cv::Mat(b->background_src.rows, b->background_src.cols, CV_32F);

	auto cam = ifm3d::Camera::MakeShared();

	ifm3d::ImageBuffer::Ptr img = std::make_shared<ifm3d::ImageBuffer>();
	//ifm3d::FrameGrabber::Ptr fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::IMG_AMP|ifm3d::IMG_CART|ifm3d::IMG_RDIS);
	ifm3d::FrameGrabber::Ptr fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::IMG_AMP|ifm3d::IMG_RDIS);


	while (true){

		while (b->cam_busy){}
		b->cam_busy = true;


		if (! fg->WaitForFrame(img.get(), 1000))
		{
	      	std::cerr << "Timeout waiting for camera!" << std::endl;
	      	exit(-1);
	    }

		b->cam_busy = false; 

	    // The locks protects shared data.
	    pthread_mutex_lock(&b->img_mutex);


		//auto process_start = std::chrono::high_resolution_clock::now();

	    //imwrite("amplitude.png", img->AmplitudeImage());
  		//src = cv::imread("//home//vm//project//build//file_io//amplitude.png");

		src = img->DistanceImage();
	    //src = img->ConfidenceImage();
	    //src = img->XYZImage();

	    setRoI(src, b->src, b->roi_width, b->roi_hight);
		    
	    pthread_mutex_unlock(&b->img_mutex);
	    pthread_cond_signal(&b->img_cond);


	    //auto process_finish = std::chrono::high_resolution_clock::now();
	    //printf("get image ms: %d\n", std::chrono::duration_cast<std::chrono::milliseconds>(process_finish - process_start).count());
	}
}

void* amplitudeMethod(void* a){

	std::cerr << "amplitudeMethod()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	while (true){

		//auto process_start = std::chrono::high_resolution_clock::now();

		pthread_cond_wait(&b->img_cond, &b->img_mutex);

		//auto process_start = std::chrono::high_resolution_clock::now();

		cv::Mat src = b->src;
		cv::Mat gauss, gray, morph, thres;

		cv::GaussianBlur(src, gauss, cv::Size(3, 3), 0, 0);

		grayScale(gauss, gray);
		cv::namedWindow("gray", cv::WINDOW_NORMAL);
		imshow("gray", gray);

		
		threshold(gray, thres, 40); // 170
		cv::namedWindow("binary", cv::WINDOW_NORMAL);
		imshow("binary", thres);

	    morphing(thres, morph, 3, 3); 
		cv::namedWindow("morph", cv::WINDOW_NORMAL);
		imshow("morph", morph);
		

	    cv::RotatedRect boundingBox = getControur(morph);

		// draw the rotated rect
		cv::Point2f corners[4];
		boundingBox.points(corners);
		drawRect(src, src, corners, boundingBox);

		cv::namedWindow("Base Image", cv::WINDOW_NORMAL);
		imshow("Base Image", src);
		cv::waitKey(1);


		pthread_mutex_lock(&b->theta_mutex);

	    b->theta = getRotateAngle(corners);
		
	    pthread_mutex_unlock(&b->theta_mutex);
	    pthread_cond_signal(&b->theta_cond);

	    //auto process_finish = std::chrono::high_resolution_clock::now();
	    //printf("image processing ms: %d\n", std::chrono::duration_cast<std::chrono::milliseconds>(process_finish - process_start).count());
	}
}

void* subtractionMethod(void* a){

	thread_args *b;
	b = (thread_args*)a;


	cv::Mat back_sub;
	setRoI(b->background_src, back_sub, b->roi_width, b->roi_hight);

	while (true){
		
		cv::Mat src;
		cv::Mat gauss, gray, morph, thres;

		pthread_cond_wait(&b->img_cond, &b->img_mutex);

		//auto process_start = std::chrono::high_resolution_clock::now();

		//cv::GaussianBlur(src, gauss, cv::Size(3, 3), 0, 0);
		
	    grayScale(src, gray);
		
	    imgSubstractGray(back_sub, gray, 5);
		cv::namedWindow("subtrac", cv::WINDOW_NORMAL);
		imshow("subtrac", gray);	    


	    threshold(gray, thres, 5); // 50 good
	    cv::namedWindow("binary", cv::WINDOW_NORMAL);
		imshow("binary", thres);

	    morphing(thres, morph, 5, 5); // 11, 7
		cv::namedWindow("morph", cv::WINDOW_NORMAL);
		imshow("morph", morph);


	    cv::RotatedRect boundingBox = getControur(morph);

		// draw the rotated rect
		cv::Point2f corners[4];
		boundingBox.points(corners);
		drawRect(src, src, corners, boundingBox);


		cv::namedWindow("Base Image", cv::WINDOW_NORMAL);
		imshow("Base Image", src);
		cv::waitKey(1);


		pthread_mutex_lock(&b->theta_mutex);

	    b->theta = getRotateAngle(corners);

	    pthread_mutex_unlock(&b->theta_mutex);
	    pthread_cond_signal(&b->theta_cond);

	    //auto process_finish = std::chrono::high_resolution_clock::now();
	    //printf("image processing ms: %d\n", std::chrono::duration_cast<std::chrono::milliseconds>(process_finish - process_start).count());
	}
}

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


char *concatStr(char *str1, char *str2){

   char *str3 = (char *) malloc(1 + strlen(str1)+ strlen(str2));
   strcpy(str3, str1);
   strcat(str3, str2);

   return str3;
}

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





int main(int argc, const char **argv)
{

	thread_args *args = new thread_args;

  	if (setCamParam()){
  		exit(0);
  	}

  	// Initializes the variables in the Struct
  	structInit((void*) args);


  	std::thread thread1(getImage, args);
  	sleep(2);
  	std::thread thread2(getImage, args);
  	sleep(2);
	std::thread thread3(amplitudeMethod, args);
	//std::thread thread3(subtractionMethod, args);
	sleep(2);
	std::thread thread4(getThetaAvg, args);
	sleep(2);

	thread1.join();
  	thread2.join();
  	thread3.join();
  	thread4.join();

  	return 0;
}