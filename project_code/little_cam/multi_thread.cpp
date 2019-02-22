#include <iostream>
#include <memory>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>


struct thread_args {
  	int counter;
  	double arr_theta[5], theta;
  	cv::Mat src, dst, background_src;

  	// locks for the different threads
  	pthread_mutex_t img_mutex, theta_mutex;
  	pthread_cond_t  img_cond, theta_cond;
};

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
		corners[i].x = corners[i].x -30;
	}	


	// draw the rotated rect
	cv::line(temp_img, corners[0], corners[1], cv::Scalar(0, 0, 255));
	cv::line(temp_img, corners[1], corners[2], cv::Scalar(0, 0, 255));
	cv::line(temp_img, corners[2], corners[3], cv::Scalar(0, 0, 255));
	cv::line(temp_img, corners[3], corners[0], cv::Scalar(0, 0, 255));

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

		if (tempHyp > hyp) 
		{
			hyp = tempHyp;
			hos = tempHos;
			mod = tempMod;
		}
	}
	//printf("theta1: %lf\n", (atan(hos / mod) * 180 / 3.14159265359));
	return (atan(hos / mod) * 180 / 3.14159265359); // Theta
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

	  	auto cam = ifm3d::Camera::MakeShared();
  	 
	  	ifm3d::ImageBuffer::Ptr img = std::make_shared<ifm3d::ImageBuffer>();
	 	ifm3d::FrameGrabber::Ptr fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::IMG_AMP|ifm3d::IMG_CART|ifm3d::IMG_RDIS);

	  	b->img_mutex = PTHREAD_MUTEX_INITIALIZER;
	  	b->img_cond = PTHREAD_COND_INITIALIZER;
	  	b->theta_mutex = PTHREAD_MUTEX_INITIALIZER;
	  	b->theta_cond = PTHREAD_COND_INITIALIZER;

	  	if (! fg->WaitForFrame(img.get(), 1000)){
      		std::cerr << "Timeout waiting for camera!" << std::endl;
      		exit(-1);
	    }

	  	cv::Mat back_img = img->DistanceImage();

	    grayScale(back_img, back_img);

	    b->background_src = back_img;

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

	auto cam = ifm3d::Camera::MakeShared();

	ifm3d::ImageBuffer::Ptr img = std::make_shared<ifm3d::ImageBuffer>();
	ifm3d::FrameGrabber::Ptr fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::IMG_AMP|ifm3d::IMG_CART|ifm3d::IMG_RDIS);

	while (true){

		if (! fg->WaitForFrame(img.get(), 1000))
		{
	      	std::cerr << "Timeout waiting for camera!" << std::endl;
	      	exit(-1);
	    }
	    
	    // The locks protects shared data.
	    pthread_mutex_lock(&b->img_mutex);
	    b->src = img->DistanceImage();
	    pthread_mutex_unlock(&b->img_mutex);
	    pthread_cond_signal(&b->img_cond);
	}
}

void* amplitudeMethod(void* a){

	std::cerr << "amplitudeMethod()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	while (true){

		pthread_cond_wait(&b->img_cond, &b->img_mutex);
		cv::Mat src = b->src;
		cv::Mat gauss, gray, morph, thres;


		cv::GaussianBlur(src, gauss, cv::Size(3, 3), 0, 0);

		grayScale(gauss, gray);
		//cv::namedWindow("gray", cv::WINDOW_NORMAL);
		//imshow("gray", gray);

		threshold(gray, thres, 150); // 170
		//cv::namedWindow("binary", cv::WINDOW_NORMAL);
		//imshow("binary", thres);

	    morphing(thres, morph, 3, 3); 
		//cv::namedWindow("morph", cv::WINDOW_NORMAL);
		//imshow("morph", morph);
		

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
	    
	}
}

void* subtractionMethod(void* a){

	thread_args *b;
	b = (thread_args*)a;

	while (true){
		
		pthread_cond_wait(&b->img_cond, &b->img_mutex);

		cv::Mat src = b->src;
		cv::Mat dst = b->dst;
			
		cv::GaussianBlur(src, dst, cv::Size(5, 5), 0, 0);

	    grayScale(dst, dst);
	    cv::namedWindow("gray", cv::WINDOW_NORMAL);
		imshow("gray", dst);

	    imgSubstractGray(b->background_src, dst, 120); //125 This is for calibrating the distance to the conveyor
	    threshold(dst, dst, 50); // 50 good
	    morphing(dst, dst, 9, 7); // 11, 7

		cv::namedWindow("morph", cv::WINDOW_NORMAL);
		imshow("morph", dst);

	    cv::RotatedRect boundingBox = getControur(dst);

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
	}
}

void* getThetaAvg(void* a){

	std::cerr << "getThetaAvg()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	while (true){

		pthread_cond_wait(&b->theta_cond, &b->theta_mutex);

		b->arr_theta[b->counter] = b->theta;

		b->counter++;
		if (b->arr_theta[0] > -1 && b->arr_theta[1] > -1 && 
			b->arr_theta[2] > -1 && b->arr_theta[3] > -1 && b->arr_theta[4] > -1)
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
			printf("avg theta: %d\n", avg_theta);
		}
		else
		{
			b->counter = 0;
		}
	}
}



int main(int argc, const char **argv)
{

	pthread_t thread_1, thread_2, thread_3;

  	thread_args *args = new thread_args;

  	// initializes the variables in the Struct
  	structInit((void*) args);

    // creates the thread for acquiring images continously
	if (pthread_create(&thread_1, NULL, &getImage, (void*) args)) {
        std::cerr << "Error:unable to create thread_1" << std::endl;
     	exit(-1);
    }

    std::cerr << "thread_1 created" << std::endl;
	// creates the thread for image processing using the Amplitude method
	sleep(2);
	if (pthread_create(&thread_2, NULL, &amplitudeMethod, (void*) args)) {
        std::cerr << "Error:unable to create thread_2" << std::endl;
        pthread_exit(&thread_1);
        std::cerr << "thread_1 STOPPED" << std::endl;
        exit(-1);
    }

    // creates the thread for image processing using the Subtraction method
/*
	sleep(2);
	if (pthread_create(&thread_2, NULL, subtractionMethod, (void*) args)) {
        std::cerr << "Error:unable to create thread_2" << std::endl;
        exit(-1);
    }
*/
    std::cerr << "thread_2 created" << std::endl;
    // creates the thread for calculating the theta angle 
    sleep(2);
	if (pthread_create(&thread_3, NULL, &getThetaAvg, (void*) args)) {
        std::cerr << "Error:unable to create thread_3" << std::endl;
        pthread_exit(&thread_1);
        std::cerr << "thread_1 STOPPED" << std::endl;
        pthread_exit(&thread_2);
        std::cerr << "thread_2 STOPPED" << std::endl;
     	exit(-1);
    }

    std::cerr << "thread_3 created" << std::endl;
    sleep(2);
    try
    {
		///// image acquisition thread /////    	
		pthread_join(thread_1, NULL);

	    ///// image processing thread /////
      	pthread_join(thread_2, NULL);

	    ///// angle processing thread /////
      	pthread_join(thread_3, NULL);

    }
    catch(const char* msg)
    {
    	printf("Error msg: %s\n", msg);
    	pthread_exit(&thread_1);
    	std::cerr << "thread_1 STOPPED" << std::endl;
   		pthread_exit(&thread_2);
   		std::cerr << "thread_2 STOPPED" << std::endl;
    	pthread_exit(&thread_3);
    	std::cerr << "thread_3 STOPPED" << std::endl;
   		exit(-1);
    }
  	
  	return 0;
}
