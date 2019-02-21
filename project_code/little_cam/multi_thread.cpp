#include <iostream>
#include <memory>
#include <pthread.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>


typedef struct thread_args {
  	int counter;
  	double theta;
  	double arr_theta[5];
  	cv::Mat src;
  	cv::Mat dst;
  	cv::Mat background_src;
  	ifm3d::ImageBuffer::Ptr img;
  	ifm3d::FrameGrabber::Ptr fg;

  	// locks for the different threads
  	int lck_img;
	int lck_theta;
	bool new_img;
	bool new_theta;
} args;

void lock(char arg, void* a){

	args *b;
	b = (args*)a;

	if (arg == 'i'){
		b->lck_img = 0;
	}
	else if (arg == 't'){
		b->lck_theta = 0;
	}
}

void unlock(char arg, void* a){

	args *b;
	b = (args*)a;

	if (arg == 'i'){
		b->lck_img = 1;
	}
	else if (arg == 't'){
		b->lck_theta = 1;
	}
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
	printf("theta1: %lf\n", (atan(hos / mod) * 180 / 3.14159265359));
	return (atan(hos / mod) * 180 / 3.14159265359); // Theta
}



int StructInit(void* a){

	args *b;
	b = (args*)a;

	try
	{
		b->theta = 0;
	  	b->counter = 0;
	  	for (size_t i = 0; i < 5; ++i)
	  	{
	  		b->arr_theta[i] = 0;
	  	}

	  	auto cam = ifm3d::Camera::MakeShared();

	  	b->img = std::make_shared<ifm3d::ImageBuffer>();
	  	b->fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::IMG_AMP|ifm3d::IMG_CART|ifm3d::IMG_RDIS);

	  	b->lck_img = 0;
	  	b->lck_theta = 0;
	  	b->new_theta = false;


	  	if (! b->fg->WaitForFrame(b->img.get(), 1000)){
      	std::cerr << "Timeout waiting for camera!" << std::endl;
      	exit(-1);
	    }
	  	cv::Mat back_img = b->img->DistanceImage();
	    grayScale(back_img, b->background_src);

		return (1);
	}
	catch(const char* msg)
	{
		return (0);
	}
}

void* getImage(void* a){ 
	while (true){
		args *b;
		b = (args*)a;

		if (! b->fg->WaitForFrame(b->img.get(), 1000))
		{
	      	std::cerr << "Timeout waiting for camera!" << std::endl;
	      	exit(-1);
	    }
	    
	    // The locks protects shared data.
	    lock('i', (void*) b);
	    b->dst = b->img->DistanceImage();
	    b->new_img = true;
	    unlock('i', (void*) b);
	}
}

void* amplitudeMethod(void* a){
	while (true){

		args *b;
		b = (args*)a;

		// lck_acquireImage = 0 if locked 
		if (b->lck_img && b->new_img){
			b->new_img = false;

			cv::Mat src = b->src;
			cv::Mat dst = b->dst;

			cv::GaussianBlur(src, dst, cv::Size(5, 5), 0, 0);

		    grayScale(dst, dst);
		    cv::namedWindow("gray", cv::WINDOW_NORMAL);
			imshow("gray", dst);


		    threshold(dst, dst, 170); // 170
		    cv::namedWindow("binary", cv::WINDOW_NORMAL);
			imshow("binary", dst);

		    morphing(dst, dst, 5, 5); 
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

			lock('t', (void*) b);
			b->theta = getRotateAngle(corners);
			b->new_theta = true;
			unlock('t', (void*) b);
		}
	}
}

void* subtractionMethod(void* a){
	while (true){
		args *b;
		b = (args*)a;
		
		// lck_acquireImage = 0 if locked 
		if (b->lck_img && b->new_img){
			b->new_img = false;

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

			lock('t', (void*) b);
			b->theta = getRotateAngle(corners);
			b->new_theta = true;
			unlock('t', (void*) b);
		}
	}
}

void* getThetaAvg(void* a){
	while (true){
		args *b;
		b = (args*)a;

		if (b->lck_theta && b->new_theta){
			b->new_theta = false;

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
}



int main(int argc, const char **argv)
{
	pthread_t thread_1, thread_2, thread_3;

  	args *args_thread;

  	// initializes the variables in the Struct
  	if (StructInit((void*) args_thread)){
  		std::cerr << "ERROR... failed to initialize Struct variables!!" << std::endl;
      	exit(-1);
  	}


    // creates the thread for acquiring images continously
	if (pthread_create(&thread_1, NULL, &getImage, (void*) args_thread)) {
        printf("Error:unable to create thread_1");
     	exit(-1);
    }

	// creates the thread for image processing using the Amplitude method
	sleep(2);
	if (pthread_create(&thread_2, NULL, &amplitudeMethod, (void*) args_thread)) {
        printf("Error:unable to create thread_2");
        exit(-1);
    }

    // creates the thread for image processing using the Subtraction method
/*
	sleep(2);
	if (pthread_create(&thread_2, NULL, subtractionMethod, (void*) args_thread)) {
        printf("Error:unable to create thread_2");
        exit(-1);
    }
*/
    // creates the thread for calculating the theta angle 
    sleep(2);
	if (pthread_create(&thread_3, NULL, &getThetaAvg, (void*) args_thread)) {
        printf("Error:unable to create thread_3");
     	exit(-1);
    }

    sleep(5);
    try{
		///// image acquisition thread /////    	
		pthread_join(thread_1, NULL);

	    ///// image processing thread /////
      	pthread_join(thread_2, NULL);

	    ///// angle processing thread /////
      	pthread_join(thread_3, NULL);

      	///// keeps main thread occupied /////
      	while (true){
      		sleep(10);
      	}

    }
    catch(const char* msg){
    	printf("Error msg: %s\n", msg);
    	pthread_exit(&thread_1);
    	pthread_exit(&thread_2);
    	pthread_exit(&thread_3);
    }

  	return 0;
}
