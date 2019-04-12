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




struct thread_args {
  	int theta;
  	double arr_theta[5];
  	double roi_width, roi_height, conf_dist, coeff_A, coeff_B;
  	cv::Mat src, dist_img, xyz;

  	// locks for the different threads
  	pthread_mutex_t img_mutex, theta_mutex, bin_mutex;
  	pthread_cond_t  img_cond, theta_cond, bin_cond;

  	cv::Point2f objCorners[4];
};


double getMedian(std::vector<double> arr){

	double median = 0;
	int arrLen = arr.size();
	std::sort(arr.begin(), arr.end());

	if (arrLen % 2 > 0)
	{
		median = (arr[(arrLen+1)/2] + arr[(arrLen-1)/2])/2;	
	}
	else
	{
  		median = arr[arrLen/2];
	}
	return median;
}

std::string getParameter(std::string param) {

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

int setParameter(std::string param, std::string newValue) {

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

double* imgCoeff(double *coeffArr){

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

void setBackgrundDist(){ 

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

void selectionMenu(){

	std::string c = "";
	std::cout << "Chose the action to be taken.\n\n";
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

void changeParameters(){

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

void* structInit(void* a){

	std::cerr << "structInit()" << std::endl;
	thread_args *b;
	b = (thread_args*)a;

	try
	{
		b->theta = 0;
	  	for (int i = 0; i < 5; ++i)
	  	{
	  		b->arr_theta[i] = 0;
	  	}

	  	b->roi_width = std::atof(getParameter("roi_x").c_str());
	  	b->roi_height = std::atof(getParameter("roi_y").c_str());
	  	b->coeff_A = std::atof(getParameter("coeff_A").c_str());
	  	b->coeff_B = std::atof(getParameter("coeff_B").c_str());
	  	b->conf_dist = std::atof(getParameter("dist").c_str());

	  	b->img_mutex 	= PTHREAD_MUTEX_INITIALIZER;
	  	b->img_cond 	= PTHREAD_COND_INITIALIZER;
	  	b->theta_mutex 	= PTHREAD_MUTEX_INITIALIZER;
	  	b->theta_cond 	= PTHREAD_COND_INITIALIZER;
	  	b->bin_mutex 	= PTHREAD_MUTEX_INITIALIZER;
	  	b->bin_cond 	= PTHREAD_COND_INITIALIZER; 	

	}
	catch(const char* msg)
	{
		std::cerr << "ERROR... failed to initialize Struct variables!!" << std::endl;
		exit(-1);
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
	if (biggestContourArea < 20)
	{
		return boundingBox;
	}else{
		boundingBox = cv::minAreaRect(contours[biggestContourIdx]);
		return boundingBox;
	}	
}

void drawRect(cv::Mat &src, cv::Mat &dst, cv::Point2f *corners, cv::RotatedRect boundingBox) {
	cv::Mat temp_img = src;

	// draw the rotated rect
	cv::line(temp_img, corners[0], corners[1], cv::Scalar(255, 255, 255));
	cv::line(temp_img, corners[1], corners[2], cv::Scalar(255, 255, 255));
	cv::line(temp_img, corners[2], corners[3], cv::Scalar(255, 255, 255));
	cv::line(temp_img, corners[3], corners[0], cv::Scalar(255, 255, 255));

	dst = temp_img;
}

int getRotateAngle(cv::Point2f *corners) {

	const float PI = 3.14159265359;
	int tempMod = 0, tempHos = 0; 
	double hyp = 0, hos = 0, mod = 0, tempHyp = 0;

	for (size_t i = 0; i < (sizeof(corners)/2)-1; i++)
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

	int angle = (atan(mod/hos) * 180 / PI);
	return (angle);
}



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



void* getImage(void* a){ 

	std::cerr << "getImage()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	cv::Mat xyz, conf;

	auto cam = ifm3d::Camera::MakeShared();
	ifm3d::ImageBuffer::Ptr img = std::make_shared<ifm3d::ImageBuffer>();
	ifm3d::FrameGrabber::Ptr fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::IMG_AMP|ifm3d::IMG_CART|ifm3d::IMG_RDIS);
	

	while (true){

		if (! fg->WaitForFrame(img.get(), 1000))
		{
	      	std::cerr << "Timeout waiting for camera!" << std::endl;
	      	exit(-1);
	    }

	    //auto process_start = std::chrono::high_resolution_clock::now();

		xyz = img->XYZImage();
		conf = img->ConfidenceImage();

		int width = int(xyz.cols * b->roi_width);
		int height = int(xyz.rows * b->roi_height);
		int x1 = int((xyz.cols - width)/2);
		int y1 = int((xyz.rows - height)/2);

		b->xyz = cv::Mat(height, width, CV_16SC3);
		cv::Mat xyz_img = cv::Mat(height, width, CV_16SC3);
		cv::Mat xyz_rec = cv::Mat::zeros(cv::Size(width, height), CV_16SC3);
		cv::Mat cloud_img = cv::Mat(height, width, CV_8U);
		cv::Mat cloud_rec = cv::Mat::zeros(cv::Size(width, height), CV_8U);
		cv::Mat conf_img = cv::Mat(height, width, CV_8U);


		// The locks protects shared data.
	    pthread_mutex_lock(&b->img_mutex);

	  	for (int x = 0; x < width; x++) 
	  	{
			for (int y = 0; y < height; y++) 
			{		
				// This bit produces the grayscale image, which is used for object detection, based on the distance measurements
				if ((b->coeff_A * xyz.at<cv::Vec3s>(cv::Point(x+x1, y+y1))[0] + b->coeff_B) < 0) // (y = a * x + b) == (y = coeff[0] * x + coeff[1])
				{
					cloud_img.at<uchar>(cv::Point(x, y)) = 0;
				}
				else
				{
					cloud_img.at<uchar>(cv::Point(x, y)) = (b->coeff_A * xyz.at<cv::Vec3s>(cv::Point(x+x1, y+y1))[0] + b->coeff_B);
				}
				xyz_img.at<cv::Vec3s>(cv::Point(x, y)) = xyz.at<cv::Vec3s>(cv::Point(x+x1, y+y1));
				conf_img.at<uchar>(cv::Point(x, y)) = conf.at<uchar>(cv::Point(x+x1, y+y1));

				// This bit filters out invalid pixels
				if (cloud_img.at<uchar>(cv::Point(x, y)) < 255)
				{
					cloud_rec.at<uchar>(cv::Point(x, y)) = cloud_img.at<uchar>(cv::Point(x, y));
					xyz_rec.at<cv::Vec3s>(cv::Point(x, y)) = xyz_img.at<cv::Vec3s>(cv::Point(x, y));
				}
				if (conf_img.at<uchar>(cv::Point(x, y)) > 0) 
				{
					cloud_rec.at<uchar>(cv::Point(x, y)) = cloud_rec.at<uchar>(cv::Point(x - 1, y - 1));
					xyz_rec.at<cv::Vec3s>(cv::Point(x, y)) = xyz_rec.at<cv::Vec3s>(cv::Point(x - 1, y - 1));
				}
			}
		}


		b->xyz = xyz_rec;
		b->src = cloud_rec;

	    pthread_mutex_unlock(&b->img_mutex);
	    pthread_cond_signal(&b->img_cond);

	    //auto process_finish = std::chrono::high_resolution_clock::now();
	    //printf("get image ms: %d\n", std::chrono::duration_cast<std::chrono::milliseconds>(process_finish - process_start).count());
	}
}

void* pointCloudMethod(void* a){

	std::cerr << "pointCloudMethod()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	//int globalCounter = 0;

	while (true){

		pthread_cond_wait(&b->img_cond, &b->img_mutex);

		//auto process_start = std::chrono::high_resolution_clock::now();

		cv::Mat src = b->src;
		cv::Mat morph, thres;

		
		threshold(src, thres, 7); // 10
		cv::namedWindow("binary", cv::WINDOW_NORMAL);
		imshow("binary", thres);

		/*
		globalCounter++;
	    std::string str = std::to_string(globalCounter);
	    std::string binstr = str;
	    binstr += "_thres.png";
	    imwrite(binstr, thres);
		*/

	    morphing(thres, morph, 9, 7); 
		cv::namedWindow("morph", cv::WINDOW_NORMAL);
		imshow("morph", morph);

		/*
		std::string morphstr = str;
	    morphstr += "_morph.png";
	    imwrite(morphstr, morph);
		*/

	    cv::RotatedRect boundingBox = getControur(morph);

		// draw the rotated rect
		cv::Point2f corners[4];
		boundingBox.points(corners);

		drawRect(src, src, corners, boundingBox);

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

		b->objCorners[0] = corners[0];
		b->objCorners[1] = corners[1];
		b->objCorners[2] = corners[2];
		b->objCorners[3] = corners[3];


	    b->theta = getRotateAngle(corners);
	    std::cout << "theta: " << b->theta << std::endl;
		
	    pthread_mutex_unlock(&b->theta_mutex);
	    pthread_cond_signal(&b->theta_cond);

	    //auto process_finish = std::chrono::high_resolution_clock::now();
	    //printf("image processing ms: %d\n", std::chrono::duration_cast<std::chrono::milliseconds>(process_finish - process_start).count());
	}
}


float getSquareArea(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3){

	return fabs((p1.x*(p2.y-p3.y) + p2.x*(p3.y-p1.y) + p3.x*(p1.y-p2.y)));
}

bool inObject(cv::Point2f *corner, cv::Point2f point){

	float A = getSquareArea(corner[0], corner[1], corner[2]);
	float A1 = getSquareArea(point, corner[1], corner[2]);
	float A2 = getSquareArea(corner[0], point, corner[2]);
	float A3 = getSquareArea(corner[0], corner[1], point);

	return (int(A) == int(A1+A2+A3));
}

void* getDimensions(void* a){

	std::cerr << "getDimensions()" << std::endl;

	thread_args *b;
	b = (thread_args*)a;

	cv::Mat img, xyz;
	cv::Point2f corners[4];
	cv::Point2f adjustedCorners[4];

	const float A = -1974.2, B = 1.32989, C = 0.9038051018;

	int numOfPixels = 92928; //img.cols * img.rows;

	while (true)
	{
		pthread_cond_wait(&b->img_cond, &b->img_mutex);

		xyz = b->xyz;
		
		corners[0] = b->objCorners[0];
		corners[1] = b->objCorners[1];
		corners[2] = b->objCorners[2];
		corners[3] = b->objCorners[3];


		std::vector<double> arrdist;
		for (int x = 0; x < xyz.cols; x++) 
	  	{
			for (int y = 0; y < xyz.rows; y++) 
			{	
				if (inObject(b->objCorners, cv::Point2f(x,y)))
				{
					arrdist.push_back(b->conf_dist - xyz.at<cv::Vec3s>(cv::Point(x, y))[0]);
				}
			}
		}


		double H = getMedian(arrdist);
		if (H < 0){
			H = 0;
		}		

		for (size_t i = 0; i < 3; i++)
		{
			adjustedCorners[i].x = xyz.at<cv::Vec3s>(corners[i])[1] + 176; // [1] = x-direction
			adjustedCorners[i].y = xyz.at<cv::Vec3s>(corners[i])[2] + 132; // [2] = y-direction  
		}

		double hyp[3] = {0}, hos = 0, mod = 0;
		for (size_t i = 0; i < 3; i++)
		{
			mod = adjustedCorners[1 + i].y - adjustedCorners[0 + i].y;
			hos = adjustedCorners[1 + i].x - adjustedCorners[0 + i].x;
			hyp[i] = sqrt(pow(mod, 2) + pow(hos, 2));
		}
		int hypLen = sizeof(hyp)/sizeof(*hyp);
		std::sort(hyp, hyp + hypLen);


		double L, W;
		if (hyp[0] > hyp[2])
		{
			L = hyp[0];
			W = hyp[2];	
		}
		else
		{
			W = hyp[0];
			L = hyp[2];		
		}

		std::cout << "\nobject length: " << L << "mm" << std::endl;
		std::cout << "object width: " << W << "mm" << std::endl;		
		std::cout << "object height: " << H << "mm" << std::endl;
	}
}





int main(int argc, const char **argv)
{

	thread_args *args = new thread_args;

	// Asks if you want to reset the conveyor distance or change parameters in the config file
	changeParameters();


  	// Initializes the variables in the Struct
  	structInit((void*) args);
	
  	// Starts the threads
  	std::thread thread1(getImage, args);
  	sleep(2);
	std::thread thread2(pointCloudMethod, args);
	sleep(2);
	//std::thread thread3(getDimensions, args);
	//sleep(2);

	// Holds the main thread until the thread has finished
	thread1.join();
  	thread2.join();
  	//thread3.join();

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
