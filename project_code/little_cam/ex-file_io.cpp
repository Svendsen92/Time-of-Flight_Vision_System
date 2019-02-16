#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>


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



int main(int argc, const char **argv)
{
  	auto cam = ifm3d::Camera::MakeShared();

  	ifm3d::ImageBuffer::Ptr img = std::make_shared<ifm3d::ImageBuffer>();
  	ifm3d::FrameGrabber::Ptr fg = std::make_shared<ifm3d::FrameGrabber>(
      								cam, ifm3d::IMG_AMP|ifm3d::IMG_CART|ifm3d::IMG_RDIS);


    if (! fg->WaitForFrame(img.get(), 1000))
    {
      	std::cerr << "Timeout waiting for camera!" << std::endl;
      	return -1;
    }
    cv::Mat gray_img, back_img, grayBack_img, binary_img, morph_img, conture_img;

    double arr_theta[5] = {-1}; 
    int counter = 0;

    back_img = img->DistanceImage();
    grayScale(back_img, grayBack_img);

    while (true){
	  	if (! fg->WaitForFrame(img.get(), 1000))
	    {
	      	std::cerr << "Timeout waiting for camera!" << std::endl;
	      	return -1;
	    }

	    
	    cv::Mat dist_img = img->DistanceImage();


	    cv::GaussianBlur(dist_img, dist_img, cv::Size(5, 5), 0, 0);

	    grayScale(dist_img, gray_img);
	    cv::namedWindow("gray", cv::WINDOW_NORMAL);
		imshow("gray", gray_img);

	//	Image Substraction method
	    imgSubstractGray(grayBack_img, gray_img, 120); //125 This is for calibrating the distance to the conveyor

	    threshold(gray_img, binary_img, 50); // 50 good

	    morphing(binary_img, morph_img, 9, 7); // 11, 7
		cv::namedWindow("morph", cv::WINDOW_NORMAL);
		imshow("morph", morph_img);

	// Other
	/*    threshold(gray_img, binary_img, 170); // 170
	    cv::namedWindow("binary", cv::WINDOW_NORMAL);
		imshow("binary", binary_img);

	    morphing(binary_img, morph_img, 5, 5); 
		cv::namedWindow("morph", cv::WINDOW_NORMAL);
		imshow("morph", morph_img);
	*/

	    cv::RotatedRect boundingBox = getControur(morph_img);

		// draw the rotated rect
		cv::Point2f corners[4];
		boundingBox.points(corners);
		drawRect(dist_img, dist_img, corners, boundingBox);

		arr_theta[counter] = getRotateAngle(corners);


		counter++;
		if (arr_theta[0] > -1 && arr_theta[1] > -1 && 
			arr_theta[2] > -1 && arr_theta[3] > -1 && arr_theta[4] > -1)
		{
			int arr_size = (sizeof(arr_theta)/sizeof(*arr_theta));

			if (counter == 5)
			{
				counter = 0;
			}

			int sum = 0;
			for(int i = 0; i < arr_size -1; i++){
	   			sum += arr_theta[i];
			}
			
			int avg_theta = sum / (arr_size-1); 
			printf("avg theta: %d\n", avg_theta);
		}else{
			counter = 0;
		}
		

	    cv::namedWindow("Base Image", cv::WINDOW_NORMAL);
		imshow("Base Image", dist_img);
	    cv::waitKey(1);
	}


  	return 0;
}