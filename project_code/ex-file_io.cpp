// ex-file_io.cpp
//
// Capture a frame from the camera, and write the data out to files. For
// exemplary purposes, we will write the amplitdue and radial distance images
// to PNG files. NOTE: we have removed the PCL I/O from this example for now as
// we are trying to deprecate PCL from our library.
//

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

	// draw the rotated rect
	cv::line(temp_img, corners[0], corners[1], cv::Scalar(0, 0, 255));
	cv::line(temp_img, corners[1], corners[2], cv::Scalar(0, 0, 255));
	cv::line(temp_img, corners[2], corners[3], cv::Scalar(0, 0, 255));
	cv::line(temp_img, corners[3], corners[0], cv::Scalar(0, 0, 255));

	dst = temp_img;
}


double getRotateAngle(cv::Point2f *corners) {
	double hyp = 0, hos = 0, mod = 0, tempMod, tempHos;
	std::vector<double> tempHyp = { 0,0,0,0 };

	for (size_t i = 0; i < tempHyp.size() - 1; i++)
	{
		tempMod = abs(double(corners[1 + i].y) - double(corners[0 + i].y));
		tempHos = abs(double(corners[1 + i].x) - double(corners[0 + i].x));
		tempHyp[i] = sqrt(pow(tempMod, 2) + pow(tempHos, 2));

		if (tempHyp[i] > hyp) {
			hyp = tempHyp[i];
			hos = tempHos;
			mod = tempMod;
		}
	}
	//std::cout << "theta atan2: " << int(atan2(mod,hos) * 180 / 3.14159265359) << std::endl; // Theta
	return round((atan(mod / hos) * 180 / 3.14159265359) *100) /100; // Theta
}



int main(int argc, const char **argv)
{
  	auto cam = ifm3d::Camera::MakeShared();

  	ifm3d::ImageBuffer::Ptr img = std::make_shared<ifm3d::ImageBuffer>();
  	ifm3d::FrameGrabber::Ptr fg =
    	std::make_shared<ifm3d::FrameGrabber>(
      	cam, ifm3d::IMG_AMP|ifm3d::IMG_CART|ifm3d::IMG_RDIS);


    if (! fg->WaitForFrame(img.get(), 1000))
    {
      	std::cerr << "Timeout waiting for camera!" << std::endl;
      	return -1;
    }
    cv::Mat gray_img, back_img, grayBack_img, binary_img, morph_img, conture_img;


    int arr_theta[5], counter = 0;

    back_img = img->DistanceImage();
    grayScale(back_img, grayBack_img);

    while (true){
  	if (! fg->WaitForFrame(img.get(), 1000))
    {
      	std::cerr << "Timeout waiting for camera!" << std::endl;
      	return -1;
    }

    
    cv::Mat dist_img = img->DistanceImage();

    grayScale(dist_img, gray_img);

    imgSubstractGray(grayBack_img, gray_img, 120); //125 This is for calibrating the distance to the conveyor

    threshold(gray_img, binary_img, 50); // 200 good

    morphing(binary_img, morph_img, 11, 7);


	cv::namedWindow("frame", cv::WINDOW_NORMAL);
	imshow("frame", morph_img);


    cv::RotatedRect boundingBox = getControur(morph_img);

	// draw the rotated rect
	cv::Point2f corners[4];
	boundingBox.points(corners);
	drawRect(dist_img, dist_img, corners, boundingBox);

	arr_theta[counter] = getRotateAngle(corners);

	counter++;

	if (arr_theta[0] > -1 && arr_theta[3] > -1 && arr_theta[5] > -1)
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
		arr_theta[counter] = 0;
		int avg_theta = sum / arr_size; 

		printf("avg theta: %d\n", avg_theta);
	}else
	{
		counter = 0;
	}


    /*
    cv::Mat n = img->AmplitudeImage();
    printf("size: %d, %d\n", n.cols, n.rows);
    printf("n pixel : %d\n", n.at<uchar>(cv::Point(200,25)));
    printf("d pixel : %d\n", d.at<uchar>(cv::Point(200,25)));
    */

    cv::namedWindow("Base Image", cv::WINDOW_NORMAL);
	imshow("Base Image", dist_img);
    cv::waitKey(1);
	}

  	//imwrite("amplitude.png", n);
  	//imwrite("radial_distance.png", d);

  	return 0;
}
