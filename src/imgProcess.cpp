#include "imgProcess.h"

imgProcess::imgProcess(){}

imgProcess::~imgProcess(){}
		
void imgProcess::grayScale(cv::Mat &src, cv::Mat &dst) {
	cv::Mat temp = cv::Mat(src.rows, src.cols, CV_8U);

	for (int x = 0; x < src.cols; x++) {
		for (int y = 0; y < src.rows; y++) {
			temp.at<uchar>(cv::Point(x, y)) = (src.at<cv::Vec3b>(cv::Point(x, y))[0] + src.at<cv::Vec3b>(cv::Point(x, y))[1] + src.at<cv::Vec3b>(cv::Point(x, y))[2]) / 3;
		}
	}
	dst = temp;
}

void imgProcess::threshold(cv::Mat &src, cv::Mat &dst, int threshold) {
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

void imgProcess::morphing(cv::Mat &src, cv::Mat &dst, int closeE) {

	cv::Mat src_temp = src;
	cv::Mat dst_temp;
	
	cv::Mat closeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(closeE, closeE));
	morphologyEx(src_temp, dst_temp, cv::MORPH_CLOSE, closeElement);

	dst = dst_temp;
}

cv::RotatedRect imgProcess::getControur(cv::Mat &img) {

	cv::Mat temp_img = img;

	std::vector<std::vector<cv::Point>> contours;
	findContours(temp_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	int biggestContourIdx = -1;
	float biggestContourArea = 0;
	for (int i = 0; i < contours.size(); i++) {
		drawContours(temp_img, contours, i, cv::Scalar(0, 0, 255), 1, 8);

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

void imgProcess::drawRect(cv::Mat &src, cv::Mat &dst, cv::Point2f *corners, cv::RotatedRect boundingBox) {
	cv::Mat temp_src = src;

	// draw the rotated rect
	cv::line(temp_src, corners[0], corners[1], cv::Scalar(255, 255, 255));
	cv::line(temp_src, corners[1], corners[2], cv::Scalar(255, 255, 255));
	cv::line(temp_src, corners[2], corners[3], cv::Scalar(255, 255, 255));
	cv::line(temp_src, corners[3], corners[0], cv::Scalar(255, 255, 255));

	dst = temp_src;
}

