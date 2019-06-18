#ifndef GEOMETRICALDATA_H
#define GEOMETRICALDATA_H

#include <iostream>
#include <opencv2/opencv.hpp>


class geometricalData
{
    private:
	   	double getMedian_(std::vector<double> vec); // finds the median value of the elements in a vector
	   	float getSquareArea_(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3); // finds the area of a square 
	   	bool inObject_(cv::Point2f *corner, cv::Point2f point); // determines whether a point is in or outside a square
	   	double setPixelLength_(cv::Point2f *corners, double conf_dist); // finds the length and width of the area covered by a pixel

	   	cv::Mat src_, xyz_;
	   	double L_ = 0, W_ = 0, H_ = 0;
	   	const float PI_ = 3.14159265359;
	   	const int numOfPixels_ = 92928; //img.cols * img.rows;
		const float A_ = -2467.8, B_ = 1.6624, C_ = 0.7498;

    public:
        geometricalData();
        ~geometricalData();
        double getHeight(cv::Mat &xyz, cv::Mat &src, cv::Point2f *corners, double conf_dist);
	   	double* getLengthAndWidth(double *LW, cv::Point2f *corners, double conf_dist);
	   	double* getDimensions(cv::Mat &xyz, cv::Mat &src, double *LWH, cv::Point2f *corners, double conf_dist);
	   	double getOrientation(cv::Point2f *corners);

};

#endif