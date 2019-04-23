#ifndef GEOMETRICALDATA_H
#define GEOMETRICALDATA_H

#include <iostream>
#include <opencv2/opencv.hpp>


class geometricalData
{
    private:
	   	double getMedian(std::vector<double> vec);
	   	float getSquareArea_(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3);
	   	bool inObject(cv::Point2f *corner, cv::Point2f point);

	   	std::vector<double> vecdist_;
	   	int pixelRatio_ = 0;
	   	double H_ = 0;
	   	double pixelLength_ = 0;
	   	double confDist_ = 0;
	   	int numOfPixels_ = 92928; //img.cols * img.rows;
		const float A_ = -2467.8, B_ = 1.6624, C_ = 0.7498;

    public:
        geometricalData();
        ~geometricalData();
        double getHeight(cv::Mat &xyz, cv::Mat &src, cv::Point2f *corners, double conf_dist);
        void setPixelLength(cv::Mat &xyz, cv::Mat &src, cv::Point2f *corners, double conf_dist);
	   	double* getLengthAndWidth(double *LW, cv::Point2f *corners, double pixelLength);
	   	double* getDimensions(cv::Mat &xyz, cv::Mat &src, double *LWH, cv::Point2f *corners, double conf_dist);

};

#endif