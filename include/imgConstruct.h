#ifndef IMGCONSTRUCT_H
#define IMGCONSTRUCT_H

#include <iostream>
#include <opencv2/opencv.hpp>


class imgConstruct
{
	private:
		double getMedian_(std::vector<double> vec);
		double getXYZRec_(cv::Mat &xyz, double conf_dist);

		int width_ = 0;
		int height_ = 0;
		int x1_ = 0;
		int y1_ = 0;

    public:
        imgConstruct();
        ~imgConstruct();
      	void generateImage(cv::Mat &xyz, cv::Mat &conf, double *coeff, double conf_dist, double roi_width, double roi_height);


        cv::Mat xyz_rec;
        cv::Mat cloud_rec;
        
};

#endif