#include "imgConstruct.h"

imgConstruct::imgConstruct(){}

imgConstruct::~imgConstruct(){}

double imgConstruct::getMedian_(std::vector<double> vec) {

	double median = 0;
	int vecLen = vec.size();
	std::sort(vec.begin(), vec.end());

	if (vecLen % 2 > 0)
	{
		median = (vec[(vecLen+1)/2] + vec[(vecLen-1)/2])/2;	
	}
	else
	{
  		median = vec[vecLen/2];
	}
	return median;
}
		
double imgConstruct::getXYZRec_(cv::Mat &xyz, double conf_dist) {

	std::vector<double> vecXYZ_rec;
	for (int x = 0; x < width_; ++x)
    {
    	for (int y = 0; y < height_; ++y)
    	{
    		if(xyz.at<cv::Vec3s>(cv::Point(x+x1_, y+y1_))[0] > 0)
    		{
    			if(xyz.at<cv::Vec3s>(cv::Point(x+x1_, y+y1_))[0] + 20 < conf_dist)
    			{
    				vecXYZ_rec.push_back(xyz.at<cv::Vec3s>(cv::Point(x+x1_, y+y1_))[0]);
    			}
    		}
    	}
    }
    
    sort(vecXYZ_rec.begin(), vecXYZ_rec.end());

    std::vector<double> vecXYZ_rec_median;
    for (int i = int(vecXYZ_rec.size()/4); i < int(vecXYZ_rec.size()/1.25); ++i)
    {
    	vecXYZ_rec_median.push_back(vecXYZ_rec[i]);
    }

    double medianXYZ_rec = getMedian_(vecXYZ_rec_median);
    //std::cout << "medianXYZ_rec: " << medianXYZ_rec << std::endl;
    return medianXYZ_rec;
}

void imgConstruct::generateImage(cv::Mat &xyz, cv::Mat &conf, double *coeff, double conf_dist, double roi_width, double roi_height) {


	width_ = int(xyz.cols * roi_width);
	height_ = int(xyz.rows * roi_height);
	x1_ = int((xyz.cols - width_) / 2);
	y1_ = int((xyz.rows - height_) / 2);


	cv::Mat xyz_img = cv::Mat(height_, width_, CV_16SC3);
	cv::Mat cloud_img = cv::Mat(height_, width_, CV_8U);
	cv::Mat conf_img = cv::Mat(height_, width_, CV_8U);

	xyz_rec = cv::Mat::zeros(cv::Size(width_, height_), CV_16SC3);
	cloud_rec = cv::Mat::zeros(cv::Size(width_, height_), CV_8U);

    // This provides the rectification value to be inputted where there are invalid pixels
    double medianXYZ_rec = getXYZRec_(xyz, conf_dist);
    //int medianCloud_rec = coeff[0] * medianXYZ_rec + coeff[1]; 
    int medianCloud_rec = 0;

  	for (int x = 0; x < width_; x++) 
  	{
		for (int y = 0; y < height_; y++) 
		{		
			// This bit produces the grayscale image, which is used for object detection, based on the distance measurements
			if ((coeff[0] * xyz.at<cv::Vec3s>(cv::Point(x+x1_, y+y1_))[0] + coeff[1]) < 0) // (y = a * x + b) == (y = coeff[0] * x + coeff[1])
			{
				cloud_img.at<uchar>(cv::Point(x, y)) = 0;
			}
			else
			{
				cloud_img.at<uchar>(cv::Point(x, y)) = (coeff[0] * xyz.at<cv::Vec3s>(cv::Point(x+x1_, y+y1_))[0] + coeff[1]);
			}
			xyz_img.at<cv::Vec3s>(cv::Point(x, y)) = xyz.at<cv::Vec3s>(cv::Point(x+x1_, y+y1_));
			conf_img.at<uchar>(cv::Point(x, y)) = conf.at<uchar>(cv::Point(x+x1_, y+y1_));

			// This bit filters out invalid pixels
			if (cloud_img.at<uchar>(cv::Point(x, y)) < 255)
			{
				cloud_rec.at<uchar>(cv::Point(x, y)) = cloud_img.at<uchar>(cv::Point(x, y));
				xyz_rec.at<cv::Vec3s>(cv::Point(x, y)) = xyz_img.at<cv::Vec3s>(cv::Point(x, y));
			}
			if (conf_img.at<uchar>(cv::Point(x, y)) > 0) 
			{
				cloud_rec.at<uchar>(cv::Point(x, y)) = medianCloud_rec;
				xyz_rec.at<cv::Vec3s>(cv::Point(x, y))[0] = medianXYZ_rec;
			}
		}
	}
}




