#ifndef IMGPROCESS_H
#define IMGPROCESS_H

#include <iostream>
#include <opencv2/opencv.hpp>


class imgProcess
{

    public:
        imgProcess();
        ~imgProcess();
        void grayScale(cv::Mat &src, cv::Mat &dst);
        void threshold(cv::Mat &src, cv::Mat &dst, int threshold);
        void morphing(cv::Mat &src, cv::Mat &dst, int closeE);
        cv::RotatedRect getControur(cv::Mat &img);
        void drawRect(cv::Mat &src, cv::Mat &dst, cv::Point2f *corners, cv::RotatedRect boundingBox);

};

#endif