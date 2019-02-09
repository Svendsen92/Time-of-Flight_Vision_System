void grayScale(cv::Mat &src, cv::Mat &dst) {
	cv::Mat temp = cv::Mat(src.rows, src.cols, CV_8U);

	for (int x = 0; x < src.cols; x++) {
		for (int y = 0; y < src.rows; y++) {
			temp.at<uchar>(cv::Point(x, y)) = (src.at<cv::Vec3b>(cv::Point(x, y))[0] + src.at<cv::Vec3b>(cv::Point(x, y))[1] + src.at<cv::Vec3b>(cv::Point(x, y))[2]) / 3;
		}
	}
	dst = temp;
}

void imgSubstractGray(cv::Mat &src, cv::Mat &dst) {

	for (int x = 0; x < src.cols; x++) {
		for (int y = 0; y < src.rows; y++) {
			if (dst.at<uchar>(cv::Point(x, y)) + 5 > src.at<uchar>(cv::Point(x, y)) && dst.at<uchar>(cv::Point(x, y)) - 5 < src.at<uchar>(cv::Point(x, y))) {
				dst.at<uchar>(cv::Point(x, y)) = 0;
			}
		}
	}
}

void imgSubstractColor(cv::Mat &src, cv::Mat &dst) {
	cv::Mat temp = dst;

	for (int x = 0; x < src.cols; x++) {
		for (int y = 0; y < src.rows; y++) {
			if (temp.at<cv::Vec3b>(cv::Point(x, y))[0] + 5 > src.at<cv::Vec3b>(cv::Point(x, y))[0] && temp.at<cv::Vec3b>(cv::Point(x, y))[0] - 5 < src.at<cv::Vec3b>(cv::Point(x, y))[0]) {
				temp.at<cv::Vec3b>(cv::Point(x, y))[0] = 0;
			}
			if (temp.at<cv::Vec3b>(cv::Point(x, y))[1] + 5 > src.at<cv::Vec3b>(cv::Point(x, y))[1] && temp.at<cv::Vec3b>(cv::Point(x, y))[1] - 5 < src.at<cv::Vec3b>(cv::Point(x, y))[1]) {
				temp.at<cv::Vec3b>(cv::Point(x, y))[1] = 0;
			}
			if (temp.at<cv::Vec3b>(cv::Point(x, y))[2] + 5 > src.at<cv::Vec3b>(cv::Point(x, y))[2] && temp.at<cv::Vec3b>(cv::Point(x, y))[2] - 5 < src.at<cv::Vec3b>(cv::Point(x, y))[2]) {
				temp.at<cv::Vec3b>(cv::Point(x, y))[2] = 0;
			}
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

void hit(cv::Mat &src, cv::Mat &dst) {


}

cv::RotatedRect getControur(cv::Mat &img) {

	std::vector<std::vector<cv::Point>> contours;
	findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	int biggestContourIdx = -1;
	float biggestContourArea = 0;
	for (int i = 0; i < contours.size(); i++) {
		drawContours(img, contours, i, cv::Scalar(0, 0, 255), 1, cv::LINE_8);

		double ctArea = cv::contourArea(contours[i]);
		if (ctArea > biggestContourArea)
		{
			biggestContourArea = ctArea;
			biggestContourIdx = i;
		}
	}
	cv::RotatedRect boundingBox = cv::minAreaRect(contours[biggestContourIdx]);
	return boundingBox;
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


int main()
{
	cv::Mat img, gray, grayHist, morph, morphDrawn, bagImg, subImg, binary;
	double theta;
	img = cv::imread("C:\\Users\\steff\\Desktop\\6th Semester\\P6 - Project\\Images\\cat_sponge_90.jpg");
	bagImg = cv::imread("C:\\Users\\steff\\Desktop\\6th Semester\\P6 - Project\\Images\\cat.jpg");

	cv::resize(img, img, cv::Size(), 0.25, 0.25);
	cv::resize(bagImg, bagImg, cv::Size(), 0.25, 0.25);
	//std::cout << "size: " << img.size() << std::endl;
	grayScale(img, subImg);
	grayScale(bagImg, bagImg);

	auto start = std::chrono::high_resolution_clock::now();
	
	grayScale(img, gray);
	imgSubstractGray(bagImg, gray);
	//imgSubstractColor(bagImg, img);
	//grayScale(img, gray);
	threshold(gray, binary, 5);
	morphing(binary, morph, 3, 1);
	
	cv::RotatedRect boundingBox = getControur(morph);

	// draw the rotated rect
	cv::Point2f corners[4];
	boundingBox.points(corners);
	drawRect(img, img, corners, boundingBox);
	theta = getRotateAngle(corners);

	auto elapsed = std::chrono::high_resolution_clock::now() - start;
	long long milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
	std::cout << "time elapsed: " << milliseconds << "ms" << std::endl;
	std::cout << "theta: " << theta << std::endl;


	imshow("Image", img);
	//imshow("Bagground Image", bagImg);
	imshow("Image binary", binary);
	imshow("Image morph", morph);
	//imshow("morphDrawn", morphDrawn);
	cv::waitKey(0);

	return(0);
}
