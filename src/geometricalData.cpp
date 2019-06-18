#include "geometricalData.h"

geometricalData::geometricalData(){}

geometricalData::~geometricalData(){}


double geometricalData::getMedian_(std::vector<double> vec) {

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

float geometricalData::getSquareArea_(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3) {

	return fabs((p1.x*(p2.y-p3.y) + p2.x*(p3.y-p1.y) + p3.x*(p1.y-p2.y)));
}

bool geometricalData::inObject_(cv::Point2f *corner, cv::Point2f point) {

	float A = getSquareArea_(corner[0], corner[1], corner[2]);
	float A1 = getSquareArea_(point, corner[1], corner[2]);
	float A2 = getSquareArea_(corner[0], point, corner[2]);
	float A3 = getSquareArea_(corner[0], corner[1], point);

	return (int(A) == int(A1+A2+A3));
}

double geometricalData::getHeight(cv::Mat &xyz, cv::Mat &src, cv::Point2f *corners, double conf_dist) {

	conf_dist = conf_dist;
	std::vector<double> vecdist = {0};

	for (int x = 0; x < xyz.cols; x++) 
  	{
		for (int y = 0; y < xyz.rows; y++) 
		{	
			if (inObject_(corners, cv::Point2f(x,y)))
			{
				if ((conf_dist - xyz.at<cv::Vec3s>(cv::Point(x, y))[0]) > 5)
				{
					vecdist.push_back(conf_dist - xyz.at<cv::Vec3s>(cv::Point(x, y))[0]);
				}
			}
		}
	}

	// finds the median height from amoungest the top two percent heightest points 
	std::vector<double> temp_vecdist;	
	sort(vecdist.begin(), vecdist.end());
	for (int i = (vecdist.size() * 0.98); i < vecdist.size(); ++i)
	{
		temp_vecdist.push_back(vecdist[i]);
	}

	H_ = getMedian_(temp_vecdist) +5; // +5 = the offset placed on the config dist value

	return H_;
}

double geometricalData::setPixelLength_(cv::Point2f *corners, double conf_dist) {

	double H = getHeight(xyz_, src_, corners, conf_dist);

	float pixelArea = 0;	

	pixelArea = (A_ + B_ * (conf_dist - (H)) + float(C_ * pow((conf_dist - (H)), 2))) / double(numOfPixels_); // pixelArea = (A + Bx + Cx²) / numOfPixels

	double pixelLength = sqrt(pixelArea);
	return pixelLength;
}

double* geometricalData::getLengthAndWidth(double *LW, cv::Point2f *corners, double conf_dist) {

	double pixelLength = setPixelLength_(corners, conf_dist);

	double hyp[3], hos = 0, mod = 0;
	for (int i = 0; i < 3; i++)
	{
		mod = int(corners[1 + i].y) - int(corners[0 + i].y) ;
		hos = int(corners[1 + i].x) - int(corners[0 + i].x) ;
		hyp[i] = sqrt(pow(mod, 2) + pow(hos, 2));	// -2 = boundary box offset
	}
	int hypLen = sizeof(hyp)/sizeof(*hyp);
	std::sort(hyp, hyp + hypLen);

	if (hyp[0] > hyp[2])
	{
		L_ = hyp[0] * pixelLength; // length
		W_ = hyp[2] * pixelLength; // width
	}
	else
	{
		L_ = hyp[2] * pixelLength; // length
		W_ = hyp[0] * pixelLength; // width
	}

	LW[0] = L_;
	LW[1] = W_;

	return LW;
}

double* geometricalData::getDimensions(cv::Mat &xyz, cv::Mat &src, double *LWH, cv::Point2f *corners, double conf_dist) {

	src_ = src;
	xyz_ = xyz;


	double LW[2];
	double* L_W = getLengthAndWidth(LW, corners, conf_dist);

	LWH[0] = L_W[0];
	LWH[1] = L_W[1];
	LWH[2] = H_;

	return LWH;
}

double geometricalData::getOrientation(cv::Point2f *corners) {

	int tempMod = 0, tempHos = 0; 
	double hyp = 0, hos = 0, mod = 0, tempHyp = 0;

	for (size_t i = 0; i < 3; i++)
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

	double theta = (atan(mod/hos) * 180 / PI_);
	return theta;
} 