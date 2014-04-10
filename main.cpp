#include "roadRoiExtract.h"
#include <iostream>

int main()
{
	cv::Mat img = cv::imread("./roadImages/rain_4.png");
	cv::Mat roadImg;
	gentech::getRoadRoiImage(img, roadImg);

	cv::namedWindow("road", 0);
	cv::imshow("road", roadImg);
	cv::waitKey(0);
	cv::destroyWindow("road");
	return 0;
}
