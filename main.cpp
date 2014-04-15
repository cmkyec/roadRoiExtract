#include "roadRoiExtract.h"
#include <iostream>

int main()
{
	cv::Mat img = cv::imread("./roadImages/rain_5.png");

	//cv::Mat roadImg;
	//gentech::getRoadRoiImage(img, roadImg);
	//cv::namedWindow("road", 0);
	//cv::imshow("road", roadImg);
	//cv::waitKey(0);
	//cv::destroyWindow("road");

	gentech::lane left, middle, right;
	gentech::getThreeLane(img, left, middle, right);
	cv::line(img, left.m_top, left.m_bottom, cv::Scalar(0, 255, 255), 2);
	cv::line(img, middle.m_top, middle.m_bottom, cv::Scalar(0, 255, 255), 2);
	cv::line(img, right.m_top, right.m_bottom, cv::Scalar(0, 255, 255), 2);
	cv::namedWindow("show", 0);
	cv::imshow("show", img);
	cv::waitKey(0);
	return 0;
}
