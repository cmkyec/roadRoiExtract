#include "roadRoiExtract.h"
#include <iostream>

int main()
{
	cv::Mat img = cv::imread("./roadImages/rain_4.png");
	cv::Mat roadImg;
	gentech::getRoadRoiImage(img, roadImg);

	gentech::lane leftLane, rightLane;
	gentech::getLeftAndRightLane(img, leftLane, rightLane);
	std::cout << "width: " << img.cols / 2 << std::endl;
	std::cout << (leftLane.m_bottom.x + rightLane.m_bottom.x) / 2 << std::endl;

	cv::namedWindow("road", 0);
	cv::imshow("road", roadImg);
	cv::waitKey(0);
	cv::destroyWindow("road");
	return 0;
}
