#include "roadRoiExtract.h"
#include <iostream>

int main()
{
	cv::Mat img = cv::imread("./roadImages/rain_5.png");
	cv::Mat roadImg;
	gentech::getRoadRoiImage(img, roadImg);

	gentech::lane leftLane, rightLane;
	gentech::getLeftAndRightLane(img, leftLane, rightLane);
	std::cout << "left: " << img.cols/2 - leftLane.m_bottom.x << std::endl;
	std::cout << "right: " << rightLane.m_bottom.x - img.cols/2 << std::endl;

	cv::namedWindow("road", 0);
	cv::imshow("road", roadImg);
	cv::waitKey(0);
	cv::destroyWindow("road");
	return 0;
}
