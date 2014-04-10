#ifndef _ROAD_ROI_EXTRACT_H_
#define _ROAD_ROI_EXTRACT_H_

#include <opencv2/opencv.hpp>

namespace gentech
{

struct lane
{
	cv::Point m_top;
	cv::Point m_bottom;
};

/**
 * detect the left lane and right lane of the road in the cameraImg.
 *
 * @param roadImg the original road image
 * @param leftLane the detected left lane of the road image
 * @param rightLane the detected right lane of the road image
 */
bool getLeftAndRightLane(const cv::Mat& cameraImg, 
		         struct lane& leftLane, 
			 struct lane& rightLane);

/**
 * extract the road roi image from the original camera image.
 */
bool getRoadRoiImage(const cv::Mat& cameraImg,
		     cv::Mat& roadImg);


}

#endif /* _ROAD_ROI_EXTRACT_H_ */
