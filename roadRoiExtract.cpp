#include "MSAC.h"
#include "roadRoiExtract.h"
#include <iostream>

namespace gentech
{

/**
 * used internal for finding the left and right lane.
 */ 
struct laneDetectorLine
{
	cv::Point top;
	cv::Point bottom;
	double angle;
};

bool laneDetectorLineCompare(const struct laneDetectorLine& a, 
		             const struct laneDetectorLine& b)
{
	return a.angle < b.angle;
}

/**
 * outstand the underlying lines in the road image.
 *
 * @param[in] srcImg original road image
 * @param[in, out] dstGray the gray image, which outstand the underlying lines in the road image
 * @param[in] lineMarkingWidth the width of the line marking in the road, 
                               which depends on the actual image
 */
void getLineCandidatesImg(const cv::Mat& srcImg, cv::Mat& dstGray, int laneMarkingWidth = 10)
{
	cv::Mat srcGray;
	if (srcImg.channels() == 3) cv::cvtColor(srcImg, srcGray, CV_BGR2GRAY);
	else srcImg.copyTo(srcGray);

 	dstGray.create(srcGray.size(), srcGray.type());
	dstGray.setTo(0);

	for (int r = 0; r < srcGray.rows; ++r) {
		unsigned char* pRowSrc = srcGray.ptr<unsigned char>(r);
		unsigned char* pRowDst = dstGray.ptr<unsigned char>(r);
		for (int c = laneMarkingWidth; c < dstGray.cols - laneMarkingWidth; ++c) {
			int tmp = 0;
			if (pRowSrc[c] != 0) {
				tmp += 2 * pRowSrc[c];
				tmp -= pRowSrc[c - laneMarkingWidth];
				tmp -= pRowSrc[c + laneMarkingWidth];
				tmp -= std::abs((int)(pRowSrc[c - laneMarkingWidth] - pRowSrc[c + laneMarkingWidth]));
				pRowDst[c] = cv::saturate_cast<unsigned char>(tmp);
			}
		}
	}
	
	cv::threshold(dstGray, dstGray, 0, 255, CV_THRESH_OTSU);
}

/**
 * detect the lines in the image by hough transform.
 *
 * @param[in] img the input image
 * @param[in, out] lines the detected lines in the image
 */
#define MAX_NUM_LINES (200)
void lineDetector(cv::Mat& img, std::vector<cv::Vec4i>& lines)
{
	CV_Assert(img.channels() == 1);

	int houghThreshold = 70;
	std::vector<cv::Vec4i> linesTmp;
	cv::HoughLinesP(img, linesTmp, 1, CV_PI/180, houghThreshold, 20, 10);
	while (linesTmp.size() > MAX_NUM_LINES) {
		houghThreshold += 10;
		linesTmp.clear();
		cv::HoughLinesP(img, linesTmp, 1, CV_PI/180, houghThreshold, 20, 10);
	}
	
	lines.clear();
	for (std::size_t i = 0; i < linesTmp.size(); ++i) {
		// remove too horizontal lines
		if (std::abs(linesTmp[i][1] - linesTmp[i][3]) < 10) continue;
		// remove too vertical lines, the middle line would be detected in the following procedure,
		// it does not matter if the middle line is removed by this process.
		// the left or right lanes may be too vertical in some situations, so threshold value is 5 not 10.
		if (std::abs(linesTmp[i][0] - linesTmp[i][2]) < 5) continue;
		lines.push_back(linesTmp[i]);
	}
}

/**
 * There are outliers in the lines detected by hough transform.
 * By the paper:Non-Linear optimization for robust estimation of vanishing point
 * we can filter out the outliers.
 *
 * @param[in] lines the lines detected by hough transform
 * @param[in] imgSize the size of image in which the line is detected
 * @param[in, out] lineFiltered the remaining lines after filtering
 *
 * @return return 1 if succeed, return 0 if failed
 *
 * note:
 * if return 0, the data in the lineFiltered should not be used.
 */ 
int lineFilter(std::vector<cv::Vec4i>& lines, 
	       cv::Size imgSize,
	       std::vector<struct laneDetectorLine>& lineFiltered) 
{
	// Call msac function for multiple vanishing point estimation
	std::vector<std::vector<cv::Point> > lineSegments;
	for (std::size_t i = 0; i < lines.size(); ++i) {
		std::vector<cv::Point> aux;
		aux.push_back(cv::Point(lines[i][0], lines[i][1]));
		aux.push_back(cv::Point(lines[i][2], lines[i][3]));
		lineSegments.push_back(aux);
	}
	std::vector<cv::Mat> vps;			
	std::vector<int> numInliers;
	std::vector<std::vector<std::vector<cv::Point> > > lineSegmentsClusters;
	MSAC msac;
	msac.init(MODE_NIETO, imgSize);
	msac.multipleVPEstimation(lineSegments, lineSegmentsClusters, numInliers, vps, 1); 

	if (vps.size() <= 0 || vps[0].at<float>(2, 0) == 0) return 0;

	// There is only one vanishing point in our application.
	cv::Point vanishingPoint;
	vanishingPoint.x = (int)vps[0].at<float>(0, 0);
	vanishingPoint.y = (int)vps[0].at<float>(1, 0);
	std::vector<std::vector<cv::Point> >& lineCluster = lineSegmentsClusters[0];
	for (std::size_t i = 0; i < lineCluster.size(); ++i) {
		struct laneDetectorLine line;
		line.top = vanishingPoint;
		if (lineCluster[i][0].y > lineCluster[i][1].y)
			line.bottom = lineCluster[i][0];
		else
			line.bottom = lineCluster[i][1];
		line.angle = atan((line.top.y - line.bottom.y) * 1.0 / (line.top.x - line.bottom.x));
		lineFiltered.push_back(line);
	}
	return 1;
}

/**
 * get the left and right road lane according the angles.
 */
void getLeftAndRightLane(std::vector<struct laneDetectorLine>& lines,
		         struct laneDetectorLine& leftLane,
			 struct laneDetectorLine& rightLane)
{
	std::sort(lines.begin(), lines.end(), laneDetectorLineCompare);

	if (lines[0].angle >= 0 || lines[lines.size() - 1].angle <= 0) {
		leftLane = lines[lines.size() - 1];
		rightLane = lines[0];
	} else {
		for (std::size_t i = 0; i < lines.size() - 1; ++i) {
			if (lines[i].angle < 0 && lines[i+1].angle > 0) {
				leftLane = lines[i];
				rightLane = lines[i+1];
				break;
			}
		}
	}
}

/**
 * convert struct laneDetectorLine to struct lane. 
 */
void lineConvert(struct laneDetectorLine& innerLine,
		 struct lane& outerLine,
		 const cv::Size& imgSize)
{
	cv::Point top, bottom;
	top = innerLine.top;
	bottom = innerLine.bottom;
	// here bottom x may be larger than image width or smaller than zero.
	bottom.x = (bottom.x - top.x) * (imgSize.height - 1 - top.y) / (bottom.y - top.y) + top.x;
	bottom.y = imgSize.height - 1;
	if (top.y < 0) {
		top.x = top.x - (top.x - bottom.x) * top.y / (top.y - bottom.y);
		top.y = 0;
	}
	if (bottom.x > imgSize.width - 1) {
		bottom.x = imgSize.width - 1;
		bottom.y = top.y - (top.x - bottom.x) * (top.y - bottom.y) / (top.x - bottom.y);
	}
	if (bottom.x < 0) {
		bottom.x = 0;
		bottom.y = top.y - (top.y - bottom.y) * top.x / (top.x - bottom.x);
	}
	outerLine.m_top = top;
	outerLine.m_bottom = bottom;
}

bool getLeftAndRightLane(const cv::Mat& cameraImg, 
		         struct lane& leftLane, 
			 struct lane& rightLane)
{
	cv::Mat lineCandidateImg;
	getLineCandidatesImg(cameraImg, lineCandidateImg);

	std::vector<cv::Vec4i> rawLines;
	lineDetector(lineCandidateImg, rawLines);
	if (rawLines.size() < 3) return false;

	std::vector<struct laneDetectorLine> lineFiltered;
	if (lineFilter(rawLines, cameraImg.size(), lineFiltered) == 0) return false;
	
	struct laneDetectorLine left, right;
	getLeftAndRightLane(lineFiltered, left, right);

	lineConvert(left, leftLane, cameraImg.size());
	lineConvert(right, rightLane, cameraImg.size());

	return false;
}

/**
 * The image from the camera always contains some regions do not belong to the road.
 * For our event detector, we are only interested in the road region.
 *
 * @param[in] srcImg the original image from the camera
 * @param[in, out] roadRoiImg the road region that we are interested in
 * @param[in] leftLane the left lane of the road
 * @param[out] rightLane the right lane of the road
 */
void getRoadRoi(const cv::Mat& srcImg, cv::Mat& roadRoiImg, 
		struct laneDetectorLine leftLane, 
		struct laneDetectorLine rightLane) 
{
	cv::Mat maskImg(srcImg.size(), CV_8UC1);
	maskImg.setTo(0);

	// get the bottom intersection of the left lane and the image boundary
	cv::Point top, bottom;
	top = leftLane.top;
	bottom = leftLane.bottom;
	bottom.x = (bottom.x - top.x) * (maskImg.rows - 1 - top.y) / (bottom.y - top.y) + top.x;
	bottom.y = maskImg.rows - 1;
	cv::line(maskImg, top, bottom, cv::Scalar::all(255), 1);

	// get the bottom intersection of the rightlane and the image boundary
	top = rightLane.top;
	bottom = rightLane.bottom;
	bottom.x = (bottom.x - top.x) * (maskImg.rows - 1 - top.y) / (bottom.y - top.y) + top.x;
	bottom.y = maskImg.rows - 1;
	cv::line(maskImg, top, bottom, cv::Scalar::all(255), 1);

	cv::Point seedPoint(maskImg.cols / 2, maskImg.rows / 2);
	cv::floodFill(maskImg, seedPoint, cv::Scalar::all(255));

	srcImg.copyTo(roadRoiImg, maskImg);
}

bool getRoadRoiImage(const cv::Mat& cameraImg,
		     cv::Mat& roadImg)
{
	cv::Mat lineCandidateImg;
	getLineCandidatesImg(cameraImg, lineCandidateImg);

	std::vector<cv::Vec4i> rawLines;
	lineDetector(lineCandidateImg, rawLines);
	if (rawLines.size() < 3) return false;

	std::vector<struct laneDetectorLine> lineFiltered;
	if (lineFilter(rawLines, cameraImg.size(), lineFiltered) == 0) return false;
	
	struct laneDetectorLine left, right;
	getLeftAndRightLane(lineFiltered, left, right);

	getRoadRoi(cameraImg, roadImg, left, right);
	return true;
}


}
