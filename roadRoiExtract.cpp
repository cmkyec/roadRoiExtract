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
 * If lane top is outside the image, it should be changed to the position intersect 
 * with the top border of the image.
 *
 * The lane bottom should be the position intersected with the bottom border of the image.
 * But if the lane bottom is outside the image, then it should be changed to the position
 * intersected with the left or right border of the image.
 */
void laneComplete(struct lane& lane, cv::Size imgSize)
{
	cv::Point top, bottom;
	top = lane.m_top;
	bottom = lane.m_bottom;
	// here bottom x may be larger than image width or smaller than zero.
	bottom.x = (bottom.x - top.x) * (imgSize.height - 1 - top.y) / (bottom.y - top.y) + top.x;
	bottom.y = imgSize.height - 1;
	if (top.y < 0) {
		top.x = top.x - (top.x - bottom.x) * top.y / (top.y - bottom.y);
		top.y = 0;
	}
	if (bottom.x > imgSize.width - 1) {
		bottom.y = top.y - (top.x - imgSize.width + 1) * (top.y - bottom.y) / (top.x - bottom.x);
		bottom.x = imgSize.width - 1;
	}
	if (bottom.x < 0) {
		bottom.y = top.y - (top.y - bottom.y) * top.x / (top.x - bottom.x);
		bottom.x = 0;
	}
	lane.m_top = top;
	lane.m_bottom = bottom;
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
inline void lineConvert(struct laneDetectorLine& innerLine,
			struct lane& outerLine)
{
	outerLine.m_top = innerLine.top;
	outerLine.m_bottom = innerLine.bottom;
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

	//cv::Mat tmp;
	//cameraImg.copyTo(tmp);
	//for (std::size_t i = 0; i < rawLines.size(); ++i) {
	//	cv::Point start(rawLines[i][0], rawLines[i][1]);
	//	cv::Point end(rawLines[i][2], rawLines[i][3]);
	//	cv::line(tmp, start, end, cv::Scalar(0, 255, 255), 2);
	//}

	std::vector<struct laneDetectorLine> lineFiltered;
	if (lineFilter(rawLines, cameraImg.size(), lineFiltered) == 0) return false;

	//cv::Mat tmp;
	//cameraImg.copyTo(tmp);
	//for (std::size_t i = 0; i < lineFiltered.size(); ++i) {
	//	cv::Point start = lineFiltered[i].top;
	//	cv::Point end = lineFiltered[i].bottom;
	//	cv::line(tmp, start, end, cv::Scalar(0, 255, 255), 2);
	//}
	
	struct laneDetectorLine left, right;
	getLeftAndRightLane(lineFiltered, left, right);

	lineConvert(left, leftLane);
	lineConvert(right, rightLane);

	laneComplete(leftLane, cameraImg.size());
	laneComplete(rightLane, cameraImg.size());

	return true;
}

// functions for get road roi region
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
		struct lane& leftLane, 
		struct lane& rightLane) 
{
	cv::Mat maskImg(srcImg.size(), CV_8UC1);
	maskImg.setTo(0);

	cv::line(maskImg, leftLane.m_top, leftLane.m_bottom, cv::Scalar::all(255), 1);
	cv::line(maskImg, rightLane.m_top, rightLane.m_bottom, cv::Scalar::all(255), 1);

	cv::Point seedPoint(maskImg.cols / 2, maskImg.rows / 2);
	cv::floodFill(maskImg, seedPoint, cv::Scalar::all(255));

	srcImg.copyTo(roadRoiImg, maskImg);
}

bool getRoadRoiImage(const cv::Mat& cameraImg,
		     cv::Mat& roadImg)
{
	struct lane leftLane, rightLane;
	if (!getLeftAndRightLane(cameraImg, leftLane, rightLane)) {
		return false;
	}
	getRoadRoi(cameraImg, roadImg, leftLane, rightLane);
	return true;
}

// functions for get middle lane
inline void getMarkerPoint(const lane& la, int imgWidth, cv::Point& p)
{
	if (la.m_bottom.x == 0 || la.m_bottom.x == imgWidth - 1) {
		p = la.m_bottom;
	} else {
		p.x = (la.m_top.x + la.m_bottom.x) / 2;
		p.y = (la.m_top.y + la.m_bottom.y) / 2;
	}
}

void markerPointAdjust(cv::Point& left, cv::Point& right,
	               const cv::Mat& roadRoiImg)
{
	if (left.y == right.y) return;
	if (left.y > right.y) {
		int x = 0;
		const cv::Vec3b* p = roadRoiImg.ptr<cv::Vec3b>(right.y);
		for (; x < roadRoiImg.cols - 1; ++x) {
			if (p[x] != cv::Vec3b(0, 0, 0)) break;
		}
		left.x = x;
		left.y = right.y;
	} else {
		int x = roadRoiImg.cols - 1;
		const cv::Vec3b* p = roadRoiImg.ptr<cv::Vec3b>(left.y);
		for (; x > 0; --x) {
			if (p[x] != cv::Vec3b(0, 0, 0)) break;
		}
		right.x = x;
		right.y = left.y;
	}
}

void getMarkerImage(cv::Mat& roadRoiImg,
		    const lane& leftLane,
		    const lane& rightLane,
		    cv::Mat& markerImg)
{
	cv::Point leftMarkerPoint, rightMarkerPoint;
	getMarkerPoint(leftLane, roadRoiImg.cols, leftMarkerPoint);
	getMarkerPoint(rightLane, roadRoiImg.cols, rightMarkerPoint);
	markerPointAdjust(leftMarkerPoint, rightMarkerPoint, roadRoiImg);

	int r_b_x = rightLane.m_bottom.x, l_b_x = leftLane.m_bottom.x;
	int w_half = roadRoiImg.cols / 2;
	int markerPointGap = rightMarkerPoint.x - leftMarkerPoint.x + 1;
	int leftMarkerLen = markerPointGap / 2 *
		std::abs(r_b_x - w_half) / (std::abs(r_b_x - w_half) + std::abs(l_b_x - w_half));
	int rightMarkerLen = markerPointGap / 2 *
		std::abs(l_b_x - w_half) / (std::abs(r_b_x - w_half) + std::abs(l_b_x - w_half));
	// marker length should be in the range [1/5, 1/3].  // to be improved.
	if (leftMarkerLen < markerPointGap / 5) leftMarkerLen = markerPointGap / 5;
	if (rightMarkerLen < markerPointGap / 5) rightMarkerLen = markerPointGap / 5;
	if (leftMarkerLen > markerPointGap / 3) leftMarkerLen = markerPointGap / 3;
	if (rightMarkerLen > markerPointGap / 3) rightMarkerLen = markerPointGap / 3;
	
	markerImg.create(roadRoiImg.size(), CV_32S);
	markerImg.setTo(0);
	for (int r = 0; r < 10; ++r) {  // marker region height set to 10 pixels
		for (int c = 0; c < leftMarkerLen; ++c) {
			cv::Point pos(leftMarkerPoint.x + c, leftMarkerPoint.y + r);
			if (pos.x > markerImg.cols - 1) pos.x = markerImg.cols - 1;
			if (pos.y > markerImg.rows - 1) pos.y = markerImg.rows - 1;
			if (roadRoiImg.at<cv::Vec3b>(pos) != cv::Vec3b(0, 0, 0)) {
				markerImg.at<int>(pos) = 1;
				//roadRoiImg.at<cv::Vec3b>(pos) = cv::Vec3b(0, 255, 255);
			}
		}
		for (int c = 0; c < rightMarkerLen; ++c) {
			cv::Point pos(rightMarkerPoint.x - c, rightMarkerPoint.y + r);
			if (pos.x < 0) pos.x = 0;
			if (pos.y > markerImg.rows - 1) pos.y = markerImg.rows - 1;
			if (roadRoiImg.at<cv::Vec3b>(pos) != cv::Vec3b(0, 0, 0)) {
				markerImg.at<int>(pos) = 2;
				//roadRoiImg.at<cv::Vec3b>(pos) = cv::Vec3b(0, 255, 255);
			}
		}
	}
}

bool getMiddleLane(const cv::Mat& roadRoiImage,
		   cv::Mat& markerImg, 
		   cv::Point& middleLaneBottom)
{
	cv::watershed(roadRoiImage, markerImg);

	// get the watershed boundary
	cv::Mat maskImg(roadRoiImage.size(), CV_8UC1);
	maskImg.setTo(0);
	// here start from 5, remove the boundary of the image
	for (int r = 5; r < markerImg.rows - 5; ++r) {
		for (int c = 5; c < markerImg.cols - 5; ++c) {
			if (markerImg.at<int>(r, c) == -1 &&
				roadRoiImage.at<cv::Vec3b>(r, c) != cv::Vec3b(0, 0, 0) &&
				roadRoiImage.at<cv::Vec3b>(r, c - 5) != cv::Vec3b(0, 0, 0) &&
				roadRoiImage.at<cv::Vec3b>(r, c + 5) != cv::Vec3b(0, 0, 0)) {
				maskImg.at<unsigned char>(r, c) = 255;
			}
		}
	}

	int houghThreshold = 70;
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(maskImg, lines, 1, CV_PI / 180, houghThreshold, 10, 10);
	if (lines.size() == 0) return false;

	// extract the longest line
	cv::Vec4i lane = lines[0];
	int length = (lane[0] - lane[2]) * (lane[0] - lane[2]) + (lane[1] - lane[3]) * (lane[1] - lane[3]);
	for (std::size_t i = 1; i < lines.size(); ++i) {
		int l = (lines[i][0] - lines[i][2]) * (lines[i][0] - lines[i][2]) +
			(lines[i][1] - lines[i][3]) * (lines[i][1] - lines[i][3]);
		if (l > length) {
			lane = lines[i];
			length = l;
		}
	}

	cv::Point a(lane[0], lane[1]), b(lane[2], lane[3]);
	if (a.y > b.y) middleLaneBottom = a;
	else
		middleLaneBottom = b;
	return true;
}

bool getThreeLane(const cv::Mat& cameraImg,
		  struct lane& leftLane,
		  struct lane& middleLane,
		  struct lane& rightLane)
{
	if (!getLeftAndRightLane(cameraImg, leftLane, rightLane)) {
		return false;
	}
	cv::Mat roadRoiImage;
	getRoadRoi(cameraImg, roadRoiImage, leftLane, rightLane);

	cv::Mat markerImg;  // marker image for watershed algorithm
	getMarkerImage(roadRoiImage, leftLane, rightLane, markerImg);

	cv::Point middleLaneBottom;
	getMiddleLane(roadRoiImage, markerImg, middleLaneBottom);

	cv::Point middleLaneTop;
	if (leftLane.m_top == rightLane.m_top) {
		middleLaneTop = leftLane.m_top;
	} else {
		middleLaneTop.x = (leftLane.m_top.x + rightLane.m_top.x) / 2;
		middleLaneTop.y = 0;
	}
	middleLane.m_top = middleLaneTop;
	middleLane.m_bottom = middleLaneBottom;
	laneComplete(middleLane, cameraImg.size());

	return true;
}

}
