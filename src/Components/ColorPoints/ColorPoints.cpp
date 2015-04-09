/*!
 * \file
 * \brief
 * \author Aleksandra Karbarczyk
 */

#include <memory>
#include <string>

#include "ColorPoints.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace ColorPoints {

ColorPoints::ColorPoints(const std::string & name) :
		Base::Component(name), minBlue("minBlue"), maxBlue("maxBlue"),
		minGreen("minGreen"), maxGreen("maxGreen"),
		minYellow("minYellow"), maxYellow("maxYellow"),
		minRed("minRed"), maxRed("maxRed") {
			registerProperty(minBlue);
			registerProperty(maxBlue);
			registerProperty(minGreen);
			registerProperty(maxGreen);
			registerProperty(minYellow);
			registerProperty(maxYellow);
			registerProperty(minRed);
			registerProperty(maxRed);
}

ColorPoints::~ColorPoints() {
}

void ColorPoints::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_circles", &in_circles);
	registerStream("in_img", &in_img);
	registerStream("in_img_mask", &in_img_mask);
	registerStream("out_points", &out_points);
	// Register handlers
	h_onProcess.setup(boost::bind(&ColorPoints::onProcess, this));
	registerHandler("onProcess", &h_onProcess);
	addDependency("onProcess", &in_circles);
	addDependency("onProcess", &in_img);

	h_onProcessWithMask.setup(boost::bind(&ColorPoints::onProcessWithMask, this));
	registerHandler("onProcessWithMask", &h_onProcessWithMask);
	addDependency("onProcessWithMask", &in_circles);
	addDependency("onProcessWithMask", &in_img);
	addDependency("onProcessWithMask", &in_img_mask);
}

bool ColorPoints::onInit() {
	CLOG(LNOTICE)<<"ColorPoints init.";
	return true;
}

bool ColorPoints::onFinish() {
	return true;
}

bool ColorPoints::onStop() {
	return true;
}

bool ColorPoints::onStart() {
	return true;
}

void ColorPoints::onProcess() {
	const cv::Mat src = in_img.read();
	const Types::Circles circles = in_circles.read();
	CLOG(LDEBUG)<<"ColorPoints: onProcess started.";
	std::vector<Types::ColorPoint> result;
	float x, y, length;
	std::vector<cv::Vec3f>::const_iterator end_it = circles.circles.end();
	for(std::vector<cv::Vec3f>::const_iterator it = circles.circles.begin(); it != end_it; ++it)
	{
		x = (*it)[0]-(*it)[2];
		if(x < 0) x = 0;
		y = (*it)[1]-(*it)[2];
		if(y < 0) y = 0;
		length = 2*(*it)[2];
		if(length+x > src.cols) length = src.cols-x;
		if(length+y > src.rows) length = src.rows-y;
		cv::Rect circle_rect = cv::Rect(x, y, length, length);
		cv::Mat circle_img(src, circle_rect);
		Types::ColorPoint::Color color = findColor(circle_img);
		result.push_back(Types::ColorPoint(cv::Point((*it)[0], (*it)[1]), color));
	}
	CLOG(LDEBUG)<<"ColorPoints: onProcess finished.";
	out_points.write(result);
}

void ColorPoints::onProcessWithMask() {
	const cv::Mat src = in_img.read();
	const cv::Mat src_mask = in_img_mask.read();
	const Types::Circles circles = in_circles.read();
	CLOG(LDEBUG)<<"ColorPoints: onProcessWithMask started.";
	std::vector<Types::ColorPoint> result;
	float x, y, length;
	std::vector<cv::Vec3f>::const_iterator end_it = circles.circles.end();
	for(std::vector<cv::Vec3f>::const_iterator it = circles.circles.begin(); it != end_it; ++it)
	{
		x = (*it)[0]-(*it)[2];
		if(x < 0) x = 0;
		y = (*it)[1]-(*it)[2];
		if(y < 0) y = 0;
		length = 2*(*it)[2];
		if(length+x > src.cols) length = src.cols-x;
		if(length+y > src.rows) length = src.rows-y;
		cv::Rect circle_rect = cv::Rect(x, y, length, length);
		cv::Mat circle_img(src, circle_rect);
		cv::Mat circle_img_mask(src_mask, circle_rect);
		Types::ColorPoint::Color color = findColor(circle_img, circle_img_mask);
		result.push_back(Types::ColorPoint(cv::Point((*it)[0], (*it)[1]), color));
	}
	CLOG(LDEBUG)<<"ColorPoints: onProcessWithMask finished.";
	out_points.write(result);
}

Types::ColorPoint::Color ColorPoints::findColor(const cv::Mat& img)
{
	std::vector<cv::Mat> hsv_planes;
	split(img, hsv_planes);

	int histSizeH = 180;
	float rangeH[] = { 0, 180 } ;
	const float* histRangeH = { rangeH };
	bool uniform = true; bool accumulate = false;
	cv::Mat histH;

	calcHist(&hsv_planes[0], 1, 0, cv::Mat(), histH, 1, &histSizeH, &histRangeH, uniform, accumulate);

	float red_sum = 0, yellow_sum = 0, green_sum = 0, blue_sum = 0;

	for(int i=minRed; i<maxRed; ++i)
		red_sum += histH.at<float>(i);
	for(int i=minYellow; i<maxYellow; ++i)
		yellow_sum += histH.at<float>(i);
	for(int i=minGreen; i<maxGreen; ++i)
		green_sum += histH.at<float>(i);
	for(int i=minBlue; i<maxBlue; ++i)
		blue_sum += histH.at<float>(i);
	
	if(red_sum>yellow_sum && red_sum>green_sum && red_sum>blue_sum)
		return Types::ColorPoint::COLOR_RED;
	else if(yellow_sum>red_sum && yellow_sum>green_sum && yellow_sum>blue_sum)
		return Types::ColorPoint::COLOR_YELLOW;
	else if(green_sum>red_sum && green_sum>yellow_sum && green_sum>blue_sum)
		return Types::ColorPoint::COLOR_GREEN;
	else if(blue_sum>red_sum && blue_sum>yellow_sum && blue_sum>green_sum)
		return Types::ColorPoint::COLOR_BLUE;
	return Types::ColorPoint::COLOR_OTHER;
}

Types::ColorPoint::Color ColorPoints::findColor(const cv::Mat& img, const cv::Mat& img_mask)
{
	std::vector<cv::Mat> hsv_planes;
	split(img, hsv_planes);

	int histSizeH = 180;
	float rangeH[] = { 0, 180 } ;
	const float* histRangeH = { rangeH };
	bool uniform = true; bool accumulate = false;
	cv::Mat histH;

	calcHist(&hsv_planes[0], 1, 0, img_mask, histH, 1, &histSizeH, &histRangeH, uniform, accumulate);

	float red_sum = 0, yellow_sum = 0, green_sum = 0, blue_sum = 0;

	for(int i=minRed; i<maxRed; ++i)
		red_sum += histH.at<float>(i);
	for(int i=minYellow; i<maxYellow; ++i)
		yellow_sum += histH.at<float>(i);
	for(int i=minGreen; i<maxGreen; ++i)
		green_sum += histH.at<float>(i);
	for(int i=minBlue; i<maxBlue; ++i)
		blue_sum += histH.at<float>(i);
	
	if(red_sum>yellow_sum && red_sum>green_sum && red_sum>blue_sum)
		return Types::ColorPoint::COLOR_RED;
	else if(yellow_sum>red_sum && yellow_sum>green_sum && yellow_sum>blue_sum)
		return Types::ColorPoint::COLOR_YELLOW;
	else if(green_sum>red_sum && green_sum>yellow_sum && green_sum>blue_sum)
		return Types::ColorPoint::COLOR_GREEN;
	else if(blue_sum>red_sum && blue_sum>yellow_sum && blue_sum>green_sum)
		return Types::ColorPoint::COLOR_BLUE;
	return Types::ColorPoint::COLOR_OTHER;
}

} //: namespace ColorPoints
} //: namespace Processors
