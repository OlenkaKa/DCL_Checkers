/*!
 * \file
 * \brief
 * \author Aleksandra Karbarczyk
 */

#include <memory>
#include <string>

#include "GenerateCheckers.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace GenerateCheckers {

GenerateCheckers::GenerateCheckers(const std::string & name) :
		Base::Component(name)  {

}

GenerateCheckers::~GenerateCheckers() {
}

void GenerateCheckers::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_circles", &in_circles);
	registerStream("in_img", &in_img);
	registerStream("in_img_map", &in_img_map);
	registerStream("out_circles", &out_circles);
	// Register handlers
	h_onProcess.setup(boost::bind(&GenerateCheckers::onProcess, this));
	registerHandler("onProcess", &h_onProcess);
	addDependency("onProcess", &in_circles);
	addDependency("onProcess", &in_img);

	h_onProcessWithMap.setup(boost::bind(&GenerateCheckers::onProcessWithMap, this));
	registerHandler("onProcessWithMap", &h_onProcessWithMap);
	addDependency("onProcessWithMap", &in_circles);
	addDependency("onProcessWithMap", &in_img);
	addDependency("onProcessWithMap", &in_img_map);
}

bool GenerateCheckers::onInit() {
	CLOG(LERROR)<<"GenerateCheckers init";
	return true;
}

bool GenerateCheckers::onFinish() {
	return true;
}

bool GenerateCheckers::onStop() {
	return true;
}

bool GenerateCheckers::onStart() {
	return true;
}

void GenerateCheckers::onProcess() {
	const cv::Mat src = in_img.read();
	const Types::Circles circles = in_circles.read();
	CLOG(LERROR)<<"Start onProcess in GenerateCheckers";
	std::vector<Types::ColorCircle> result;
	float x, y, length;
	std::vector<Vec3f>::const_iterator end_it = circles.circles.end();
	for(std::vector<Vec3f>::const_iterator it = circles.circles.begin(); it != end_it; ++it)
	{
		x = (*it)[0]-(*it)[2];
		y = (*it)[1]-(*it)[2];
		length = 2*(*it)[2];
		cv::Rect circle_rect = cv::Rect(x, y, length, length);
		cv::Mat circle_img(src, circle_rect);
		Types::ColorCircle::Color color = findColor(circle_img);
		result.push_back(Types::ColorCircle(cv::Point((*it)[0], (*it)[1]), color));
	}
	CLOG(LERROR)<<"End onProcess in GenerateCheckers";
	out_circles.write(result);
}

void GenerateCheckers::onProcessWithMap() {
	const cv::Mat src = in_img.read();
	const cv::Mat src_map = in_img_map.read();
	const Types::Circles circles = in_circles.read();
	CLOG(LERROR)<<"Start onProcess with map in GenerateCheckers";
	std::vector<Types::ColorCircle> result;
	float x, y, length;
	std::vector<Vec3f>::const_iterator end_it = circles.circles.end();
	for(std::vector<Vec3f>::const_iterator it = circles.circles.begin(); it != end_it; ++it)
	{
		x = (*it)[0]-(*it)[2];
		y = (*it)[1]-(*it)[2];
		length = 2*(*it)[2];
		cv::Rect circle_rect = cv::Rect(x, y, length, length);
		cv::Mat circle_img(src, circle_rect);
		cv::Mat circle_img_map(src_map, circle_rect);
		Types::ColorCircle::Color color = findColor(circle_img, circle_img_map);
		result.push_back(Types::ColorCircle(cv::Point((*it)[0], (*it)[1]), color));
	}
	CLOG(LERROR)<<"End onProcess with map in GenerateCheckers";
	out_circles.write(result);
}

Types::ColorCircle::Color GenerateCheckers::findColor(const cv::Mat& img)
{
	// TODO
	std::vector<cv::Mat> hsv_planes;
	split(img, hsv_planes);

	int histSizeH = 180;
	float rangeH[] = { 0, 180 } ;
	const float* histRangeH = { rangeH };
	bool uniform = true; bool accumulate = false;
	cv::Mat histH;

	calcHist(&hsv_planes[0], 1, 0, cv::Mat(), histH, 1, &histSizeH, &histRangeH, uniform, accumulate);

	float red_sum = 0, yellow_sum = 0, green_sum = 0, blue_sum = 0;

	CLOG(LERROR)<<"Matrix size: "<<histH.size().height<<" "<<histH.size().width<<" Matrix type: "<<histH.type();

	for(int i=0; i<7; ++i)
		red_sum += histH.at<float>(i);
	for(int i=20; i<28; ++i)
		yellow_sum += histH.at<float>(i);
	for(int i=91; i<98; ++i)
		green_sum += histH.at<float>(i);
	for(int i=107; i<113; ++i)
		blue_sum += histH.at<float>(i);
	
	if(red_sum>yellow_sum && red_sum>green_sum && red_sum>blue_sum)
		return Types::ColorCircle::COLOR_RED;
	else if(yellow_sum>red_sum && yellow_sum>green_sum && yellow_sum>blue_sum)
		return Types::ColorCircle::COLOR_YELLOW;
	else if(green_sum>red_sum && green_sum>yellow_sum && green_sum>blue_sum)
		return Types::ColorCircle::COLOR_GREEN;
	else if(blue_sum>red_sum && blue_sum>yellow_sum && blue_sum>green_sum)
		return Types::ColorCircle::COLOR_BLUE;
	return Types::ColorCircle::COLOR_OTHER;
}

Types::ColorCircle::Color GenerateCheckers::findColor(const cv::Mat& img, const cv::Mat& img_map)
{
	// TODO
	std::vector<cv::Mat> hsv_planes;
	split(img, hsv_planes);

	int histSizeH = 180;
	float rangeH[] = { 0, 180 } ;
	const float* histRangeH = { rangeH };
	bool uniform = true; bool accumulate = false;
	cv::Mat histH;

	calcHist(&hsv_planes[0], 1, 0, img_map, histH, 1, &histSizeH, &histRangeH, uniform, accumulate);

	float red_sum = 0, yellow_sum = 0, green_sum = 0, blue_sum = 0;

	CLOG(LERROR)<<"Matrix size: "<<histH.size().height<<" "<<histH.size().width<<" Matrix type: "<<histH.type();

	for(int i=0; i<7; ++i)
		red_sum += histH.at<float>(i);
	for(int i=20; i<28; ++i)
		yellow_sum += histH.at<float>(i);
	for(int i=91; i<98; ++i)
		green_sum += histH.at<float>(i);
	for(int i=107; i<113; ++i)
		blue_sum += histH.at<float>(i);
	
	if(red_sum>yellow_sum && red_sum>green_sum && red_sum>blue_sum)
		return Types::ColorCircle::COLOR_RED;
	else if(yellow_sum>red_sum && yellow_sum>green_sum && yellow_sum>blue_sum)
		return Types::ColorCircle::COLOR_YELLOW;
	else if(green_sum>red_sum && green_sum>yellow_sum && green_sum>blue_sum)
		return Types::ColorCircle::COLOR_GREEN;
	else if(blue_sum>red_sum && blue_sum>yellow_sum && blue_sum>green_sum)
		return Types::ColorCircle::COLOR_BLUE;
	return Types::ColorCircle::COLOR_OTHER;
}

} //: namespace GenerateCheckers
} //: namespace Processors
