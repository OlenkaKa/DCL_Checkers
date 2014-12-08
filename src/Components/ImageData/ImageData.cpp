/*!
 * \file
 * \brief
 * \author Aleksandra Karbarczyk
 */

#include <memory>
#include <string>

#include "ImageData.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace ImageData {

ImageData::ImageData(const std::string & name) :
		Base::Component(name) , 
		fields_number("fields_number", 8, "fields_number") {
	registerProperty(fields_number);

}

ImageData::~ImageData() {
}

void ImageData::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_color_points", &in_color_points);
	registerStream("in_contours", &in_contours);
	registerStream("out_data", &out_data);
	// Register handlers
	h_onProcess.setup(boost::bind(&ImageData::onProcess, this));
	registerHandler("onProcess", &h_onProcess);
	addDependency("onProcess", &in_color_points);
	addDependency("onProcess", &in_contours);

}

bool ImageData::onInit() {
	CLOG(LERROR)<<"ImageData init";
	return true;
}

bool ImageData::onFinish() {
	return true;
}

bool ImageData::onStop() {
	return true;
}

bool ImageData::onStart() {
	return true;
}

void ImageData::onProcess() {
	std::vector<std::vector<cv::Point> > white_fields = in_contours.read();
	std::vector<Types::ColorPoint> color_points = in_color_points.read();
	Types::ImageData result;

	CLOG(LERROR)<<"ImageData: Process started.";
	result.white_fields_num = white_fields.size();
	result.checker_fields = color_points;
	int max_x = INT_MIN;
	int max_y = INT_MIN;
	int min_x = INT_MAX;
	int min_y = INT_MAX;
	for (std::vector<std::vector<cv::Point> >::iterator it = white_fields.begin() ; it != white_fields.end(); ++it)
	{
		int sum_x = 0;
		int sum_y = 0;
		for (std::vector<cv::Point>::iterator it2 = (*it).begin() ; it2 != (*it).end(); ++it2)
		{
			if ((*it2).x > max_x)
				max_x = (*it2).x;
			else if ((*it2).x < min_x)
				min_x = (*it2).x;

			if ((*it2).y > max_y)
				max_y = (*it2).y;
			else if ((*it2).y < min_y)
				min_y = (*it2).y;

			sum_x += (*it2).x;
			sum_y += (*it2).y;
		}
		cv::Point result_point(sum_x/(*it).size(), sum_y/(*it).size());
		//result.addWhiteField(result_point);
	}
	result.max_corner = cv::Point(max_x, max_y);
	result.min_corner = cv::Point(min_x, min_y);

	CLOG(LERROR)<<"ImageData: Process finished.";
	out_data.write(result);
}



} //: namespace ImageData
} //: namespace Processors
