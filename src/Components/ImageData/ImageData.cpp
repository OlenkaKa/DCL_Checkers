/*!
 * \file
 * \brief
 * \author Aleksandra Karbarczyk
 */

#include <memory>
#include <string>

#include "ProcessData.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace ProcessData {

ProcessData::ProcessData(const std::string & name) :
		Base::Component(name) , 
		fields_number("fields_number", 8, "fields_number") {
	registerProperty(fields_number);

}

ProcessData::~ProcessData() {
}

void ProcessData::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_circles", &in_circles);
	registerStream("in_contours", &in_contours);
	registerStream("out_data", &out_data);
	// Register handlers
	h_onProcess.setup(boost::bind(&ProcessData::onProcess, this));
	registerHandler("onProcess", &h_onProcess);
	addDependency("onProcess", &in_circles);
	addDependency("onProcess", &in_contours);

}

bool ProcessData::onInit() {
	CLOG(LERROR)<<"ProcessData init";
	return true;
}

bool ProcessData::onFinish() {
	return true;
}

bool ProcessData::onStop() {
	return true;
}

bool ProcessData::onStart() {
	return true;
}

void ProcessData::onProcess() {
	std::vector<std::vector<cv::Point> > white_fields = in_contours.read();
	std::vector<Types::ColorCircle> circles = in_circles.read();
	Types::ImageData result;

	CLOG(LERROR)<<"ProcessData: Process started.";
	result.circles = circles;
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
		result.addWhiteField(result_point);
	}
	result.max_x = max_x;
	result.max_y = max_y;
	result.min_x = min_x;
	result.min_y = min_y;

	CLOG(LERROR)<<"ProcessData: Process finished.";
	/*
	CLOG(LERROR)<<"ProcessData: Results:";
	CLOG(LERROR)<<"\tmax x: "<<result.max_x;
	CLOG(LERROR)<<"\tmax y: "<<result.max_y;
	CLOG(LERROR)<<"\tmin x: "<<result.min_x;
	CLOG(LERROR)<<"\tmin y: "<<result.min_y;
	*/
	out_data.write(result);
}



} //: namespace ProcessData
} //: namespace Processors
