/*!
 * \file
 * \brief
 * \author Aleksandra Karbarczyk
 */

#include <memory>
#include <string>

#include "CvInRange.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace CvInRange {

CvInRange::CvInRange(const std::string & name) :
		Base::Component(name) , 
		channels("channels", boost::bind(&CvInRange::onChannelsChanged, this, _1, _2), 0),
		min_values_str("min_values", boost::bind(&CvInRange::onMinValuesChanged, this, _1, _2), ""), 
		max_values_str("max_values", boost::bind(&CvInRange::onMaxValuesChanged, this, _1, _2), "") {
	registerProperty(channels);
	registerProperty(min_values_str);
	registerProperty(max_values_str);
}

CvInRange::~CvInRange() {
}

void CvInRange::prepareInterface() {
	// Register data streams, events and event handlers
	registerStream("in_img", &in_img);
	registerStream("out_img", &out_img);
	// Register handlers
	h_onProcess.setup(boost::bind(&CvInRange::onProcess, this));
	registerHandler("onProcess", &h_onProcess);
	addDependency("onProcess", &in_img);
	
	std::stringstream ss_min(min_values_str);
	int value;
	while (ss_min >> value)
	{
		min_values.push_back(value);
		if (ss_min.peek() == ',')
			ss_min.ignore();
	}
	std::stringstream ss_max(max_values_str);
	while (ss_max >> value)
	{
		max_values.push_back(value);
		if (ss_max.peek() == ',')
			ss_max.ignore();
	}
}

bool CvInRange::onInit() {
	return true;
}

bool CvInRange::onFinish() {
	return true;
}

bool CvInRange::onStop() {
	return true;
}

bool CvInRange::onStart() {
	return true;
}

void CvInRange::onProcess() {
	cv::Mat src = in_img.read();
	cv::Mat dst;
	std::vector<int>::const_iterator min_first = min_values.begin(),
			min_last = min_first+channels,
			max_first = max_values.begin(),
			max_last = max_first+channels;
	cv::inRange(src, std::vector<int>(min_first, min_last), std::vector<int>(max_first, max_last), dst);
	out_img.write(dst);
}

void CvInRange::onChannelsChanged(int old_value, int new_value) {
	channels = new_value;
}

void CvInRange::onMinValuesChanged(const std::string & old_values, const std::string & new_values) {
	min_values.clear();
	int value;
	std::stringstream ss_min(new_values);
	while (ss_min >> value)
	{
		min_values.push_back(value);
		if (ss_min.peek() == ',')
			ss_min.ignore();
	}
}

void CvInRange::onMaxValuesChanged(const std::string & old_values, const std::string & new_values) {
	max_values.clear();
	int value;
	std::stringstream ss_max(new_values);
	while (ss_max >> value)
	{
		max_values.push_back(value);
		if (ss_max.peek() == ',')
			ss_max.ignore();
	}
}

} //: namespace CvInRange
} //: namespace Processors
