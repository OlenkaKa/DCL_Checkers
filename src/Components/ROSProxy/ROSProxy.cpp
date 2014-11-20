/*!
 * \file
 * \brief
 * \author Aleksandra Karbarczyk
 */

#include <memory>
#include <string>

#include "ROSProxy.hpp"
#include "Common/Logger.hpp"
#include "Types/ImageData.hpp"

#include "irp6_checkers/ImageData.h"

#include <boost/bind.hpp>

namespace Processors {
namespace ROSProxy {

ROSProxy::ROSProxy(const std::string & name) :
		Base::Component(name) , 
		topic_name("topic_name", boost::bind(&ROSProxy::onTopicNameChanged, this, _1, _2), "image_data") {
	registerProperty(topic_name);

}

ROSProxy::~ROSProxy() {
}

void ROSProxy::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_data", &in_data);
	// Register handlers
	h_onNewData.setup(boost::bind(&ROSProxy::onNewData, this));
	registerHandler("onNewData", &h_onNewData);
	addDependency("onNewData", &in_data);

}

bool ROSProxy::onInit() {
	static char * tmp = NULL;
	static int tmpi;
	ros::init(tmpi, &tmp, "discode", ros::init_options::NoSigintHandler);
	nh = new ros::NodeHandle;
	pub = nh->advertise<irp6_checkers::ImageData>(topic_name, 1000);
	return true;
}

bool ROSProxy::onFinish() {
	delete nh;
	return true;
}

bool ROSProxy::onStop() {
	return true;
}

bool ROSProxy::onStart() {
	return true;
}

void ROSProxy::onNewData() {
	Types::ImageData img_data = in_data.read();
	irp6_checkers::ImageData msg;
	irp6_checkers::Point point;
	irp6_checkers::ColorCircle circle;
	
	point.x = img_data.max_x;
	point.y = img_data.max_y;
	msg.MaxChessboardField = point;

	point.x = img_data.min_x;
	point.y = img_data.min_y;
	msg.MinChessboardField = point;

	std::vector<cv::Point>::iterator end_it_fields = img_data.white_fields.end();
	for(std::vector<cv::Point>::iterator it_fields = img_data.white_fields.begin(); it_fields!=end_it_fields; ++it_fields)
	{
		point.x = (*it_fields).x;
		point.y = (*it_fields).y;
		msg.WhiteFields.push_back(point);
	}
	
	std::vector<Types::ColorCircle>::iterator end_it_circles = img_data.circles.end();
	for(std::vector<Types::ColorCircle>::iterator it_circles = img_data.circles.begin(); it_circles!=end_it_circles; ++it_circles)
	{
		circle.x = (*it_circles).center.x;
		circle.y = (*it_circles).center.y;
		circle.color = (*it_circles).color;
		msg.Circles.push_back(circle);
	}

	pub.publish(msg);
	ros::spinOnce();
}

void ROSProxy::onTopicNameChanged(const std::string & old_value, const std::string & new_value) {
}


} //: namespace ROSProxy
} //: namespace Processors
