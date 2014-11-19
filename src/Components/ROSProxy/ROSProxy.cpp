/*!
 * \file
 * \brief
 * \author Aleksandra Karbarczyk
 */

#include <memory>
#include <string>

#include "ROSProxy.hpp"
#include "Common/Logger.hpp"

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
	// TODO
	static char * tmp = NULL;
	static int tmpi;
	ros::init(tmpi, &tmp, "discode", ros::init_options::NoSigintHandler);
	nh = new ros::NodeHandle;
	//pub = nh->advertise<irp6_checkers::ImageData>(topic_name, 1000);
	pub = nh->advertise<std_msgs::Int32>(topic_name, 1000);
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
	// TODO
	std_msgs::Int32 msg;
	msg.data = 5;
	pub.publish(msg);
	ros::spinOnce();
}

void onTopicNameChanged(const std::string & old_value, const std::string & new_value) {
	// TODO
}


} //: namespace ROSProxy
} //: namespace Processors
