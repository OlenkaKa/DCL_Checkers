/*!
 * \file
 * \brief
 * \author Aleksandra Karbarczyk
 */

#include <memory>
#include <string>

#include "ImageSender.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <cv_bridge/cv_bridge.h>

namespace Processors {
namespace ImageSender {

ImageSender::ImageSender(const std::string & name) :
		Base::Component(name) , 
		topic_name("topic_name", boost::bind(&ImageSender::onTopicNameChanged, this, _1, _2), "image") {
	registerProperty(topic_name);

}

ImageSender::~ImageSender() {
}

void ImageSender::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_img", &in_img);
	// Register handlers
	registerHandler("onNewImage", boost::bind(&ImageSender::onNewImage, this));
	addDependency("onNewImage", &in_img);

}

bool ImageSender::onInit() {
	static char * tmp = NULL;
	static int tmpi;
	ros::init(tmpi, &tmp, "image_sender", ros::init_options::NoSigintHandler);
	nh = new ros::NodeHandle;
	it = new image_transport::ImageTransport(*nh);
	pub = it->advertise(topic_name, 1);
	return true;
}

bool ImageSender::onFinish() {
	delete it;
	delete nh;
	return true;
}

bool ImageSender::onStop() {
	return true;
}

bool ImageSender::onStart() {
	return true;
}

void ImageSender::onNewImage() {
	cv::Mat image = in_img.read();
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	pub.publish(msg);
	ros::spinOnce();
}

void ImageSender::onTopicNameChanged(const std::string & old_value, const std::string & new_value) {
}

} //: namespace ImageSender
} //: namespace Processors
