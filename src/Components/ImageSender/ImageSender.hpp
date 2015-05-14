/*!
 * \file
 * \brief 
 * \author Aleksandra Karbarczyk
 */

#ifndef IMAGESENDER_HPP_
#define IMAGESENDER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "image_transport/image_transport.h"


namespace Processors {
namespace ImageSender {

/*!
 * \class ImageSender
 * \brief ImageSender processor class.
 *
 * Description TODO
 */
class ImageSender: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ImageSender(const std::string & name = "ImageSender");

	/*!
	 * Destructor
	 */
	virtual ~ImageSender();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	// Input data streams
	Base::DataStreamIn<cv::Mat> in_img;

	// Output data streams

	// Handlers

	// Properties
	Base::Property<std::string> topic_name;

	
	// Handlers
	void onNewImage();
	
	void onTopicNameChanged(const std::string & old_value, const std::string & new_value);
	
	ros::NodeHandle * nh;
	image_transport::ImageTransport * it;
	image_transport::Publisher pub;

};

} //: namespace ImageSender
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ImageSender", Processors::ImageSender::ImageSender)

#endif /* IMAGESENDER_HPP_ */
