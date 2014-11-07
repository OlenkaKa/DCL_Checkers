/*!
 * \file
 * \brief 
 * \author Aleksandra Karbarczyk
 */

#ifndef CVINRANGE_HPP_
#define CVINRANGE_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace CvInRange {

/*!
 * \class CvInRange
 * \brief CvInRange processor class.
 *
 * Wrapper for InRange function from OpenCV library.
 */
class CvInRange: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CvInRange(const std::string & name = "CvInRange");

	/*!
	 * Destructor
	 */
	virtual ~CvInRange();

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
	
	/*!
	 * Event handler function.
	 */
	void onProcess();
	
	/*!
	 * Callback called when max values are changed
	 */
	void onChannelsChanged(int old_value, int new_value);
	
	/*!
	 * Callback called when max values are changed
	 */
	void onMinValuesChanged(const std::string & old_values, const std::string & new_values);
	
	/*!
	 * Callback called when min values are changed
	 */
	void onMaxValuesChanged(const std::string & old_values, const std::string & new_values);

	// Input data streams
	Base::DataStreamIn<cv::Mat> in_img;

	// Output data streams
	Base::DataStreamOut<cv::Mat> out_img;

	// Handlers
	Base::EventHandler2 h_onProcess;

	// Properties
	Base::Property<int> channels;
	Base::Property<std::string> min_values_str;
	Base::Property<std::string> max_values_str;
	
	// Values for cv::InRange function
	std::vector<int> min_values;
	std::vector<int> max_values;
};

} //: namespace CvInRange
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CvInRange", Processors::CvInRange::CvInRange)

#endif /* CVINRANGE_HPP_ */
