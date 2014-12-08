/*!
 * \file
 * \brief 
 * \author Aleksandra Karbarczyk
 */

#ifndef COLORPOINTS_HPP_
#define COLORPOINTS_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "Types/Circles/Circles.hpp"
#include "Types/ColorPoint.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace ColorPoints {

/*!
 * \class ColorCircles
 * \brief ColorCircles processor class.
 *
 * Description TODO
 */
class ColorPoints: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ColorPoints(const std::string & name = "ColorPoints");

	/*!
	 * Destructor
	 */
	virtual ~ColorPoints();

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
	Base::DataStreamIn<Types::Circles> in_circles;
	Base::DataStreamIn<cv::Mat> in_img;
	Base::DataStreamIn<cv::Mat> in_img_mask;

	// Output data streams
	Base::DataStreamOut<std::vector<Types::ColorPoint> > out_points;

	// Handlers
	Base::EventHandler2 h_onProcess;
	Base::EventHandler2 h_onProcessWithMask;

	// Properties

	
	// Handlers
	void onProcess();
	void onProcessWithMask();
	
	// Others
	Types::ColorPoint::Color findColor(const cv::Mat& img);
	Types::ColorPoint::Color findColor(const cv::Mat& img, const cv::Mat& img_mask);

};

} //: namespace ColorPoints
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ColorPoints", Processors::ColorPoints::ColorPoints)

#endif /* COLORPOINTS_HPP_ */
