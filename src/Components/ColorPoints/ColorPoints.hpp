/*!
 * \file
 * \brief 
 * \author Aleksandra Karbarczyk
 */

#ifndef GENERATECHECKERS_HPP_
#define GENERATECHECKERS_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "Types/Circles/Circles.hpp"
#include "Types/ColorCircle.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace GenerateCheckers {

/*!
 * \class GenerateCheckers
 * \brief GenerateCheckers processor class.
 *
 * Description TODO
 */
class GenerateCheckers: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	GenerateCheckers(const std::string & name = "GenerateCheckers");

	/*!
	 * Destructor
	 */
	virtual ~GenerateCheckers();

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
	Base::DataStreamIn<cv::Mat> in_img_map;

	// Output data streams
	Base::DataStreamOut<std::vector<Types::ColorCircle> > out_circles;

	// Handlers
	Base::EventHandler2 h_onProcess;
	Base::EventHandler2 h_onProcessWithMap;

	// Properties

	
	// Handlers
	void onProcess();
	void onProcessWithMap();
	
	// Others
	Types::ColorCircle::Color findColor(const cv::Mat& img);
	Types::ColorCircle::Color findColor(const cv::Mat& img, const cv::Mat& img_map);

};

} //: namespace GenerateCheckers
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("GenerateCheckers", Processors::GenerateCheckers::GenerateCheckers)

#endif /* GENERATECHECKERS_HPP_ */
