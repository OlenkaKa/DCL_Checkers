/*!
 * \file
 * \brief 
 * \author Aleksandra Karbarczyk
 */

#ifndef IMGEDATA_HPP_
#define IMGEDATA_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "Types/ImageData.hpp"
#include "Types/ColorPoint.hpp"

#include <opencv2/opencv.hpp>

namespace Processors {
namespace ImageData {

/*!
 * \class ImageData
 * \brief ImageData processor class.
 *
 * Description TODO
 */
class ImageData: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ImageData(const std::string & name = "ImageData");

	/*!
	 * Destructor
	 */
	virtual ~ImageData();

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
	Base::DataStreamIn<std::vector<Types::ColorPoint> > in_color_points;
	Base::DataStreamIn<std::vector<std::vector<cv::Point> > > in_contours;

	// Output data streams
	Base::DataStreamOut<Types::ImageData> out_data;

	// Handlers
	Base::EventHandler2 h_onProcess;

	// Properties
	Base::Property<int> fields_number;

	
	// Handlers
	void onProcess();

};

} //: namespace ImageData
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ImageData", Processors::ImageData::ImageData)

#endif /* IMGEDATA_HPP_ */
