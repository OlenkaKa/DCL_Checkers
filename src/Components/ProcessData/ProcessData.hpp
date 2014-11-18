/*!
 * \file
 * \brief 
 * \author Aleksandra Karbarczyk
 */

#ifndef PROCESSDATA_HPP_
#define PROCESSDATA_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "Types/ChessboardData.hpp"
#include "Types/Circles/Circles.hpp"

#include <opencv2/opencv.hpp>

namespace Processors {
namespace ProcessData {

/*!
 * \class ProcessData
 * \brief ProcessData processor class.
 *
 * Description TODO
 */
class ProcessData: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ProcessData(const std::string & name = "ProcessData");

	/*!
	 * Destructor
	 */
	virtual ~ProcessData();

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
	Base::DataStreamIn<std::vector<std::vector<cv::Point> > > in_contours;

	// Output data streams
	Base::DataStreamOut<Types::ChessboardData> out_data;

	// Handlers
	Base::EventHandler2 h_onProcess;

	// Properties
	Base::Property<int> chessboard_size;

	
	// Handlers
	void onProcess();

};

} //: namespace ProcessData
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ProcessData", Processors::ProcessData::ProcessData)

#endif /* PROCESSDATA_HPP_ */
