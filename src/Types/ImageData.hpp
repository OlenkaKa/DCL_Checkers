/*
 * ImageData.hpp
 *
 *  Created on: 18-11-2014
 *      Author: Aleksandra Karbarczyk
 */

#ifndef IMAGEDATA_HPP_
#define IMAGEDATA_HPP_

#include "Types/ColorPoint.hpp"

#include <opencv2/core/core.hpp>

namespace Types {

class ImageData {
public:
	ImageData() {
	}

	void addColorPoint(Types::ColorPoint point) {
		checker_fields.push_back(point);
	}
	
	int white_fields_num;
	cv::Point max_corner;
	cv::Point min_corner;
	std::vector<Types::ColorPoint> checker_fields;
};

}

#endif /* IMAGEDATA_HPP_ */
