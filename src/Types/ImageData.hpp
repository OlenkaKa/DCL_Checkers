/*
 * ImageData.hpp
 *
 *  Created on: 18-11-2014
 *      Author: Aleksandra Karbarczyk
 */

#ifndef IMAGEDATA_HPP_
#define IMAGEDATA_HPP_

#include "Types/ColorCircle.hpp"

#include <opencv2/core/core.hpp>

namespace Types {

class ImageData {
public:
	ImageData() {
	}

	void addCircle(Types::ColorCircle circle) {
		circles.push_back(circle);
	}

	void addWhiteField(cv::Point field) {
		white_fields.push_back(field);
	}

	int max_x;
	int max_y;
	int min_x;
	int min_y;
	std::vector<cv::Point> white_fields;
	std::vector<Types::ColorCircle> circles;
};

}

#endif /* IMAGEDATA_HPP_ */
