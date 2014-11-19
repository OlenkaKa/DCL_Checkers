/*
 * ColorCircle.hpp
 *
 *  Created on: 18-11-2014
 *      Author: Aleksandra Karbarczyk
 */

#ifndef COLORCIRCLE_HPP_
#define COLORCIRCLE_HPP_

#include <opencv2/core/core.hpp>

namespace Types {

class ColorCircle {
public:

	enum Color {
		COLOR_BLUE,
		COLOR_GREEN,
		COLOR_RED,
		COLOR_YELLOW,
		COLOR_OTHER
	};
	
	ColorCircle(cv::Point checker_center, Color checker_color):
		center(checker_center), color(checker_color) {
	}

	cv::Point center;
	Color color;
};

}

#endif /* COLORCIRCLE_HPP_ */
