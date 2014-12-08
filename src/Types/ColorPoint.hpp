/*
 * ColorPoint.hpp
 *
 *  Created on: 05-12-2014
 *      Author: Aleksandra Karbarczyk
 */

#ifndef COLORPOINT_HPP_
#define COLORPOINT_HPP_

#include <opencv2/core/core.hpp>

namespace Types {

class ColorPoint {
public:

	enum Color {
		COLOR_BLUE = 0,
		COLOR_GREEN = 1,
		COLOR_RED = 2,
		COLOR_YELLOW = 3,
		COLOR_OTHER = 4
	};
	
	ColorPoint(cv::Point new_point, Color new_color):
		point(new_point), color(new_color) {
	}

	cv::Point point;
	Color color;
};

}

#endif /* COLORPOINT_HPP_ */
