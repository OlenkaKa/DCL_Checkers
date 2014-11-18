/*
 * ChessboardData.hpp
 *
 *  Created on: 18-11-2014
 *      Author: Aleksandra Karbarczyk
 */

#ifndef CHESSBOARDDATA_HPP_
#define CHESSBOARDDATA_HPP_

#include <opencv2/core/core.hpp>

namespace Types {

class ChessboardData {
public:
	ChessboardData() {
	}

	void addCircle(cv::Point circle) {
		circes_centers.push_back(circle);
	}

	void addWhiteField(cv::Point field) {
		white_fields_centers.push_back(field);
	}

	/*
	void setMaxX(int x) {
		max_x = x;
	}

	void setMaxY(int y) {
		max_y = y;
	}

	void setMinX(int x) {
		min_x = x;
	}

	void setMinY(int y) {
		min_y = y;
	}
	*/

//private:
	int max_x;
	int max_y;
	int min_x;
	int min_y;
	std::vector<cv::Point> white_fields_centers;
	std::vector<cv::Point> circes_centers;
};

}

#endif /* CHESSBOARDDATA_HPP_ */
