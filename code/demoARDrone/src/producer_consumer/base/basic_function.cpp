/*
 * global_data.cpp
 *
 *  Created on: Aug 20, 2013
 *      Author: truongnt
 */

#include "basic_function.h"

namespace utilities {
boost::mutex mutex_print;
boost::mutex mutex_logging;
std::string lastPrettyFunction;
std::string lastPrettyFunctionlogging;
std::ofstream of;

std::string NumberToString ( int Number )
{
	std::ostringstream ss;
	ss << Number;
	return ss.str();
}

int StringToNumber ( const std::string &Text )
{
	std::istringstream ss(Text);
	int result;
	return ss >> result ? result : 0;
}

double toRadian(double degree) {
	return (degree / 180.0 * CV_PI);
}
double toDegree(double rad) {
	return (rad * 180.0 / CV_PI);
}


}

