/*
 * basic_function.hpp
 *
 *  Created on: Aug 20, 2013
 *      Author: truongnt
 */

#pragma once

#include <fstream>
#include <string>
#include <iostream>

#include <boost/thread.hpp>
#include "boost/thread/mutex.hpp"

//using namespace std;
#define CV_PI 3.1415926535897932384626433832795
namespace utilities {

extern std::string NumberToString ( int Number );
extern int StringToNumber ( const std::string &Text );


extern double toRadian(double degree) ;
extern double toDegree(double rad) ;



extern boost::mutex mutex_print;
extern boost::mutex mutex_logging;
extern std::string lastPrettyFunction;
extern std::string lastPrettyFunctionlogging;
extern std::ofstream of;

enum Color {
	black,
	red,
	green,
	yellow,
	blue,
	magenta,
	cyan,
	lightgray
};
}

#define printing(str) \
		utilities::mutex_print.lock(); \
		if (std::string(__FUNCTION__) == utilities::lastPrettyFunction){ \
			std::cout<<"\033[0m"<<"\t"<<str<<std::endl; \
		} \
		else { \
			std::cout<<"\033[0m"<< __FUNCTION__<<std::endl \
			<<"\t"<<str<<std::endl; \
			utilities::lastPrettyFunction = std::string(__FUNCTION__); \
		} \
		utilities::mutex_print.unlock(); \

#define printing_without_endl(str) \
		utilities::mutex_print.lock(); \
		if (std::string(__FUNCTION__) == utilities::lastPrettyFunction){ \
			std::cout<<"\033[0m"<<"\t"<<str; \
		} \
		else { \
			std::cout<<"\033[0m"<< __FUNCTION__<<std::endl \
			<<"\t"<<str; \
			utilities::lastPrettyFunction = std::string(__FUNCTION__); \
		} \
		utilities::mutex_print.unlock(); \

#define printing_var(str)  \
		utilities::mutex_print.lock(); \
		std::cout<<"\033[0m"<< #str<<"\t"<<str<<std::endl; \
		utilities::mutex_print.unlock(); \

//http://www.cplusplus.com/forum/unices/36461/
//black - 30
//red - 31
//green - 32
//brown - 33
//blue - 34
//magenta - 35
//cyan - 36
//lightgray - 37
#define printing_color_base(str2, color) \
		{ \
			std::string strColor = "\033[0;"+utilities::NumberToString(30 + (int)color)+"m"; \
			if (std::string(__FUNCTION__) == utilities::lastPrettyFunction){ \
				std::cout<<strColor<<str2; \
			} \
			else { \
				std::cout<< __FUNCTION__<<std::endl; \
				std::cout<<strColor<<str2; \
				utilities::lastPrettyFunction = std::string(__FUNCTION__); \
			} \
		} \
		//std::cout<<"\033[0m"<<" ";

#define printing_with_color(str, color) \
		{ \
			utilities::mutex_print.lock(); \
			printing_color_base (std::string("\t" + std::string(str) + "\n"), color); \
			utilities::mutex_print.unlock(); \
		} \

#define printing_with_color_without_endl(str, color) \
		{ \
			utilities::mutex_print.lock(); \
			printing_color_base (std::string("\t" + std::string(str)), color); \
			utilities::mutex_print.unlock(); \
		} \

#define logging(str) \
		utilities::mutex_logging.lock(); \
		utilities::of.open("logging.txt", std::ios::out | std::ios::app); \
		if (std::string(__PRETTY_FUNCTION__) == utilities::lastPrettyFunctionlogging){\
			utilities::of<<"\t"<<str<<std::endl;} \
			else {utilities::of<< __PRETTY_FUNCTION__<<" \n\t"<<str<<std::endl; utilities::lastPrettyFunctionlogging = std::string(__PRETTY_FUNCTION__);} \
			utilities::of.close(); \
			utilities::mutex_logging.unlock();

#define logging1(strKey, strValue) \
		utilities::mutex_logging.lock(); \
		utilities::of.open("logging.txt", std::ios::out | std::ios::app); \
		utilities::of<<strKey<<"\t"<<strValue<<"\t"<< __PRETTY_FUNCTION__<<std::endl; \
		utilities::of.close(); \
		utilities::mutex_logging.unlock();

#define logging2(strKey, strValue1, strValue2) \
		utilities::mutex_logging.lock(); \
		utilities::of.open("logging.txt", std::ios::out | std::ios::app); \
		utilities::of<<strKey<<"\t"<<strValue1<<"\t"<<strValue2<<"\t"<< __PRETTY_FUNCTION__<<std::endl; \
		utilities::of.close(); \
		utilities::mutex_logging.unlock();

#define logging3(strKey, strValue1, strValue2, strValue3) \
		utilities::mutex_logging.lock(); \
		utilities::of.open("logging.txt", std::ios::out | std::ios::app); \
		utilities::of<<strKey<<"\t"<<strValue1<<"\t"<<strValue2<<"\t"<<strValue3<<"\t"<< __PRETTY_FUNCTION__<<std::endl; \
		utilities::of.close(); \
		utilities::mutex_logging.unlock();//#define printing2(str) utilities::mutex_print.lock(); std::cout<<str<<std::endl; utilities::mutex_print.unlock();

