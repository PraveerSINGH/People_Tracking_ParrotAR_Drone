/*
 * cvmat_serilization.h
 *
 *  Created on: Aug 21, 2013
 *      Author: truongnt
 */

#ifndef CVMAT_SERILIZATION_H_
#define CVMAT_SERILIZATION_H_

#include "basic_function.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)
namespace boost {
	namespace serialization {

	template <class Archive>
	void save(Archive & ar, const ::cv::Mat& m, const unsigned int version)
	{
		logging(version);
		//std::cout<<<<version<<std::endl;
		//printing("serialization cvMat version=" + utilities::NumberToString(version));
		size_t elem_size = m.elemSize();
		size_t elem_type = m.type();

		ar & m.cols;
		ar & m.rows;
		ar & elem_size;
		ar & elem_type;

		const size_t data_size = m.cols * m.rows * elem_size;
		ar & boost::serialization::make_array(m.ptr(), data_size);
	}

	template <class Archive>
	void load(Archive & ar, ::cv::Mat& m, const unsigned int version)
	{
		logging(version);
		//printing("DEserialization cvMat version=" + utilities::NumberToString(version));
		int cols, rows;
		size_t elem_size, elem_type;

		ar & cols;
		ar & rows;
		ar & elem_size;
		ar & elem_type;

		m.create(rows, cols, elem_type);

		size_t data_size = m.cols * m.rows * elem_size;
		ar & boost::serialization::make_array(m.ptr(), data_size);
	}

	}
}


#endif /* CVMAT_SERILIZATION_H_ */
