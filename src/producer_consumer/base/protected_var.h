/*
 * message_data.h
 *
 *  Created on: Aug 30, 2013
 *      Author: truongnt
 */

#ifndef PROTECTED_VAR_H_
#define PROTECTED_VAR_H_

#include <boost/thread.hpp>
#include "boost/thread/mutex.hpp"

template <class T>
class ProtectedVariable {
	mutable boost::mutex mt_protectVar_;
	T var_;
	void set(const T& var) {
		boost::mutex::scoped_lock lock(mt_protectVar_);
		var_ = var;
	}

	T get() {
		boost::mutex::scoped_lock lock(mt_protectVar_);
		return var_;
	}
};

#endif /* MESSAGE_DATA_H_ */
