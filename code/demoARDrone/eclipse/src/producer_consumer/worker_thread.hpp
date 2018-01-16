/*
 * worker_thread.hpp
 *
 *  Created on: Aug 20, 2013
 *      Author: truongnt
 */

#ifndef WORKER_THREAD_HPP_
#define WORKER_THREAD_HPP_

#include "boost/thread.hpp"
#include <iostream>
//#include "basic_function.h"
using namespace std;

class worker_thread {
protected:
	int mThreadNo;
public:
	worker_thread(int N) {
		mThreadNo = N;
		//mThread = boost::thread(&worker_thread::processing, this);
	}
	virtual ~worker_thread() {

	}

	virtual void operator()() = 0;
	/*
	{
		utilities::printing("running thread No = " + utilities::NumberToString(mThreadNo));

		//----------------------------------------------------------------------
		//TO DO:
		boost::this_thread::sleep(boost::posix_time::seconds(mThreadNo));

		//----------------------------------------------------------------------
		utilities::printing("stopping thread No = " + utilities::NumberToString(mThreadNo));
	}
	*/
};


#endif /* WORKER_THREAD_HPP_ */
