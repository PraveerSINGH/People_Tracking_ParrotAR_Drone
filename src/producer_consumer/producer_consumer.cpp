/*
 * producer_consumer.cpp
 *
 *  Created on: Aug 20, 2013
 *      Author: truongnt
 */

#include "producer_consumer.h"

namespace producer_consumer_thread {
void Consumer::operator()() {
	printing("running CONSUMER No " +utilities::NumberToString(mThreadNo));

	//----------------------------------------------------------------------
	//TO DO:
	MessageData value;
	for (int i = 0; i < 3; i++) {
		this->getData(value);
		printing("	CONSUMER No " + utilities::NumberToString(mThreadNo) + " pop: " +utilities::NumberToString(value.mLapNo));
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}

	//----------------------------------------------------------------------
	printing("stopping CONSUMER No " +utilities::NumberToString(mThreadNo));
}
}

