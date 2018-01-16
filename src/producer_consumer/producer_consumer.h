/*
 * Producer.h
 *
 *  Created on: Aug 20, 2013
 *      Author: truongnt
 */

#ifndef PRODUCER_H_
#define PRODUCER_H_
#include "base/base_services.h"
#include <iostream>
#include "worker_thread.hpp"
using namespace std;

namespace producer_consumer_thread {
class Producer:public worker_thread {
protected:
	std::string mportSrc;
	std::string mportDst;
	std::string mhostDst;
public:
	Producer(int n, const std::string& portSrc ="",
			const std::string& hostDst="", const std::string& portDst=""):worker_thread(n) {
		mportSrc = portSrc;
		mportDst = portDst;
		mhostDst = hostDst;
	}
	virtual bool setData(const MessageData& data) {
		GloQueueData.push(data);
		return true;
	}
	//support consumer
	virtual void getData(MessageData& data) {
		GloQueueData.wait_and_pop(data);
	}

	void operator()() {
		printing("running PRODUCER No " +utilities::NumberToString(mThreadNo));

		//----------------------------------------------------------------------
		//TO DO:
		for (int i = 0; i < 3; i++) {
			MessageData data;
			this->setData(data);
			printing("	PRODUCER No " + utilities::NumberToString(mThreadNo) + " add: " +utilities::NumberToString(i));
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		}

		//----------------------------------------------------------------------
		printing("stopping PRODUCER No " +utilities::NumberToString(mThreadNo));
	}

};

class Consumer:public worker_thread {
protected:
	std::string mportSrc;
	std::string mportDst;
	std::string mhostDst;
public:
	Consumer(int n, const std::string& portSrc ="",
			const std::string& hostDst="", const std::string& portDst=""):worker_thread(n) {
		mportSrc = portSrc;
		mportDst = portDst;
		mhostDst = hostDst;
	}
	virtual void getData(MessageData& data) {
		GloQueueData.wait_and_pop(data);
	}

	void operator()() override;
};
}
#endif /* PRODUCER_H_ */
