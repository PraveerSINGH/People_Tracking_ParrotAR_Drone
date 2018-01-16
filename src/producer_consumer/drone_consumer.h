/*
 * drone_consumer.h
 *
 *  Created on: Aug 20, 2013
 *      Author: truongnt
 */

#ifndef DRONE_CONSUMER_H_
#define DRONE_CONSUMER_H_
#include "base/base_services.h"

#include "producer_consumer.h"
#include <opencv2/core/core.hpp>
#include "hawaii/GPU/autoMat.h"
#include "sender_receiver.h"
#include <list>

namespace producer_consumer_thread {

class DroneConsumerClient: public Consumer {
public:
	DroneConsumerClient(int n,
			const std::string& port,
			const std::string& hostLap2, const string& portLap2);
	virtual ~DroneConsumerClient();
	void operator()() override;
};

} /* namespace producer_thread */
#endif /* DRONE_CONSUMER_H_ */
