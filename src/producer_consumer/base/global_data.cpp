/*
 * global_data.cpp
 *
 *  Created on: Aug 20, 2013
 *      Author: truongnt
 */

#include "global_data.h"

concurrent_queue<MessageData> GloQueueData;
concurrent_queue<MessageData> GloQueueCommand;

std::string GlostrSaveSimulationFolder = "Simulation";

SystemState GloSystemState = SystemState::initState;
boost::mutex mt_protectSystemState;
SystemState getSystemState() {
	boost::mutex::scoped_lock lock(mt_protectSystemState);
	return GloSystemState;
}
void setSystemState(SystemState state) {
	boost::mutex::scoped_lock lock(mt_protectSystemState);
	GloSystemState = state;
}
