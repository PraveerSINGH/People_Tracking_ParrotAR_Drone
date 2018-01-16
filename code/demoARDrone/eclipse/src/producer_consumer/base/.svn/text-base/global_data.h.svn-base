/*
 * GlobalData.hpp
 *
 *  Created on: Aug 20, 2013
 *      Author: truongnt
 */

#pragma once
#include "basic_function.h"
#include "message_data.h"
#include "concurrent_queue.h"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

#define TRACKING_PERFORMANCE

extern concurrent_queue<MessageData> GloQueueData;
extern concurrent_queue<MessageData> GloQueueCommand;
extern std::string GlostrSaveSimulationFolder;

enum SystemState {
	initState,
	activeState
};

//SystemState GloSystemState;
extern SystemState getSystemState();
extern void setSystemState(SystemState);

