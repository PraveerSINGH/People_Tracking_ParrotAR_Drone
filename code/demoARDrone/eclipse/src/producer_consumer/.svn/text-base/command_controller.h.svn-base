/*
 * command_controller.h
 *
 *  Created on: Sep 9, 2013
 *      Author: truongnt
 */

#ifndef COMMAND_CONTROLLER_H_
#define COMMAND_CONTROLLER_H_
#include "base/base_services.h"

namespace producer_thread {

template<typename Data>
class CommandController {
protected:
	std::queue<Data> mListCommand;
	mutable boost::mutex mt_protectCurrentCommand;
public:
	CommandController();
	virtual ~CommandController();

	bool getCurrentCommand(Data& cm) {
		boost::mutex::scoped_lock lock(mt_protectCurrentCommand);
		if (mListCommand.empty()) {
			return false;
		}

		cm = mListCommand.front();
		return true;
	}

	void addCommand(const Data& cm) {
		boost::mutex::scoped_lock lock(mt_protectCurrentCommand);
		mListCommand.push(cm);
	}

	void nextCommand() {
		boost::mutex::scoped_lock lock(mt_protectCurrentCommand);
		mListCommand.pop();
	}
};

} /* namespace producer_thread */
#endif /* COMMAND_CONTROLLER_H_ */
