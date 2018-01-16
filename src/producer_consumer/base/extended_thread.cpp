/*
 * ExtendedThread.cpp
 *
 *  Created on: Sep 6, 2013
 *      Author: truongnt
 */

#include "extended_thread.h"

namespace boost {
boost::mutex support_ExtendedThread::mt_protectCurrentRunningThread;
std::map<boost::thread::id, std::string> support_ExtendedThread::currentRunningThread = std::map<boost::thread::id, std::string>();
bool support_ExtendedThread::addNewRunningThread(std::string strName) {
	try {
		mt_protectCurrentRunningThread.lock();
		utilities::mutex_print.lock();
		printing_color_base("\tAdd new thread\n", utilities::Color::yellow);

		//	std::string thread_id = boost::lexical_cast<std::string>(boost::this_thread::get_id());
		//			printing_with_color_function(thread_id,utilities::Color::cyan);
		currentRunningThread[boost::this_thread::get_id()] = strName;

		printing_color_base("\tThere are " + utilities::NumberToString(currentRunningThread.size()) + " running threads\n", utilities::Color::yellow);
		std::map<boost::thread::id, std::string>::iterator b = currentRunningThread.begin();
		while (b != currentRunningThread.end()) {
			if (boost::this_thread::get_id() != b->first) {
				printing_color_base("\t"+ boost::lexical_cast<std::string>(b->first) + "       ", utilities::Color::green);
			}
			else {
				printing_color_base("\t"+ boost::lexical_cast<std::string>(b->first) + "[NEW]  ", utilities::Color::green);
			}
			printing_color_base(b->second + "\n", utilities::Color::yellow);
			b++;
		}

		utilities::mutex_print.unlock();
		mt_protectCurrentRunningThread.unlock();
		return true;

	} catch (std::exception& e)
	{
		printing("[ERROR] addNewRunningThread: " + std::string(e.what()));
		mt_protectCurrentRunningThread.unlock();
		return false;
	}
}

bool support_ExtendedThread::removeNewRunningThread() {
	try {
		mt_protectCurrentRunningThread.lock();
		utilities::mutex_print.lock();
		printing_color_base("\tRemove thread: " + currentRunningThread[boost::this_thread::get_id()] + "\n", utilities::Color::yellow);
		currentRunningThread.erase(boost::this_thread::get_id());
		//printRunningThread();

		printing_color_base("\tThere are " + utilities::NumberToString(currentRunningThread.size()) + " running threads\n", utilities::Color::yellow);
		std::map<boost::thread::id, std::string>::iterator b = currentRunningThread.begin();
		while (b != currentRunningThread.end()) {
			printing_color_base("\t"+ boost::lexical_cast<std::string>(b->first) + "       ", utilities::Color::green);
			printing_color_base(b->second + "\n", utilities::Color::yellow);
			b++;
		}
		utilities::mutex_print.unlock();
		mt_protectCurrentRunningThread.unlock();
		return true;
	} catch (std::exception& e)
	{
		printing("[ERROR] removeNewRunningThread: " + std::string(e.what()));
		mt_protectCurrentRunningThread.unlock();
		return false;
	}
}
} /* namespace producer_thread */
