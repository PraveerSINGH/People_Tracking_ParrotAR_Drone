/*
 * ExtendedThread.h
 *
 *  Created on: Sep 6, 2013
 *      Author: truongnt
 */

#ifndef EXTENDEDTHREAD_H_
#define EXTENDEDTHREAD_H_

//boost libraries
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread.hpp>
#include "boost/thread/mutex.hpp"
#include "boost/thread/condition_variable.hpp"

#include "basic_function.h"
#include "global_data.h"

namespace boost {
class support_ExtendedThread {
public:
	static boost::mutex mt_protectCurrentRunningThread;
	static std::map<boost::thread::id, std::string> currentRunningThread;
	static bool addNewRunningThread(std::string strName);

	static bool removeNewRunningThread();

	//static void printRunningThread();
};

class ExtendedThread {
public:
	template <class F,class C>
	static void runThread(std::string threadName,
			shared_ptr<boost::thread>& pthread,
			F functionName, C ownerClass
	) {
		try {
			ExtendedThread extend_t;

			pthread =  shared_ptr<boost::thread>(
					new boost::thread(&ExtendedThread::testFunction1argument<F, C>, extend_t,
							//ss.str(),
							threadName,//thread information
							functionName, ownerClass));

		} catch (std::exception& e)
		{
			printing("[ERROR] runThread0: " + std::string(e.what()));
		}
	}

	template <class F,class C>
	static void runThread(std::string threadName,
			boost::thread*& pthread,
			F functionName, C ownerClass
	) {
		try {
			ExtendedThread extend_t;

			pthread =  new boost::thread(&ExtendedThread::testFunction1argument<F, C>, extend_t,
							//ss.str(),
							threadName,//thread information
							functionName, ownerClass);

		} catch (std::exception& e)
		{
			printing("[ERROR] runThread0: " + std::string(e.what()));
		}
	}

	//bind to another function
	template <class F,class C>
	void testFunction1argument(//std::string id_t,
			std::string threadName,
			F functionName, C ownerClass) {
		try {
			support_ExtendedThread::addNewRunningThread(threadName);
			//support_ExtendedThread::printRunningThread();

			boost::bind(functionName, ownerClass)();

			support_ExtendedThread::removeNewRunningThread();
			//support_ExtendedThread::printRunningThread();

		} catch (std::exception& e)
		{
			printing("[ERROR] testFunction0argument: " + threadName + " " + std::string(e.what()));
		}
	}
	//----------------------------------------------------------------------------------------
	template <class F,class C, class A1>
	static void runThread(std::string threadName,
			shared_ptr<boost::thread>& pthread,
			F functionName, C ownerClass,
			A1 a1) {
		try {
			ExtendedThread extend_t;

			pthread =  shared_ptr<boost::thread>(
					new boost::thread(&ExtendedThread::testFunction1argument<F, C, A1>, extend_t,
							//ss.str(),
							threadName,//thread information
							functionName, ownerClass, a1));
		} catch (std::exception& e)
		{
			printing("[ERROR] runThread: " + std::string(e.what()));
		}
	}

	template <class F,class C, class A1>
	static void runThread(std::string threadName,
			boost::thread*& pthread,
			F functionName, C ownerClass,
			A1 a1
	) {
		try {
			ExtendedThread extend_t;

			pthread =  new boost::thread(&ExtendedThread::testFunction1argument<F, C, A1>, extend_t,
							//ss.str(),
							threadName,//thread information
							functionName, ownerClass, a1);

		} catch (std::exception& e)
		{
			printing("[ERROR] runThread0: " + std::string(e.what()));
		}
	}

	//bind to another function
	template <class F,class C, class A1>
	void testFunction1argument(//std::string id_t,
			std::string threadName,
			F functionName, C ownerClass, A1 a1) {
		try {
			support_ExtendedThread::addNewRunningThread(threadName);
			//support_ExtendedThread::printRunningThread();

			boost::bind(functionName, ownerClass, a1)();

			support_ExtendedThread::removeNewRunningThread();
			//support_ExtendedThread::printRunningThread();
		} catch (std::exception& e)
		{
			printing("[ERROR] testFunction1argument: " + std::string(e.what()));
		}
	}

	//----------------------------------------------------------------------------------------
		template <class F,class C, class A1, class A2>
		static void runThread(std::string threadName,
				shared_ptr<boost::thread>& pthread,
				F functionName, C ownerClass,
				A1 a1, A2 a2) {
			try {
				ExtendedThread extend_t;

				pthread =  shared_ptr<boost::thread>(
						new boost::thread(&ExtendedThread::testFunction2argument<F, C, A1, A2>, extend_t,
								//ss.str(),
								threadName,//thread information
								functionName, ownerClass, a1, a2));
			} catch (std::exception& e)
			{
				printing("[ERROR] runThread: " + std::string(e.what()));
			}
		}

		template <class F,class C, class A1, class A2>
		static void runThread(std::string threadName,
				boost::thread*& pthread,
				F functionName, C ownerClass,
				A1 a1, A2 a2
		) {
			try {
				ExtendedThread extend_t;

				pthread =  new boost::thread(&ExtendedThread::testFunction2argument<F, C, A1, A2>, extend_t,
								//ss.str(),
								threadName,//thread information
								functionName, ownerClass, a1, a2);

			} catch (std::exception& e)
			{
				printing("[ERROR] runThread0: " + std::string(e.what()));
			}
		}

		//bind to another function
		template <class F,class C, class A1, class A2>
		void testFunction2argument(//std::string id_t,
				std::string threadName,
				F functionName, C ownerClass, A1 a1, A2 a2) {
			try {
				support_ExtendedThread::addNewRunningThread(threadName);
				//support_ExtendedThread::printRunningThread();

				boost::bind(functionName, ownerClass, a1, a2)();

				support_ExtendedThread::removeNewRunningThread();
				//support_ExtendedThread::printRunningThread();
			} catch (std::exception& e)
			{
				printing("[ERROR] testFunction1argument: " + std::string(e.what()));
			}
		}
};

} /* namespace producer_thread */
#endif /* EXTENDEDTHREAD_H_ */
