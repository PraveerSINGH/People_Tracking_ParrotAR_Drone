/*
 * drone_consumer.cpp
 *
 *  Created on: Aug 20, 2013
 *      Author: truongnt
 */

#include "drone_consumer.h"
#include "hawaii/common/helpers.h"
#include <opencv2/imgproc/imgproc.hpp>

#include <unistd.h>
#include <dirent.h>
#include <sys/types.h> // for opendir(), readdir(), closedir()
#include <sys/stat.h> // for stat()
namespace producer_consumer_thread {

//SimulateDroneProducerClient
DroneConsumerClient::DroneConsumerClient(int n,
		const std::string& port,
		const std::string& hostLap2, const string& portLap2)
:		Consumer(n) {
	this->mportSrc = port;
	this->mportDst = portLap2;
	this->mhostDst = hostLap2;
}

DroneConsumerClient::~DroneConsumerClient() {
	// TODO Auto-generated destructor stub
}

void DroneConsumerClient::operator ()() {
	try {
		//Connection_Manager connect(this->mportSrc, this->mhostDst, this->mportDst);
		//connect.run();
		//connect.initClientState();


	    boost::asio::io_service io_service;
	    //-------------------
	    server s(io_service, utilities::StringToNumber(this->mportSrc), 2);
	    client2 c(io_service, this->mhostDst, this->mportDst);

	    //http://stackoverflow.com/questions/16365561/boost-threading-and-mutexes-in-a-functor
	    //explain while we need to use boost::ref, because copy not allow in boost::mutex.
	    boost::thread t_client2 = boost::thread(&client2::process_sender, boost::ref(c) ) ;
	    //-------------------
	    io_service.run();
	    t_client2.join();

//
//
//		boost::thread* client_side_sender;
//		boost::ExtendedThread::runThread(std::string("runSender2"),
//				client_side_sender,
//				//boost::mem_fn(&Connection_Manager::runClientSideSender), &connect
//				&Connection_Manager::runSender, &connect
//				,2
//		);
//
//		boost::thread* client_side_receiver;
//		boost::ExtendedThread::runThread(std::string("runReceiver2"),
//				client_side_receiver,
//				boost::mem_fn(&Connection_Manager::runReceiver), &connect
//				,2
//				//, //no argument
//		);
//
//		boost::thread_group tg;
//		tg.add_thread(client_side_sender);
//		tg.add_thread(client_side_receiver);
//		tg.join_all();
//
//		while (true) {}
	} catch (std::exception& e)
	{
		printing("[ERROR] DroneConsumerClient::operator (): " + std::string(e.what()));
	}
}

} /* namespace producer_thread */
