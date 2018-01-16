// Copyright (C) 2013 by - FZI Forschungszentrum Informatik am Karlsruher Institut fuer Technologie
//                       - Institut Eurécom
//                       - Télécom ParisTech
// 
// Author: Benjamin Ranft (benjamin.ranft@web.de)
// 
// This file is part of demoARDrone.
// 
// demoARDrone is free software: you can redistribute it and/or modify it under the terms of the GNU General Public 
// License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later 
// version.
// 
// demoARDrone is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with libHawaii. If not, see 
// <http://www.gnu.org/licenses/>.


// selector for different AR.Drone 2.0 applications
// ================================================

#include "appBase.h"
#include "appFollowLine.h"
#include "appFollowTag.h"
#include "appDevel.h"
#include "hawaii/common/hardware.h"

#include "producer_consumer/base/base_services.h"

#include "producer_consumer/producer_consumer.h"
#include "producer_consumer/worker_thread.hpp"
#include "boost/thread.hpp"
#include "producer_consumer/drone_producer.h"
#include "producer_consumer/drone_consumer.h"
#include "producer_consumer/sender_receiver.h"

#include "boost/date_time/posix_time/posix_time.hpp"

using namespace std;

#define NO_PRODUCER 4
#define NO_COMSUMER 1

void dense3DOffline(std::string filename) ;

static void show_usage(ostream& os)
{
	os << "Usage: <demo> <option(s)>\n"
			<< "Demo:\n"
			<< "\t -h,--help\tShow help.\n\n"
			<< "\t -fl\t\tFollow line.\n\n"
			<< "\t -ft\t\tFollow tag.\n\n"
			<< "\t -d3D\t\tConnect to Drone to get data and save.\n"
			<< "\t\t\tPress key d to trigger function. \n"
			<< "\t\t\tPress key o or l to change height.\n"
			<< "\t\t\tData will store in folder demoARDrone/data/dense3D*.yml\n\n"
			<< "\t -d3Doffline\tFrom data, try to do dense 3D.\n"
			<< "\t\t\t-d3Doffline <filename>\n\n"
			<< "\t -s3D\t\tSparse 3D.\n"
			<< "\t\t\tLandmark-based navigation.\n\n"
			<< "\t -dev\t\tDeveloping application.\n"
			<< "\t\t\tOptions: \n"
			<< "\t\t\tserver: Run threads for laptop 1.\n"
			<< "\t\t\tExample ./bin/demoARDrone -dev [-save <Save Folder>] server <portSrc> <hostDst> <portDst>\n"
			<< "\t\t\tclient: Run threads for laptop 2.\n"
			<< "\t\t\tExample ./bin/demoARDrone -dev [-save <Save Folder>] client <portSrc> <hostDst> <portDst>\n"
			<< "\t\t\tSimulation server: \n"
			<< "\t\t\tExample ./bin/demoARDrone -dev [-simulation <Simulation Folder>] server <portSrc> <hostDst> <portDst>\n"
			<< "\t\t\tSimulation client: \n"
			<< "\t\t\tExample ./bin/demoARDrone -dev [-simulation <Simulation Folder>] client <portSrc> <hostDst> <portDst>\n"
			<< std::endl;
}

#define MAX_ARGUMENTS 10

int main(int argc, char* argv[]) {
	std::string strArgv[MAX_ARGUMENTS];
	for (int i = 0; i < argc; i++) {
		strArgv[i] = argv[i];
	}

	if (strArgv[1] == "-h" || strArgv[1] == "--help") {
		show_usage(std::cout);
	}
	if (strArgv[1] == "-fl") {
		// GPU initialization
		hawaii::GPU::init() ; {
			DroneAppFollowLine droneApp ;
			// run and block until the drone application is finished, process multiple different callbacks at a time
			droneApp.start( std::min( 2, hawaii::CPUCores ) ) ;
			droneApp.wait();
			// GPU shutdown
		} hawaii::GPU::reset() ;

		return 0;
	}
	if (strArgv[1] == "-ft") {
		// GPU initialization
		hawaii::GPU::init() ; {
			DroneAppFollowTag  droneApp ;
			// run and block until the drone application is finished, process multiple different callbacks at a time
			droneApp.start( std::min( 2, hawaii::CPUCores ) ) ;
			droneApp.wait();
			// GPU shutdown
		} hawaii::GPU::reset() ;

		return 0;
	}
	if (strArgv[1] == "-d3D") {
		// GPU initialization
		hawaii::GPU::init() ; {
			DroneAppDevel      droneApp(true) ;//set IsUsingDense3D = true
			// run and block until the drone application is finished, process multiple different callbacks at a time
			droneApp.start( std::min( 2, hawaii::CPUCores ) ) ;
			droneApp.wait();
			// GPU shutdown
		} hawaii::GPU::reset() ;

		return 0;
	}
	if (strArgv[1] == "-d3Doffline" && argc == 3) {
		dense3DOffline(strArgv[2]);
		return 0;
	}
	if (strArgv[1] == "-s3D") {
		// GPU initialization
		hawaii::GPU::init() ; {
			DroneAppDevel      droneApp ;//default set IsUsingDense3D = false
			// run and block until the drone application is finished, process multiple different callbacks at a time
			droneApp.start( std::min( 2, hawaii::CPUCores ) ) ;
			droneApp.wait();
			// GPU shutdown
		} hawaii::GPU::reset() ;


		return 0;
	}
	if (strArgv[1] == "-dev") {
		std::string portSrc;
		std::string hostDst;
		std::string portDst;
		std::string SimulationFolder = "";
		std::string ClientServer;

		if (strArgv[2] == "-simulation") {
			SimulationFolder = strArgv[3];

			ClientServer = strArgv[4];

			portSrc = strArgv[5];
			hostDst = strArgv[6];
			portDst = strArgv[7];
		}
		else {
			ClientServer = strArgv[2];

			portSrc = strArgv[3];
			hostDst = strArgv[4];
			portDst = strArgv[5];
		}

		//remote client - local server
		if (ClientServer == "client") {
			//laptop 2
			printing("running remote client");

			producer_consumer_thread::DroneConsumerClient silCon(2, portSrc, hostDst, portDst);
			//boost::thread t_sil_con(silCon);
			boost::shared_ptr<boost::thread> t_sil_con;
			boost::ExtendedThread::runThread(std::string("DroneConsumerClient"),
					t_sil_con,
					&producer_consumer_thread::DroneConsumerClient::operator(), &silCon
					//, //no argument
			);


			if (SimulationFolder == "") {
				//run with Drone
				hawaii::GPU::init() ; {
					producer_consumer_thread::DroneProducer2 silPro(2, 1000/2);
					silPro.start( std::min( 2, hawaii::CPUCores ) ) ;
					silPro.wait();
					// GPU shutdown
				} hawaii::GPU::reset() ;
			}
			else {
				//simulation
				producer_consumer_thread::SimulateDroneProducer silPro(2);
				//boost::thread t_sil_pro(silPro);
				//t_sil_pro.join();

				boost::shared_ptr<boost::thread> t_sil_pro;
				boost::ExtendedThread::runThread(std::string("SimulateDroneProducer"),
						t_sil_pro,
						&producer_consumer_thread::SimulateDroneProducer::operator(), &silPro
						//, //no argument
				);
				t_sil_pro->join();
			}

			t_sil_con->join();
			printing("stopping main");
		}

		if (ClientServer == "server") {
			printing("running local server");
			try
			{
				setSystemState(SystemState::activeState);
//				boost::thread_group tg_consumer;
//				producer_consumer_thread::DroneConsumer* t_con_arr[NO_COMSUMER];
//				//vector<boost::shared_ptr<boost::thread>> pthread(NO_COMSUMER);
//				for (int i = 0; i < NO_COMSUMER; i++) {
//					t_con_arr[i] = new producer_consumer_thread::DroneConsumer(i);
//
//					boost::thread* pthread;
//					boost::ExtendedThread::runThread(std::string("DroneConsumer"),
//							pthread,
//							&producer_consumer_thread::DroneConsumer::operator(), t_con_arr[i]
//							                                                                               //, //no argument
//					);
//					tg_consumer.add_thread(pthread);
//				}

				producer_consumer_thread::DroneProducerServer silProServer(1, portSrc, hostDst, portDst);
				//boost::thread t_pro_ser(silProServer);
				boost::shared_ptr<boost::thread> t_pro_ser;
				boost::ExtendedThread::runThread(std::string("DroneProducerServer"),
						t_pro_ser,
						&producer_consumer_thread::DroneProducerServer::operator(), &silProServer
						//, //no argument
				);

				if (SimulationFolder == "") {
					//run with Drone
					hawaii::GPU::init() ; {
						producer_consumer_thread::DroneProducer1 silPro(1, 1000/24);
						silPro.start( std::min( 2, hawaii::CPUCores ) ) ;
						silPro.wait();
						// GPU shutdown
					} hawaii::GPU::reset() ;
				}
//				else {
//					//simulation
//					producer_consumer_thread::SimulateDroneProducer silPro(1);
//					//boost::thread t_sil_pro(silPro);
//					//t_sil_pro.join();
//
//					boost::shared_ptr<boost::thread> t_sil_pro;
//					boost::ExtendedThread::runThread(std::string("SimulateDroneProducer"),
//							t_sil_pro,
//							&producer_consumer_thread::SimulateDroneProducer::operator(), &silPro
//							//, //no argument
//					);
//					t_sil_pro->join();
//				}

				t_pro_ser->join();
				//boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
				//tg_consumer.interrupt_all();
				//tg_consumer.join_all();

				printing("stopping main");
			}
			catch (std::exception& e)
			{
				std::cerr << e.what() << std::endl;
			}

		}
		return 0;
	}

	printing("not enough parameters");
	show_usage(std::cerr);
	return 1;
}
