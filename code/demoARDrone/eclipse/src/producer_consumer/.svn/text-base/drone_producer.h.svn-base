/*
 * drone_producer.h
 *
 *  Created on: Aug 20, 2013
 *      Author: truongnt
 */

#ifndef DRONE_PRODUCER_H_
#define DRONE_PRODUCER_H_

#include <iostream>
#include "appBase.h"
#include "hawaii/common/tracker.h"
#include "producer_consumer.h"
#include <opencv2/core/core.hpp>
#include "sender_receiver.h"
#include "base/base_services.h"
#include "../sparse3D2.h"

namespace producer_consumer_thread {
//support remote control for Drone2
enum AutoNavigatorState {
		stateRotating,    	// just make sure to turn, no thing else
		stateMoving,    	// just make sure to move, no thing else
		stateInactive    	// don't do anything, just hover()
	};
class AutoNavigator {

protected:
	AutoNavigatorState state_ ;
	mutable boost::mutex mt_state_;
public:
	bool checkState(const AutoNavigatorState& state) {
		boost::lock_guard< boost::mutex > lock( mt_state_ ) ;
		return state_ == state;
	}
	void setState(const AutoNavigatorState& state) {
		boost::lock_guard< boost::mutex > lock( mt_state_ ) ;
		state_ = state;
	}

protected:
	double droneYawTarget_;
	double upTarget_;
	double altitudeDelta;

	cv::Vec3d      rotationPrevious_;
	cv::Vec3d      translationPrevious_;

public:
	AutoNavigator();
	virtual ~AutoNavigator();
	//functions that support rotate Drone
	static double toRadian(double degree) {
		return (degree / 180.0 * CV_PI);
	}
	static double toDegree(double rad) {
		return (rad * 180.0 / CV_PI);
	}

	static double toDroneYaw(const cv::Vec3d      rotationGlobal) {
		return - AutoNavigator::toDegree(rotationGlobal( 1 ));
	}
	//positive: turn left, negative: turn right

	//Ex: Turn(AutoNavigator::toDegree(rotationGlobal(1)), 90); //turn left 90 degree
	//rotationGlobal(1) means yaw in radian
	void Turn(const cv::Vec3d      translationGlobal, const cv::Vec3d      rotationGlobal, double degree) {
		boost::lock_guard< boost::mutex > lock( mt_state_ ) ;
		const double droneYaw = AutoNavigator::toDroneYaw(rotationGlobal);
		droneYawTarget_ = droneYaw + degree;

		//this->translationPrevious_ = translationGlobal;
		this->rotationPrevious_ = rotationGlobal;
		state_ = AutoNavigatorState::stateRotating;
	}

	//functions that support move Drone
	//distance positive: move up
	void MoveUpDown(const cv::Vec3d      translationGlobal, const cv::Vec3d      rotationGlobal, double distance) {
		boost::lock_guard< boost::mutex > lock( mt_state_ ) ;
		droneYawTarget_ = - translationGlobal(2) - distance;
		this->translationPrevious_ = translationGlobal;
		this->rotationPrevious_ = rotationGlobal;

		altitudeDelta = distance;
		state_ = AutoNavigatorState::stateMoving;
	}

	//when start engage
	void init(const cv::Vec3d      translationGlobal, const cv::Vec3d      rotationGlobal);

	bool getCommands(       DroneCommands& commands,
	                            const cv::Vec3d      rotationGlobal,
	                            const cv::Vec3d      translationGlobal );
};

//---------------------------------------------------------------------------------------
class DroneProducer: public DroneAppBase, public Producer {
protected:
	boost::posix_time::time_duration mTimeBetweenImages;
	boost::posix_time::ptime mSendImageLastTime;
	boost::posix_time::time_duration mSendImageDeltaTime;
public:
	DroneProducer(int n, int TimeBetweenImages = 1000/24);
	virtual ~DroneProducer();
	virtual void processImageFront( const cv_bridge::CvImage imageFront ) override ;
	virtual void processKeystrokes(                                     ) override ;
};

class DroneProducer1: public DroneProducer {
protected:
	AutoNavigator remoteNavigator_;
public:
	DroneProducer1(int n, int TimeBetweenImages = 1000/24);
	virtual ~DroneProducer1();
	virtual void processImageFront( const cv_bridge::CvImage imageFront ) override ;
	virtual void processKeystrokes(                                     ) override ;

	//consumer thread
protected:
	boost::thread         consumerThread_  ;
	//this function is created in a new thread in DroneProducer1::DroneProducer1(...)
	void consume();
//public:
//	void start( const unsigned int threads ) {
//		this->consumerThread_ = boost::thread( &DroneProducer1::consume, this ) ;
//	}
//	void wait() {
//		if( this->consumerThread_.joinable() ) {
//			this->consumerThread_.join() ;
//		}
//	}

protected:
	std::list<MessageData> listWaitingSynchronizeStream_;
	MessageData *md_Stream1_;
	MessageData *md_Stream2_;

	MessageData *md_CurrentStream1_;
	MessageData *md_CurrentStream2_;

	//remake sparse3D demo
protected:
//	const double& focalLengthFrontU,
//	            & focalLengthFrontV,
//	            & focalLengthBottomU,
//	            & focalLengthBottomV ;
//	const double& principalPointFrontU,
//	            & principalPointFrontV,
//	            & principalPointBottomU,
//	            & principalPointBottomV ;

	Sparse3D2  sparse3D2  ;
	void process2ImageStreams(MessageData *stream1, MessageData *stream2);
};
class DroneProducer2: public DroneProducer {
protected:
	AutoNavigator remoteNavigator_;
public:
	DroneProducer2(int n, int TimeBetweenImages = 1000/24);
	virtual ~DroneProducer2();
	virtual void processKeystrokes(                                     ) override ;
};

class DroneProducerServer: public Producer {
public:
	DroneProducerServer(int n,
			const std::string& port,
			const std::string& hostLap2, const string& portLap2);
	virtual ~DroneProducerServer();
	void operator()();
//
//protected:
//	Connection_Manager mConnect;
};

class SimulateDroneProducer: public Producer {
protected:
	boost::posix_time::time_duration mTimeBetweenImages;
	boost::posix_time::ptime mSendImageLastTime;
	boost::posix_time::time_duration mSendImageDeltaTime;
public:
	SimulateDroneProducer(int n);
	virtual ~SimulateDroneProducer();
	void operator()() override;
};

} /* namespace producer_thread */
#endif /* DRONE_PRODUCER_H_ */
