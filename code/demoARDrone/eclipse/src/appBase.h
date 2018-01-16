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


// base class for building actual AR.Drone 2.0 applications upon
// =============================================================

#pragma once

#include "commands.h"
#include "odometryDrone.h"
#include "toast2/stereo/undistortRectify.h"
#include "hawaii/common/timer.h"
#include <opencv2/core/core.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <cv_bridge/cv_bridge.h>
#pragma GCC diagnostic pop
#include <sensor_msgs/CameraInfo.h>
#include <ardrone_autonomy/Navdata.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>

#include "producer_consumer/base/base_services.h"

// drone's state from "Navdata": The names below were just copied from "ARDroneLib/Soft/Common/control_states.h" and 
//                               "https://github.com/AutonomyLab/ardrone_autonomy#legacy-navigation-data". Not all of 
//                               them actually make too much sense to me...
enum StateDrone {
	stateDroneDefault  = 0,
	stateDroneInit     = 1,
	stateDroneLanded   = 2,
	stateDroneFlying   = 3,
	stateDroneHovering = 4,
	stateDroneTest     = 5,
	stateDroneTakeOff  = 6,
	stateDroneGoToFix  = 7,
	stateDroneLanding  = 8,
	stateDroneLooping  = 9
} ;

// class to derive your drone application from: includes input image undistortion, manual keyboard and gamepad control, 
//                                              connection to ROS, convenience features etc.
class DroneAppBase {
	
	// run the drone application, wait for it to finish or stop it immediately
	public:
	void start( const unsigned int threads = 1 ) ;
	void wait() ;
	void stop() ;
	
	// c'tor to optionally select the bottom instead of the front camera, feel free to change on-line
	public:
	DroneAppBase( bool cameraBottomNotFrontArg = false ) ;
	virtual ~DroneAppBase() ;
	protected:
	bool cameraBottomNotFront ;
	
	// processing functions to be implemented by derived classes
	// user note: The Images are undistorted already. The resulting virtual cameras' properties are available as matrices 
	//            "this->camera{Front,Bottom}" or scalars "this->{focalLength,principalPoint}{Front,Bottom}{U,V}" below.
	public:
	virtual void processImageFront(  const cv_bridge::CvImage        imageFront  ) ;
	virtual void processImageBottom( const cv_bridge::CvImage        imageBottom ) ;
	virtual void processNavdata(     const ardrone_autonomy::Navdata navdata     ) ;
	
	// sensor data: Alternatively to the copies given to the methods above, you may read the most recent data here.
	private:
	cv_bridge::CvImage        imageFrontReal  ;
	cv_bridge::CvImage        imageBottomReal ;
	ardrone_autonomy::Navdata navdataReal     ;
	public:
	cv_bridge::CvImage        const& imageFront  ;
	cv_bridge::CvImage        const& imageBottom ;
	ardrone_autonomy::Navdata const& navdata     ;
	
	// odometry based on drone's on-board sensors, buffered poses at points in time when the respective most recent 
	// image has come in, read-only public accessors
	private:
	OdometryDrone odoDroneReal               ;
	cv::Vec3d     rotationImageFrontReal     ;
	cv::Vec3d     rotationImageBottomReal    ;
	cv::Vec3d     translationImageFrontReal  ;
	cv::Vec3d     translationImageBottomReal ;
	public:
	OdometryDrone const& odoDrone               ;
	cv::Vec3d     const& rotationImageFront     ;
	cv::Vec3d     const& rotationImageBottom    ;
	cv::Vec3d     const& translationImageFront  ;
	cv::Vec3d     const& translationImageBottom ;
	
	// control commands and status
	// user note: Your commands will be regularly sent to the drone only if "this->engaged" is true, and after having 
	//            been merged with manual control. You cannot activate autonomous mode; only the operator may manually 
	//            do so by pressing "B" on the keyboard or "green/A" on the gamepad.
	public:
	DroneCommands commands ;
	private: bool        engagedReal ;
	public:  bool const& engaged     ;
	
	// helpers to optionally reset your application's state
	// user note: Keep in mind that your commands are only sent to the drone in autonomous mode. If the state of your 
	//            derived application needs to be reset after autonomous control is engaged or after taking off, 
	//            override "resetAt*()" accordingly.
	protected:	
	virtual void resetAtTakeOff() {}
	virtual void resetAtEngage()  {}
	
	// undistortion of input images
	// user note:      These are most probably not of any concern for you...
	// developer note: ...but they must be initialized before the references below.
	private:
	toast2::stereo::Undistorter undistorterFront,
	                            undistorterBottom ;
	bool undistorterFrontInit,
	     undistorterBottomInit ;
	
	// camera matrices and relevant scalars to be read by anyone interested
	public:
	const cv::Matx< double, 3, 3 >& cameraFront,
	                              & cameraBottom ;
	const double& focalLengthFrontU,
	            & focalLengthFrontV,
	            & focalLengthBottomU,
	            & focalLengthBottomV ;
	const double& principalPointFrontU,
	            & principalPointFrontV,
	            & principalPointBottomU,
	            & principalPointBottomV ;
	
	// manual flight via keyboard or gamepad
	// user note #1: Several keys, buttons and axes are already used to control manual flight, and should therefore not 
	//               be evaluated for other purposes by any derived drone application:
	//               - forward/backward:    U/J keys and left stick vertical
	//               - strafe left/right:   H/K keys and left stick horizontal
	//               - up/down:             O/L keys and right stick vertical
	//               - turn left/right:     Y/I keys and right stick horizontal 
	//               - take off/land:       T/G keys and green(A)/red(B) buttons
	//               - engage/dis-engage:   B/spacebar keys and start/back buttons
	//               - end the application: Esc key or Ctrl+C key combination
	//               The Z key is also mapped to turning left, because it's in Y's place on a German keyboard.
	// user note #2: For quickest responses, the remaining inputs should be implemented in "processKeystrokes()" - 
	//               however, any other method works as well. They can be accessed as follows:
	//               - check (multiple) keyboard key states via "this->keystates[ SDLK_* ]"
	//               - read a gamepad's directional cross like "this->cross & SDL_HAT_{UP,DOWN,LEFT,RIGHT,CENTERED}"
	//               - get a gamepad's analog triggers [0.0f..1.0f] and sticks [-1.0f..1.0f] (both already used!) via
	//                 "this->analog[ trigger{Left,Right} ]" and "this->analog[ stick{Left,Right}{Horiz,Verti} ]"
	//               - check a gamepad's buttons like "this->buttons[ button* ]"
	// developer note: Gamepad support has been developed using a Logitech F710 in XInput mode. It should be compatible 
	//                 at least with Logitech's F510 and F310 models as well as the Microsoft Xbox controller.
	public:
	virtual void processKeystrokes() {}
	// keyboard
	private: const unsigned char*        keystatesReal ;
	public:  const unsigned char* const& keystates     ;
	//remote keyboard: these functions and variables build up
	//in a similar way with normal keyboard, which is implemented
	//in function watchdogThreadFunction.
	//there are 3 steps:
	//	+ PC1 send command: mapping from keyboard of PC1 to Command: function ...
	//	+ send this command (warped in MessageData)...
	//	+ receve and mapping into remote keyboard: function setCommand(Command)

	//for more information about const
	//http://publications.gbdirect.co.uk/c_book/chapter8/const_and_volatile.html
	private: unsigned char lastRemoteKey_;
	private: boost::posix_time::ptime lastTimeRemoteKeyPress_;
	private: unsigned char remotekeystateRealNotConst_[256];
	private: const unsigned char* remotekeystatesReal_;
	private: const unsigned char* const& remotekeystates;
	//
	public: void setCommand(const Command& cm);
	//update/reset the state of the Key
	//For example: turn off last key pressed after 40ms --> in order to make smootherKeyboard works well
	private: void RefreshRemoteKey();
	// digital cross
	private:       unsigned char  crossReal ;
	public:  const unsigned char& cross     ;
	// analog sticks and triggers
	public:
	enum StickTrigger {
		stickLeftHoriz  = 0,
		stickLeftVerti  = 1,
		triggerLeft     = 2,
		stickRightHoriz = 3,
		stickRightVerti = 4,
		triggerRight    = 5
	} ;
	private:       double    analogReal[  6 ] ;
	public:  const double (& analog)[     6 ] ;
	// buttons
	public:
	enum Button {
		buttonA      = 0,
		buttonGreen  = 0,
		buttonB      = 1,
		buttonRed    = 1,
		buttonX      = 2,
		buttonBlue   = 2,
		buttonY      = 3,
		buttonYellow = 3,
		buttonL      = 4,
		buttonR      = 5,
		buttonBack   = 6,
		buttonStart  = 7
	} ;
	private:       bool    buttonsReal[ 8 ] ;
	public:  const bool (& buttons)[    8 ] ;
	
	// connection to ROS using "pimpl" idiom, callbacks for subscribed topics, battery state monitoring
	private:
	struct ConnROS ;
	boost::scoped_ptr< ConnROS > connROS ;
	void callbackImageFront(  const sensor_msgs::ImageConstPtr&   imageFrontROSPtr  ) ;
	void callbackImageBottom( const sensor_msgs::ImageConstPtr&   imageBottomROSPtr ) ;
	void callbackCameraInfo(  const sensor_msgs::CameraInfoPtr&   cameraInfoPtr     ) ;
	void callbackNavdata(     const ardrone_autonomy::NavdataPtr& navdataPtr        ) ;
	float batteryPercentPrev ;
	
	// watch dog and helpers
	private:
	void watchdogThreadFunc() ;
	boost::thread         watchdogThread  ;
	bool                  watchdogRunning ;
	hawaii::common::Timer watchdogTimer   ;
} ; // class "DroneAppBase"
