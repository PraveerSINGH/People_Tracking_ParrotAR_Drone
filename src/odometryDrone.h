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


// odometry using additional sensors of AR.Drone 2.0
// =================================================

#include "hawaii/common/tracker.h"
#include <ardrone_autonomy/Navdata.h>
#include <opencv2/core/core.hpp>
#include <boost/thread.hpp>

//#include "producer_consumer/base/basic_function.h"

// odometry using drone's built-in sensors: Altitude is measured via an ultrasonic sensor. Rotations are also directly 
//                                          available from a combination of inertial sensors and a 3D magnetometer. 
//                                          Therefore, this class only integrates forward/backward and sideways 
//                                          velocities. The drone tries to compensate them for drifts using the optical 
//                                          flow measured by the drone's bottom camera.
class OdometryDrone {
	
	// helpers to convert rotation or translation vectors to a new reference yaw angle
	public:
	static cv::Vec3d rotateRotation(    const cv::Vec3d rotation,
	                                    const double    yawZero     ) ;
	static cv::Vec3d rotateTranslation( const cv::Vec3d translation,
	                                    const double    yawZero     ) ;
	// default c'tor
	public:
	OdometryDrone() ;
	
	// integrate most recent "Navdata" message, buffer previous time stamp to apply trapezoidal rule, filter velocities
	public:
	void processNavdata( const ardrone_autonomy::Navdata navdata ) ;
	protected:
	float tmPrev ;
	hawaii::common::Tracker1DPT1 vxTracker,
	                             vyTracker ;
	
	// current pose: These don't use the drone's coordinate system and units, but instead standard computer vision 
	//               conventions:
	//               - "rotation( 0 )" pitch in radians, front-up back-down (i.e. flying backward) positive
	//               - "rotation( 1 )" yaw in radians, top-view clockwise (i.e. turning right) positive
	//               - "rotation( 2 )" roll in radians, left-up right-down (i.e. flying right) positive
	//               - "translation( 0 )" global distance along yaw == 90°
	//               - "translation( 1 )" negative flight altitude
	//               - "translation( 2 )" global distance along yaw == 0°
	public:
	cv::Vec3d getRotation(    const double yawZero = 0.0 ) const ;
	cv::Vec3d getTranslation( const double yawZero = 0.0 ) const ;
	protected:
	cv::Vec3d rotationGlobal,
	          translationGlobal ;
} ;
