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


// navigate based on landmarks at corridor crossings
// =================================================

#pragma once

#include "commands.h"
#include <opencv2/core/core.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

// map detected landmarks to new flight directions
class Navigator {
	
	// c'tor with required properties of undistorted (!) camera images
	public:
	Navigator( const double focalLengthUArg,
	           const double focalLengthVArg,
	           const double principalPointUArg,
	           const double principalPointVArg ) ;
	protected:
	const double focalLengthU,
	             focalLengthV,
	             principalPointU,
	             principalPointV ;
	
	// map landmark type (1st index) and value (2nd index) to relative further flight angle 
	// user note: The angles for actually used landmarks must be set manually in degrees, top-view clockwise positive.
	public:
	double droneYawRelativeMap[ 32 ][ 128 ] ;
	
	// target distance for approaching symbol, in meters, defaults to 1.5
	public:
	double symbolDistForwardTarget ;
	
	// internal state
	protected:
	enum State {
		stateApproaching, // fly towards landmark until symbol-specific distance is reached
		stateRotating,    // turn towards symbol-specific further flight direction
		stateInactive     // don't do anything, particularly set flight control commands
	} state ;
	
	// detect the landmark, buffer its type and relative position
	public:
	void processImageFront( const cv::Mat   image,
	                        const cv::Vec3d rotationGlobal ) ;
	protected:
	int symbolType,   // in [0,31]
	    symbolValue ; // in [0,127]
	double symbolDistForward,  // in meters
	       symbolDistUp,       // in meters
	       symbolAngleLeft   ; // in degrees
	double droneYawTarget ; // in degrees
	
	// get control commands depending on state: When inactive, "commands" is not modified and "false" is returned. 
	public:
	bool getCommands(       DroneCommands& commands,
	                  const cv::Vec3d      rotationGlobal ) ;
	
	// ensure thread safety and a consistent state
	protected:
	mutable boost::mutex mutex ;
	
} ; // class "Navigator"
