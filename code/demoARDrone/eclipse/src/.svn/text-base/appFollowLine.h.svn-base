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


// follow a red line on the ground
// ===============================

#pragma once

#include "appBase.h"
#include "hawaii/common/tracker.h"

// derived from base drone application
class DroneAppFollowLine : public DroneAppBase {
	
	// c'tor to set maximum forward speed (0.0 1.0]
	public:
	DroneAppFollowLine( const double speed = 0.15 ) :
		DroneAppBase( true ),
		trkHeight( 0.05 ),
		trkRoll(   0.05 ),
		trkOffset( 0.05 ),
		trkAngle(  0.05 ),
		targetHeight( 1.5 ) {
		this->commands.controllerLinX.uMax =  speed ; this->commands.controllerLinY.uMax =  0.20 ;
		this->commands.controllerLinX.uMin = -speed ; this->commands.controllerLinY.uMin = -0.20 ;
	}
	
	// filter noisy measurements
	protected:
	hawaii::common::Tracker1DPT1 trkHeight,  // height above ground [m]
	                             trkRoll   ; // roll angle w.r.t. ground [rad], right is positive
											 // developer note: yaw, not roll, I think
	virtual void processNavdata( const ardrone_autonomy::Navdata navdata ) override {
		this->trkHeight( navdata.altd / 1000.0         ) ;
		this->trkRoll(   navdata.rotX * CV_PI / 180.0  ) ;
	}
	
	// filter line detection results and determine flight commands
	protected:
	hawaii::common::Tracker1DPT1 trkOffset,  // lateral offset from line [m], drone left of line is positive
	                             trkAngle  ; // angle difference from line [rad], clock-wise from drone to line is positive
	double targetHeight ;
	virtual void processImageBottom( const cv_bridge::CvImage imageBottom ) override ;
} ;
