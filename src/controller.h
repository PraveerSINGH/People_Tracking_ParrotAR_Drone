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


// closed-loop controllers, actually independent from drone
// ========================================================

#pragma once

#include "hawaii/common/timer.h"
#include <limits>
#include <cmath>

// PID controller with anti-wind-up
class ControllerPID {
	
	// optional full initialization with PID-parameters and actuator limits
	public:
	ControllerPID( const double kPArg   = 1.0,
	               const double kIArg   = 0.0,
	               const double kDArg   = 0.0,
	               const double uMaxArg = std::numeric_limits< double >::max(),
	               const double uMinArg = std::numeric_limits< double >::min() ) ;
	
	// public parameters
	public:
	double kP, kI, kD   ; // controller constants
	double uMax, uMin   ; // actuator limits for anti-wind-up
	
	// get actuator command from error, which is target minus actual control value
	public:
	double operator ()( const double error ) ;
	
	// reset the previous error and the error integral, e.g. before (re-)activating the corresponding actuator
	public:
	void reset() ;
	
	// temporarily disable the integrator
	public:
	bool disableIntegrator ;
	
	// internal state
	protected:
	double                errorIntegral ; // integral of previous errors
	double                errorPrevious ; // error at previous call
	hawaii::common::Timer timer         ; // helper for integral and derivative control
	
} ; // class "ControllerPID"
