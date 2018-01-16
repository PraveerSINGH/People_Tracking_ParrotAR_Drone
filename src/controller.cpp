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

#include "controller.h"
#include <algorithm>

// optional full initialization with PID-parameters and actuator limits
ControllerPID::ControllerPID( const double kPArg,
                              const double kIArg,
                              const double kDArg,
                              const double uMaxArg,
                              const double uMinArg ) :
	// public parameters
	kP(   kPArg   ),
	kI(   kIArg   ),
	kD(   kDArg   ),
	uMax( uMaxArg ),
	uMin( uMaxArg != std::numeric_limits< double >::max()
	   && uMinArg == std::numeric_limits< double >::min() ?
	     -uMaxArg :
	      uMinArg ),
	
	// temporarily disable the integrator
	disableIntegrator( false ),
	
	// internal state
	errorIntegral( 0.0 ),
	errorPrevious( NAN ),
	timer( false ) {
}

// get actuator command from error, which is target minus actual control value
double ControllerPID::operator ()( const double error ) {
	
	// handle first call specifically
	if( !this->timer.isRunning() ) {
		
		// start timer
		this->timer.tic() ;
		
		// remember error for next time
		this->errorPrevious = error ;
		
		// simple proportional control, obey actuator limits
		return std::max( this->uMin, std::min( this->uMax, this->kP * error ) ) ;
	}
	
	// get time since last call, restart timer
	const double secondsLastCall = this->timer.tocTic() / 1000.0 ;
	
	// compute controller contributions
	const double uP = this->kP * error,
	             uI = this->kI * this->errorIntegral,
	             uD = this->kD * ( error - this->errorPrevious ) / secondsLastCall ;
	
	// full PID control, obey actuator limits
	const double uUnlim = uP + uI + uD,
	             uLim   = std::max( this->uMin, std::min( this->uMax, uUnlim ) ) ;
	
	// anti-wind-up: only integrate if actuator is not saturated in the integral controller's direction
	// developer note: implemented according to www.isep.pw.edu.pl/ZakladNapedu/lab-ane/anti-windup.pdf
	if( !this->disableIntegrator
	 && ( ( uUnlim == uLim )
//	   || ( uI     * error ) < 0.0 ) ) { // dev. note: fig.  9 of the file above
	   || ( uUnlim * error ) < 0.0 ) ) { // dev. note: fig. 10 of the file above
		
		// linearly interpolate between previous and current errors
		this->errorIntegral += ( this->errorPrevious + error ) / 2.0 * secondsLastCall ;
	}
	
	// remember error for next time
	this->errorPrevious = error ;
	
	// finally...
	return uLim ;
}
	
// reset the previous error and the error integral, e.g. before (re-)activating the corresponding actuator
void ControllerPID::reset() {
	this->errorIntegral = 0.0 ;
	this->errorPrevious = NAN ;
	this->timer.reset() ;
}
