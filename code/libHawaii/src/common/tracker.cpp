// Copyright (C) 2013 by FZI Forschungszentrum Informatik am Karlsruher Institut fuer Technologie
// 
// Author: Benjamin Ranft (benjamin.ranft@web.de)
// 
// This file is part of libHawaii.
// 
// libHawaii is free software: you can redistribute it and/or modify it under the terms of the GNU General Public 
// License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later 
// version.
// 
// libHawaii is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with libHawaii. If not, see 
// <http://www.gnu.org/licenses/>.


// track scalar measures
// =====================

#include "hawaii/common/tracker.h"
#include "hawaii/common/optimization.h"
#include <limits>

namespace hawaii {
namespace common {

// lock-free exponential 1D filter
// -------------------------------

// set state
void Tracker1DExponential::operator ()( const double z ) {
	
	// atomic operation to allow concurrent usage
	union { double d ; int64_t i ; } xPrev, xNew ;
	do {
		xPrev.d = this->x ;
		if( HAWAII_LIKELY( !isnan( xPrev.d ) ) ) {
			xNew.d = xPrev.d * this->ratio + z * ( 1.0 - this->ratio ) ;
		} else {
			xNew.d = z ;
		}
	} while( xPrev.i != __sync_val_compare_and_swap( (int64_t*)&this->x, xPrev.i, xNew.i ) ) ;
}

// "PT1-like" 1D filter
// --------------------

// assignment using, but not copying the mutex
Tracker1DPT1& Tracker1DPT1::operator =( const Tracker1DPT1& that ) {
	if( this != &that ) {
		boost::lock_guard< boost::mutex > lockThis( this->mutex ),
		                                  lockThat( that.mutex  ) ;
		this->timer = that.timer ;
		this->init  = that.init  ;
		this->x     = that.x     ;
		this->tau   = that.tau   ;
	}
	return *this ;
}

// update state
void Tracker1DPT1::operator ()( const double z ) {
	
	// prevent race conditions
	boost::lock_guard< boost::mutex > lock( this->mutex ) ;
	
	// update or initialize?
	if( HAWAII_LIKELY( this->init ) ) {
		
		// compute weighted mean based on time since last update
		const double deltaT = this->timer.tocTic() / 1000.0 ;
		const double ratio  = this->tau / ( this->tau + deltaT ) ;
		this->x = this->x * ratio + z * ( 1.0 - ratio ) ;
	}
	
	// initialize exactly with measurement
	else {
		this->timer.tic() ;
		this->init = true ;
		this->x = z ;
	}
}

// "Kalman-like" 1D filter
// -----------------------

// assignment using, but not copying the mutex
Tracker1DKalman& Tracker1DKalman::operator =( const Tracker1DKalman& that ) {
	if( this != &that ) {
		boost::lock_guard< boost::mutex > lockThis( this->mutex ),
		                                  lockThat( that.mutex  ) ;
		this->timer = that.timer ;
		this->init  = that.init  ;
		this->x     = that.x     ;
		this->p     = that.p     ;
		this->q     = that.q     ;
		this->r     = that.r     ;
	}
	return *this ;
}

// specific interface
void Tracker1DKalman::predict( double& xRet,
                               double& pRet ) const {
	// prevent race conditions
	boost::lock_guard< boost::mutex > lock( this->mutex ) ;
	
	// predict state variance based on time since last update
	xRet = this->x ;
	pRet = this->p + this->q * this->timer.toc() / 1000.0 ;
}
void Tracker1DKalman::update( const double z,
                              const double rArg ) {
	// prevent race conditions
	boost::lock_guard< boost::mutex > lock( this->mutex ) ;
	
	// update or initialize?
	if( HAWAII_LIKELY( this->init ) ) {
		
		// internally predict state variance
		this->p += this->q * this->timer.tocTic() / 1000.0 ;
		
		// compute weighted mean based on state variance and measurement noise
		const double denominator = rArg + p ;
		this->x = ( rArg * this->x + this->p * z ) / denominator ;
		this->p =   rArg           * this->p       / denominator ;
	}
	
	// initialize exactly with measurement
	else {
		this->timer.tic() ;
		this->init = true ;
		this->x    = z    ;
		this->p    = rArg ;
	}
}

} } // namespaces "hawaii::common"
