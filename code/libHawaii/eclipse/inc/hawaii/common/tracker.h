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

#pragma once

#include "hawaii/common/error.h"
#include "hawaii/common/timer.h"
#include "hawaii/GPU/nvccFixHeaderAbove.h"
#include <boost/thread.hpp>
#include "hawaii/GPU/nvccFixHeaderBelow.h"

namespace hawaii {
namespace common {

// exponential, lock-free 1D tracker
class Tracker1DExponential {
	
	// c'tor with optional ratio
	public:
	Tracker1DExponential( const double ratioArg = 0.9 ) :
		x( NAN ),
		ratio( ratioArg ) {
		HAWAII_ERROR_CONDITIONAL( this->ratio < 0.0 || this->ratio > 1.0, "invalid ratio" ) ;
	}
	
	// get state
	public:
	operator double() const { return this->x ; }
	bool isInit() const { return !isnan( this->x ) ; }
	
	// (re-)set state
	public:
	void operator ()( const double z ) ;
	void reset() { this->x = NAN ; }
	
	// members
	protected:
	double x ;
	double ratio ;
} ;

// base class for time-dependent, locking 1D trackers
// user note: Do not use these for tracking times, as they will apply a higher weight to longer time spans and 
//            therefore distort the estimate. 
class Tracker1DBase {
	
	// c'tor for un-initialized instance, prevent use of base class
	protected:
	Tracker1DBase() :
		timer( false ),
		init( false ),
		x( NAN ) {
	}
	~Tracker1DBase() {}
	
	// get state
	public:
	operator double() const { return this->x ; }
	bool isInit() const { return this->init ; }
	
	// (re-)set state
	public:
	void operator ()( const double z ) { (void)z ; HAWAII_ERROR( "virtual inheritance not used, so only handle pointers to derived types" ) ; }
	void reset()                       {           HAWAII_ERROR( "virtual inheritance not used, so only handle pointers to derived types" ) ; }
	protected:
	void resetBase() {
		this->timer.reset() ;
		this->init = false ;
		this->x = NAN ;
	}
	
	// members
	protected:
	mutable boost::mutex  mutex ;
	hawaii::common::Timer timer ;
	bool                  init  ;
	double                x     ;
	
} ; // class "Tracker1DBase"

// "PT1-like" 1D filter
class Tracker1DPT1 : public Tracker1DBase {
	
	// c'tor
	public:
	Tracker1DPT1( const double tauArg ) :
		tau( tauArg ) { // PT1 time constant
		HAWAII_ERROR_CONDITIONAL( this->tau < 0.0, "invalid time constant" ) ;
	}
	
	// copy construction and assignment using, but not copying the mutex
	public:
	Tracker1DPT1( const Tracker1DPT1& that ) { *this = that ; }
	Tracker1DPT1& operator =( const Tracker1DPT1& that ) ;
	
	// (re-)set state
	public:
	void operator ()( const double z ) ;
	void reset() {
		boost::lock_guard< boost::mutex > lock( this->mutex ) ;
		this->resetBase() ;
	}
	
	// additional members
	protected:
	double tau ;
} ;

// "Kalman-like" 1D filter
class Tracker1DKalman : public Tracker1DBase {
	
	// c'tor
	public:
	Tracker1DKalman( const double qArg = 1.0,
	                 const double rArg = 1.0 ) :
		p( NAN  ),  // state variance
		q( qArg ),  // process noise per second(!)
		r( rArg ) { // measurement noise
		HAWAII_ERROR_CONDITIONAL( this->q < 0.0 || this->r < 0.0, "invalid process or measurement noise" ) ;
	}
	
	// copy construction and assignment using, but not copying the mutex
	public:
	Tracker1DKalman( const Tracker1DKalman& that ) { *this = that ; }
	Tracker1DKalman& operator =( const Tracker1DKalman& that ) ;
	
	// (re-)set state
	public:
	void operator ()( const double z ) { this->update( z ) ; }
	void reset() {
		boost::lock_guard< boost::mutex > lock( this->mutex ) ;
		this->p = NAN ;
		this->resetBase() ;
	}
	
	// specific interface
	// user note: Calling "update()" with an additional measurement noise will use it for that call only.
	public:
	void predict( double& xRet ) const { xRet = *this ; }
	void predict( double& xRet,
	              double& pRet ) const ;
	void update( const double z ) { this->update( z, this->r ) ; }
	void update( const double z,
	             const double rArg ) ;
	
	// additional members
	protected:
	double p ;
	double q ;
	double r ;
} ;

} } // namespaces "hawaii::common"
