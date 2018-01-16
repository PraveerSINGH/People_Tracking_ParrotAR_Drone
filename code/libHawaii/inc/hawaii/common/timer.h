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


// measure time spans in a lock-free way
// =====================================

#pragma once

#include <stdint.h>

namespace hawaii {

// global functions for timing one thing at a time(!) without instantiating the class below
void   tic()    ; // (re-)start global timer
double toc()    ; // get milliseconds since last "tic()"
double tocTic() ; // both of the above

namespace common {

// class for simultaneously timing multiple things via individual instances
class Timer {
	
	// time representation, helpers to get and convert it
	public:
	typedef int64_t Rep ;
	static Rep timeSinceEpoch() ;
	static const double factorRepToMilliseconds ;
	
	// time at which "tic()" was last called, special value to detect un-initialized timer
	protected:
	Rep start ;
	static const Rep startNAN ;
	
	// c'tor with option to directly start running
	public:
	Timer( const bool ticArg = true ) :
		start( ticArg ? Timer::timeSinceEpoch() : Timer::startNAN ) {
	}
	
	// above global functions' equivalents
	public:
	void tic() {
		this->start = Timer::timeSinceEpoch() ;
	}
	double toc() const {
		return ( Timer::timeSinceEpoch() - this->start ) * Timer::factorRepToMilliseconds ;
	}
	double tocTic() {
		
		// atomic operation to allow concurrent usage
		Rep end, startPrev ;
		do { end       = Timer::timeSinceEpoch() ;
		     startPrev = this->start             ; 
		} while( startPrev != __sync_val_compare_and_swap( &this->start, startPrev, end ) ) ;
		return ( end - startPrev ) * Timer::factorRepToMilliseconds ;
	}
	
	// check timer's state, stop it
	public:
	bool isRunning() const { return this->start != Timer::startNAN ; }
	void reset()           {        this->start  = Timer::startNAN ; }
} ;

} } // namespaces "hawaii::common"
