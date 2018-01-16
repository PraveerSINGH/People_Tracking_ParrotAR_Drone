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

#include "hawaii/common/timer.h"
#include <chrono>

// helpers(s) only used here
namespace {
	
	// clock to be used, compile-time check types
	// developer note: Similarly to the "pimpl" pattern, the header must not see C++11's "std::chrono" to maintain NVCC 
	//                 compatibility. The alternative "boost::chrono" is not yet available in the commonly used version 
	//                 1.46. Therefore, a compile-time check of the header's manual time representation definition is 
	//                 needed here.
	#if defined ( __GNUC__ ) && defined ( __GNUC_MINOR__ ) && ( ( ( __GNUC__ ) * 100 ) + ( __GNUC_MINOR__ ) ) < 407
		typedef std::chrono::monotonic_clock Clock ;
	#else
		typedef std::chrono::steady_clock    Clock ;
	#endif
	static_assert( std::is_same< Clock::rep, hawaii::common::Timer::Rep >::value,
	               "\"hawaii::common::Timer::Rep\" type inconsistent with used clock" ) ;
	
	// instance for global functions
	hawaii::common::Timer timer( false ) ;
}

namespace hawaii {

// global functions for timing one thing at a time(!)
void   tic()    {        timer.tic()    ; }
double toc()    { return timer.toc()    ; }
double tocTic() { return timer.tocTic() ; }

namespace common {

// "Timer": class for simultaneously timing multiple things via individual instances
// ---------------------------------------------------------------------------------

// initialize static constants
const double     Timer::factorRepToMilliseconds = 1000.0 * (double)Clock::period::num / (double)Clock::period::den ;
const Timer::Rep Timer::startNAN                = std::numeric_limits< Timer::Rep >::min()                         ;

// helper to get current time representation
Timer::Rep Timer::timeSinceEpoch() {
	return Clock::now().time_since_epoch().count() ;
}

} } // namespaces "hawaii::common"
