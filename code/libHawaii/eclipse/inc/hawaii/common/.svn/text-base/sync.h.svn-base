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


// synchronization-related types
// =============================

#pragma once

namespace hawaii {
namespace common {

// replacement for "std::pair" to remind you to obtain the mutex "first" and access the data "second"
// developer note #1: Using the original "std::pair" fails to compile by demanding a copy c'tor from the mutex with the 
//                    combination of GCC 4.7 and Boost 1.49 only. However, these are common default versions e.g. in 
//                    Ubuntu 12.10 and 13.04.
// developer note #2: Less "std::pair"-like and more enforcing alternatives use...
//                    a) Boost's well-known mutex interface and run-time checks:
//                    - "www.drdobbs.com/windows/associate-mutexes-with-data-to-prevent-r/224701827"
//                    b) mandatory own accessor types:
//                    - "www.mr-edd.co.uk/blog/associate_mutexes_to_prevent_races"
//                    - "codereview.stackexchange.com/questions/15632/force-locking-for-thread-safety"
//                    - "www.drdobbs.com/cpp/enforcing-correct-mutex-usage-with-synch/225200269"
template< typename Mutex, typename Data >
class SyncPair {
	
	// names like in "std::pair"
	public:
	typedef Mutex first_type  ; mutable Mutex first  ;
	typedef Data  second_type ;         Data  second ;
	
	// c'tors from "Data" or whole "SyncPair"
	SyncPair() {}
	template<                    typename DataArg > SyncPair( const                     DataArg&  secondArg   ) { *this = secondArg   ; }
	template< typename MutexArg, typename DataArg >	SyncPair( const SyncPair< MutexArg, DataArg > syncPairArg ) { *this = syncPairArg ; }
	
	// assignment from "Data" or whole "SyncPair", prevent deadlock on self-assignment
	template< typename DataArg >
	SyncPair< Mutex, Data >& operator =( const DataArg& secondArg ) {
		this->first.lock() ;
		this->second = secondArg ;
		this->first.unlock() ;
		return *this ;
	}
	template< typename MutexArg, typename DataArg >
	SyncPair< Mutex, Data >& operator =( const SyncPair< MutexArg, DataArg > syncPairArg ) {
		if( this != &syncPairArg ) {
			this->first.lock() ;
			syncPairArg.first.lock() ;
			this->second = syncPairArg.second ;
			this->first.unlock() ;
			syncPairArg.first.unlock() ;
		}
		return *this ;
	}

} ; // class template "SyncPair"

} } // namespaces "hawaii::common"
