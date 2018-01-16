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


// thread-safe replacements for "std::{cout,cerr}"
// ===============================================
// "std::{cout,cerr}" may print after each "<<" and therefore interleave inputs from threads using it concurrently. 
// This is avoided within "hawaii::{cout,cerr}" by only printing after the last "<<".
// user note: Newer versions of the GNU Standard C++ Library may not show this problem any more when concurrently using
//            either "std::cout" or "std::cerr", but using both still shows the above issue (at least with GCC 4.6.3). 

#pragma once

#include <iostream>
#include <sstream>
#include <memory>

namespace hawaii {
namespace common {

// means what it says!
namespace internal {

// class to buffer inputs before printing them at once
class OstreamBufferPrinter {
	
	// append to the buffer
	public:
	template< typename Printable >
	OstreamBufferPrinter& operator <<( const Printable& printableArg ) {
		*this->buffer << printableArg ;
		return *this ;
	}
	
	// require target stream to be set
	public:
	OstreamBufferPrinter( std::ostream& targetArg ) :
		target( targetArg                ),
		buffer( new std::ostringstream() ) {
	}
	
	// wait until d'tor before printing buffer at once
	public:
	~OstreamBufferPrinter() {
		std::cout << std::flush ;
		std::cerr << std::flush ;
		this->target << this->buffer->str() << std::flush ;
	}
	
	// buffer to collect data to be printed, target to finally print to
	protected:
	std::ostream&                       target ;
	std::auto_ptr< std::ostringstream > buffer ;
	
} ; // class "OstreamBufferPrinter"

// thread safe-replacement for "std::ostream", the type of "std::{cout,cerr}"
class OstreamInitializer {
	
	// create an individual buffer/printer each time "operator <<()" is called
	public:
	template< typename Printable >
	OstreamBufferPrinter operator <<( const Printable& printableArg ) const {
		OstreamBufferPrinter ret( this->target ) ;
		ret << printableArg ;
		return ret ;
	}
	
	// require target stream to be set
	public:
	OstreamInitializer( std::ostream& targetArg ) :
		target( targetArg ) {
	}
	
	// target to finally print to
	protected:
	std::ostream& target ;
	
} ; // class "OstreamInitializer"

} } // namespaces "common::internal"

// global instances
extern hawaii::common::internal::OstreamInitializer cout ;
extern hawaii::common::internal::OstreamInitializer cerr ;
extern const std::string                            endl ;

} // namespace "hawaii"
