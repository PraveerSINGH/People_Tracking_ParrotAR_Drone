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


// find minimum or maximum of a variable number of arguments with different yet compatible types
// =============================================================================================

#pragma once

#include "hawaii/GPU/nvccProhibit.h"
#include <type_traits>
#include <utility>

namespace hawaii {

// adapted from "stackoverflow.com/a/7472115", added rvalue references and forwarding
template< typename T > T min( T&& t ) { return std::forward< T >( t ) ; } // allow template termination
template< typename T > T max( T&& t ) { return std::forward< T >( t ) ; } // "
template< typename T1, typename T2, typename ... Tn, typename C = typename std::common_type< T1, T2, Tn ... >::type >
C min( T1&& t1, T2&& t2, Tn&& ... tn ) {
	return std::forward< C >( hawaii::min( std::forward< C >( t1 < t2 ? t1 : t2 ),
	                                       std::forward< C >( tn                ) ... ) ) ;
}
template< typename T1, typename T2, typename ... Tn, typename C = typename std::common_type< T1, T2, Tn ... >::type >
C max( T1&& t1, T2&& t2, Tn&& ... tn ) {
	return std::forward< C >( hawaii::max( std::forward< C >( t1 > t2 ? t1 : t2 ),
	                                       std::forward< C >( tn                ) ... ) ) ;
}

} // namespace "hawaii"
