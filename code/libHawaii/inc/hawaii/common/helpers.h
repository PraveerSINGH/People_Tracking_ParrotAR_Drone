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


// various small helpers
// =====================

#pragma once

#include "hawaii/common/ostream.h"
#include "hawaii/GPU/nvccFixC++11.h"
#include "hawaii/GPU/nvccFixHeaderAbove.h"
#include <opencv2/highgui/highgui.hpp>
#include "hawaii/GPU/nvccFixHeaderBelow.h"

// print a variable or show an image including its name without having to type it twice
#define HAWAII_PRINT(  var   ) { hawaii::cout << #var " = " << ( var ) << hawaii::endl ; }
#define HAWAII_IMSHOW( cvMat ) { cv::imshow( #cvMat, cvMat ) ; }

namespace hawaii {

// shared pointer to also hold stack variables: Construction or assignment from a reference manually increases the 
//                                              reference count by 1 in order to prevent double deletion.
template< typename Elem >
class Ptr : public cv::Ptr< Elem > {
	
	// c'tors (see above)
	public:
	Ptr(                             ) : cv::Ptr< Elem >(      ) {                  }
	Ptr(                Elem*   ptr  ) : cv::Ptr< Elem >( ptr  ) {                  }
	Ptr(                Elem&   obj  ) : cv::Ptr< Elem >( &obj ) { this->addref() ; }
	Ptr( const cv::Ptr< Elem >& that ) : cv::Ptr< Elem >( that ) {                  }
	
	// assignment operators (see above)
	Ptr& operator =(                Elem*   ptr  ) { *this = hawaii::Ptr< Elem >( ptr  ) ; return *this ; }
	Ptr& operator =(                Elem&   obj  ) { *this = hawaii::Ptr< Elem >( obj  ) ; return *this ; }
	Ptr& operator =( const cv::Ptr< Elem >& that ) { *this = hawaii::Ptr< Elem >( that ) ; return *this ; }
	
	// added whole element access like with "boost::*_ptr"
	Elem& get() const { return *this->obj ; }
	Elem& operator *() const { return this->get() ; }
} ;

// delete only non-null objects/arrays and - if possible - set pointer to null afterwards
template< typename T > inline void deleteObject( T const * __restrict       & objPtr ) { if( objPtr ) { delete   objPtr ; objPtr = nullptr ; } }
template< typename T > inline void deleteObject( T const * __restrict const & objPtr ) { if( objPtr ) { delete   objPtr ;                    } }
template< typename T > inline void deleteArray(  T const * __restrict       & arrPtr ) { if( arrPtr ) { delete[] arrPtr ; arrPtr = nullptr ; } }
template< typename T > inline void deleteArray(  T const * __restrict const & arrPtr ) { if( arrPtr ) { delete[] arrPtr ;                    } }

} // namespace "hawaii"
