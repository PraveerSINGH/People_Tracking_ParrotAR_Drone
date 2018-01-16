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


// include CUDA if available, GPU initialization and specific helpers
// ==================================================================

#pragma once

#include "hawaii/GPU/nvccFixC++11.h"
#include "hawaii/common/types.h"
#include "hawaii/GPU/nvccFixHeaderAbove.h"
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/core/version.hpp>
#ifndef HAWAII_NO_CUDA
	#if   ( CV_VERSION_EPOCH >  2 )                                                                 \
	 || ( ( CV_VERSION_EPOCH == 2 ) && ( ( CV_VERSION_MAJOR >  4 )                                  \
	                                  || ( CV_VERSION_MAJOR == 4 ) && ( CV_VERSION_MINOR >= 9 ) ) )
		#include <opencv2/core/gpu_stream_accessor.hpp>
	#else
		#include <opencv2/gpu/stream_accessor.hpp>
	#endif
#endif
#include "hawaii/GPU/nvccFixHeaderBelow.h"

// include CUDA only if available and enabled...
#ifndef HAWAII_NO_CUDA
	
	// C for CUDA runtime API
	#include <cuda_runtime.h>
	
	// useful helper functions from the CUDA SDK samples
	#ifndef __CUDACC__
		#pragma GCC diagnostic push
		#pragma GCC diagnostic ignored "-Wswitch"
		#pragma GCC diagnostic ignored "-Wunused-function"
	#endif
	#include <helper_cuda.h>
	#ifndef __CUDACC__
		#pragma GCC diagnostic pop
	#endif
	
// replace generally used types otherwise
#else
	
	// allow at least in declarations
	typedef void* cudaStream_t ;
	typedef void* cudaEvent_t  ;
	
	// generates e.g. "struct uchar2 { uchar x, y ; }"
	#define CUDA_VECTOR_1( TYPE ) struct TYPE ## 1 { TYPE x          ; }
	#define CUDA_VECTOR_2( TYPE ) struct TYPE ## 2 { TYPE x, y       ; }
	#define CUDA_VECTOR_3( TYPE ) struct TYPE ## 3 { TYPE x, y, z    ; }
	#define CUDA_VECTOR_4( TYPE ) struct TYPE ## 4 { TYPE x, y, z, w ; }
	CUDA_VECTOR_1( uchar     ) ;
	CUDA_VECTOR_2( uchar     ) ;
	CUDA_VECTOR_3( uchar     ) ;
	CUDA_VECTOR_4( uchar     ) ;
	CUDA_VECTOR_1(  char     ) ;
	CUDA_VECTOR_2(  char     ) ;
	CUDA_VECTOR_3(  char     ) ;
	CUDA_VECTOR_4(  char     ) ;
	CUDA_VECTOR_1( ushort    ) ;
	CUDA_VECTOR_2( ushort    ) ;
	CUDA_VECTOR_3( ushort    ) ;
	CUDA_VECTOR_4( ushort    ) ;
	CUDA_VECTOR_1(  short    ) ;
	CUDA_VECTOR_2(  short    ) ;
	CUDA_VECTOR_3(  short    ) ;
	CUDA_VECTOR_4(  short    ) ;
	CUDA_VECTOR_1( uint      ) ;
	CUDA_VECTOR_2( uint      ) ;
	CUDA_VECTOR_3( uint      ) ;
	CUDA_VECTOR_4( uint      ) ;
	CUDA_VECTOR_1(  int      ) ;
	CUDA_VECTOR_2(  int      ) ;
	CUDA_VECTOR_3(  int      ) ;
	CUDA_VECTOR_4(  int      ) ;
	CUDA_VECTOR_1( ulong     ) ;
	CUDA_VECTOR_2( ulong     ) ;
	CUDA_VECTOR_3( ulong     ) ;
	CUDA_VECTOR_4( ulong     ) ;
	CUDA_VECTOR_1(  long     ) ;
	CUDA_VECTOR_2(  long     ) ;
	CUDA_VECTOR_3(  long     ) ;
	CUDA_VECTOR_4(  long     ) ;
	CUDA_VECTOR_1( ulonglong ) ;
	CUDA_VECTOR_2( ulonglong ) ;
	CUDA_VECTOR_3( ulonglong ) ;
	CUDA_VECTOR_4( ulonglong ) ;
	CUDA_VECTOR_1(  longlong ) ;
	CUDA_VECTOR_2(  longlong ) ;
	CUDA_VECTOR_3(  longlong ) ;
	CUDA_VECTOR_4(  longlong ) ;
	CUDA_VECTOR_1( float     ) ;
	CUDA_VECTOR_2( float     ) ;
	CUDA_VECTOR_3( float     ) ;
	CUDA_VECTOR_4( float     ) ;
	CUDA_VECTOR_1( double    ) ;
	CUDA_VECTOR_2( double    ) ;
	CUDA_VECTOR_3( double    ) ;
	CUDA_VECTOR_4( double    ) ;
	#undef CUDA_VECTOR_1
	#undef CUDA_VECTOR_2
	#undef CUDA_VECTOR_3
	#undef CUDA_VECTOR_4
	
	// another "uint3" for configuring kernel launches
	struct dim3 {
		unsigned int x, y, z ;
		dim3( unsigned int xArg = 1,
		      unsigned int yArg = 1,
		      unsigned int zArg = 1 ) :
			x( xArg ),
			y( yArg ),
			z( zArg ) {
		}
		dim3( uint3 that ) :
			x( that.x ),
			y( that.y ),
			z( that.z ) {
		}
		operator uint3() const {
			uint3 ret ;
			ret.x = this->x ;
			ret.y = this->y ;
			ret.z = this->z ;
			return ret ;
		}
	} ;
	
#endif // #ifdef-#else "CUDA available?"

namespace hawaii {
namespace GPU {

// initialize all available GPUs with the following preferences:
// - be able to map page-locked host memory into the device address space
// - yield the controlling CPU thread when synchronizing (instead of potentially having it spin)
// - enable peer-to-peer communication with all other GPUs
// user note: In case of "cudaErrorSetOnActiveProcess" you are calling this too late within your application.
// developer note: If this wasn't a static library, this could be done automatically in the c'tor of a singleton.
void init() ;
void reset() ;

// grid for launching image processing CUDA kernels: Each thread computes 1 pixel vertically and "colsPerThread" pixels 
//                                                   horizontally
dim3 grid( const int  rows,
           const int  cols,
           const dim3 blockDim,
           const uint colsPerThread = 1 ) ;
inline dim3 grid( const cv::Size size,
                  const dim3 blockDim,
                  const uint colsPerThread = 1 ) {
	return hawaii::GPU::grid( size.height, size.width, blockDim, colsPerThread ) ;
}

// replacement for "cv::gpu::Stream::Null()": does not cause run-time errors due to missing compile-time GPU support
extern cv::gpu::Stream& streamNull ;

} } // namespaces "hawaii::GPU"
