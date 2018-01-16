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

#include "hawaii/GPU/CUDA.h"
#include "hawaii/GPU/pool.h"
#include "hawaii/common/error.h"

namespace hawaii {
namespace GPU {

// initialize all available GPUs with the following preferences:
// - be able to map page-locked host memory into the device address space
// - yield the controlling CPU thread when synchronizing (instead of potentially having it spin)
// - enable peer-to-peer communication with all other GPUs
void init() {
	
	// check compile-time CUDA support
	#ifndef HAWAII_NO_CUDA
		
		// count GPUs (and do nothing if there are none), save caller's active GPU
		int GPUs ; if( cudaSuccess != cudaGetDeviceCount( &GPUs ) ) { GPUs = 0 ; }
		int GPUCall = -1 ; if( GPUs > 0 ) { HAWAII_ERROR_CHECK_CUDA( cudaGetDevice( &GPUCall ) ) ; }
		
		// set flags for above items #1 and #2 on all GPUs
		// developer note: The heuristic of the default setting "cudaDeviceScheduleAuto" does not account for CPU load.
		const unsigned int flags = cudaDeviceMapHost | cudaDeviceScheduleSpin ; // TODO evaluate "cudaDeviceSchedule{Auto,Spin,Yield}"
		for( int GPU = 0 ; GPU < GPUs ; ++GPU ) {
			HAWAII_ERROR_CHECK_CUDA( cudaSetDevice( GPU ) ) ;
			HAWAII_ERROR_CHECK_CUDA( cudaSetDeviceFlags( flags ) ) ;
		}
		
		// enable peer-to-peer access between all GPUs supporting it
		for( int GPU = 0 ; GPU < GPUs ; ++GPU ) {
			HAWAII_ERROR_CHECK_CUDA( cudaSetDevice( GPU ) ) ;
			for( int GPUPeer = 0 ; GPUPeer < GPUs ; ++GPUPeer ) {
				if( GPU != GPUPeer ) {
					int supported ; HAWAII_ERROR_CHECK_CUDA( cudaDeviceCanAccessPeer( &supported, GPU, GPUPeer ) ) ;
					if( supported ) {
						HAWAII_ERROR_CHECK_CUDA( cudaDeviceEnablePeerAccess( GPUPeer, 0 ) ) ;
					}
				}
			}
		}
		
		// restore caller's active GPU
		if( GPUCall >= 0 ) { HAWAII_ERROR_CHECK_CUDA( cudaSetDevice( GPUCall ) ) ; }
		
	#endif
}
void reset() {
	
	// check compile-time CUDA support
	#ifndef HAWAII_NO_CUDA
		
		// clear global memory pools for all GPUs and allocation sizes
		hawaii::GPU::memoryPool.clear() ;
		
		// count GPUs (and do nothing if there are none), save caller's active GPU
		int GPUs ; if( cudaSuccess != cudaGetDeviceCount( &GPUs ) ) { GPUs = 0 ; }
		int GPUCall = -1 ; if( GPUs > 0 ) { HAWAII_ERROR_CHECK_CUDA( cudaGetDevice( &GPUCall ) ) ; }
		
		// activate and reset all GPUs
		for( int GPU = 0 ; GPU < GPUs ; ++GPU ) {
			cudaSetDevice( GPU ) ;
			cudaDeviceReset() ;
		}
		
		// restore caller's active GPU
		if( GPUCall >= 0 ) { HAWAII_ERROR_CHECK_CUDA( cudaSetDevice( GPUCall ) ) ; }
		
	#endif
}

// grid for launching image processing CUDA kernels: Each thread computes 1 pixel vertically and "colsPerThread" pixels 
//                                                   horizontally
dim3 grid( const int  rows,
           const int  cols,
           const dim3 blockDim,
           const uint colsPerThread ) {
	
	// check input
	HAWAII_ERROR_CONDITIONAL( rows < 1 || cols < 1, "Number of rows and columns must be positive." ) ;
	
	// compute next-largest grid to cover all rows and columns
	const uint colThreads = ( cols + colsPerThread-1 ) / colsPerThread ;
	return dim3( ( colThreads + blockDim.x-1 ) / blockDim.x,
	             ( rows       + blockDim.y-1 ) / blockDim.y ) ;
}

// replacement for "cv::gpu::Stream::Null()": does not cause run-time errors due to missing compile-time GPU support
namespace {
	#if   ( CV_VERSION_EPOCH >  2 )                                                                 \
	 || ( ( CV_VERSION_EPOCH == 2 ) && ( ( CV_VERSION_MAJOR >  4 )                                  \
												 || ( CV_VERSION_MAJOR == 4 ) && ( CV_VERSION_MINOR >= 9 ) ) )
		struct StreamDummy {
			struct Impl {
				void* stream ;
				Impl() : stream( nullptr ) {}
			} ;
			cv::Ptr< Impl > impl_ ;
			StreamDummy() : impl_( new Impl ) {}
		} ;
	#else
		struct StreamDummy {
			void* impl ;
			StreamDummy() : impl( nullptr ) {}
		} ;
	#endif
	const StreamDummy streamDummy ;
}
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
cv::gpu::Stream& streamNull = *(cv::gpu::Stream*)&streamDummy ;
#pragma GCC diagnostic pop

} } // namespaces "hawaii::GPU"
