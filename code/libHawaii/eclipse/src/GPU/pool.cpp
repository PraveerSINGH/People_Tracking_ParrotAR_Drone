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


// pools to avoid the latency and synchronization caused by "cudaMalloc*()" and "cudaHostAlloc()"
// ==============================================================================================

#include "hawaii/GPU/pool.h"
#include "hawaii/GPU/CUDA.h"
#include "hawaii/common/error.h"

// helper(s) only used here
namespace {
	
	// really allocate memory: only called in case it is not available from a pool, always creates a single row of 
	//                         "unsigned char" elements 
	void allocateReally( cv::gpu::GpuMat& gpuMatArg, const size_t bytes ) {
		
		// check compile-time GPU support
		HAWAII_ERROR_NO_CUDA ;
		#ifndef HAWAII_NO_CUDA
			
			// wrap OpenCV
			gpuMatArg.create( 1, bytes, CV_8UC1 ) ;
			
		#else
			
			// avoid compiler warnings
			(void)gpuMatArg ;
			(void)bytes     ;
			
		#endif
	}
	void allocateReally( cv::gpu::CudaMem& cudaMemArg, const size_t bytes ) {
		
		// check compile-time GPU support
		HAWAII_ERROR_NO_CUDA ;
		#ifndef HAWAII_NO_CUDA
			
			// (dont't) wrap OpenCV
			// developer note #1: The code below uses its own macros because the allocation type enum has been changed 
			//                    between OpenCV 2.4.5 and 2.4.9. On that occasion, OpenCV's maintainers did not manage to 
			//                    fix the issues below however...
			#if   ( CV_VERSION_EPOCH >  2 )                                                                 \
			 || ( ( CV_VERSION_EPOCH == 2 ) && ( ( CV_VERSION_MAJOR >  4 )                                  \
			                                  || ( CV_VERSION_MAJOR == 4 ) && ( CV_VERSION_MINOR >= 9 ) ) )
				#define CUDAMEM_PAGE_LOCKED   cv::gpu::CudaMem::PAGE_LOCKED
				#define CUDAMEM_MAPPED        cv::gpu::CudaMem::SHARED
				#define CUDAMEM_WRITECOMBINED cv::gpu::CudaMem::WRITE_COMBINED
			#else
				#define CUDAMEM_PAGE_LOCKED   cv::gpu::CudaMem::ALLOC_PAGE_LOCKED
				#define CUDAMEM_MAPPED        cv::gpu::CudaMem::ALLOC_ZEROCOPY
				#define CUDAMEM_WRITECOMBINED cv::gpu::CudaMem::ALLOC_WRITE_COMBINED
			#endif
			// developer note #2: The line below does not work due to an unnecessarily limited implementation within 
			//                    OpenCV (as of June 19th 2013): Although the three flags actually are orthogonal, only 
			//                    one may be selected at a time - enabling either fast copying between CPU and GPU or 
			//                    mapping CPU memory into the GPU address space, but not both. Also "cv::gpu::CudaMem::
			//                    (ALLOC_)PAGE_LOCKED" maps to "cudaHostAllocDefault" instead of "cudaHostAllocPortable", 
			//                    so the allocated memory cannot be used in other CUDA contexts.
			#if 0
				instance.create( 1, bytes, CV_8UC1, CUDAMEM_PAGE_LOCKED | CUDAMEM_MAPPED ) ;
			#else
				cudaMemArg.release() ;
				HAWAII_ERROR_CHECK_CUDA( cudaHostAlloc( &cudaMemArg.data, bytes, cudaHostAllocPortable | cudaHostAllocMapped ) ) ;
				cudaMemArg.datastart = cudaMemArg.data         ;
				cudaMemArg.dataend   = cudaMemArg.data + bytes ;
				cudaMemArg.rows = 1     ;
				cudaMemArg.cols = bytes ;
				cudaMemArg.step = bytes ;
				cudaMemArg.flags = cv::Mat::MAGIC_VAL | CV_8UC1 | cv::Mat::CONTINUOUS_FLAG ;
				// developer note #3: As if the issue above wasn't enough, "cv::gpu::CudaMem::createGpuMatHeader()" checks 
				//                    "cv::gpu::CudaMem::alloc_type" via "==" instead of "&", so it must not be set correctly 
				//                    on purpose.
				#if 0
					cudaMemArg.alloc_type = CUDAMEM_PAGE_LOCKED | CUDAMEM_MAPPED ;
				#else
					cudaMemArg.alloc_type = CUDAMEM_MAPPED ;
				#endif
				cudaMemArg.refcount = (int*)cv::fastMalloc( sizeof( *cudaMemArg.refcount ) ) ;
				*cudaMemArg.refcount = 1 ;
			#endif
			
		#else
			
			// avoid compiler warnings
			(void)cudaMemArg ;
			(void)bytes      ;
			
		#endif
	}
	
	// try to get an instance with the requested properties from the corresponding pool, allocate on failure
	template< typename GpuMatOrCudaMem >
	void allocateFromPoolSet( hawaii::common::SyncPair< boost::mutex, std::vector< GpuMatOrCudaMem > >* const pools,
	                          GpuMatOrCudaMem& instanceArg,
	                          const int rows, const int cols, const int type ) {
		
		// full size of required memory including padding for biggest alignment among all GPUs
		const size_t elem  = CV_ELEM_SIZE( type ) ;
		const size_t align = hawaii::GPUAlignmentMax() ;
		const size_t step  = ( ( elem * cols + align - 1 ) / align ) * align ;
		const size_t total = step * rows ;
		
		// find corresponding pool and its instance size
		const size_t poolIdx = 64 - __builtin_clzll( total - 1 ) ;
		const size_t bytes   = 1 << poolIdx ;
		auto& pool = pools[ poolIdx ] ;
		
		// search pool for unused instance
		bool found = false ;
		pool.first.lock() ;
		for( auto& instancePool : pool.second ) {
			
			// reference count of 1 => pool itself is the only user, so take that instance and skip further search
			if( *instancePool.refcount == 1 ) {
				instanceArg = instancePool ;
				found = true ;
				break ;
			}
		}
		
		// allocate only if no unused instance found in pool
		if( !found ) {
			allocateReally( instanceArg, bytes ) ;
			pool.second.push_back( instanceArg ) ;
		}
		pool.first.unlock() ;
		
		// adjust instance to meet requested properties
		instanceArg.rows    = rows ;
		instanceArg.cols    = cols ;
		instanceArg.step    = step ;
		instanceArg.flags   = cv::Mat::MAGIC_VAL | type | cv::Mat::CONTINUOUS_FLAG * ( rows == 1 || elem * cols == step ) ;
		instanceArg.dataend = instanceArg.datastart + rows * step ;
		
	} // function "allocateFromPoolSet()"
	
	// stringify status of one set of size-binned pools
	template< typename GpuMatOrCudaMem >
	std::string stringifyPoolSet( const hawaii::common::SyncPair< boost::mutex, std::vector< GpuMatOrCudaMem > >* const pools ) {
		
		// buffer to collect text in
		std::ostringstream buf ;
		
		// go through size bins
		for( size_t poolIdx = 0 ; poolIdx < 64 ; ++poolIdx ) {
			auto& pool = pools[ poolIdx ] ;
			
			// count number of available and lent instances
			size_t avlb = 0, lent = 0 ;
			for( const auto& instance : pool.second ) { *instance.refcount > 1 ? ++lent : ++avlb ; }
				
			// only print if pool not empty
			if( avlb || lent ) {
				const size_t bytesMax = 1 << poolIdx ; 
				buf << "\n" << "between " << bytesMax / 2 + 1 << " and " << bytesMax << " bytes: " << avlb << " available, " << lent << " lent" ;
			}
		}
		
		// return to allow embedding into outside string stream 
		return buf.str() ;
	}	

} // anonymous namespace
				
namespace hawaii {
namespace GPU {

// replacement for "cv::gpu::GpuMat::create()": modifies the given instance only if necessary,  (see above)
void MemoryPool::operator ()( cv::gpu::GpuMat& gpuMatArg,
                              const int rows, const int cols, const int type,
                              MemoryType memory ) {
	
	// check input and compile-time GPU support
	HAWAII_ERROR_CONDITIONAL( rows < 1 || cols < 1, "Number of rows and columns must be at least 1." ) ;
	HAWAII_ERROR_NO_CUDA ;
	#ifndef HAWAII_NO_CUDA
		
		// find active GPU's preferred memory type
		const int GPU = cv::gpu::getDevice() ;
		if( memory == hawaii::GPU::memoryAuto ) {
			memory = hawaii::GPUMemoryOwn( GPU ) ? hawaii::GPU::memoryOwn : hawaii::GPU::memoryMapped ;
		}
		
		// modification necessary?
		// developer note #1: unified addressing (requires 64-bit OS and "Fermi" or later GPU) needed to determine 
		//                    anything about given GPU pointer
		// developer note #2: When modifying this, keep in mind that almost the same code also exists in the 
		//                    implementation of "hawaii::GPU::AutoMat::writeGPU( const int rows, ... )" within 
		//                    "libHawaii/src/GPU/autoMat.cpp".
		if( GPUUnifiedAddressing( GPU ) == true
		 && gpuMatArg.data              != nullptr ) {
			
			// correct properties?
			if( gpuMatArg.rows   == rows
			 && gpuMatArg.cols   == cols
			 && gpuMatArg.type() == type ) {
				
				// correct memory type and location
				cudaPointerAttributes attr ; HAWAII_ERROR_CHECK_CUDA( cudaPointerGetAttributes( &attr, gpuMatArg.data ) ) ;
				if( attr.devicePointer != nullptr
				 && ( ( memory          == hawaii::GPU::memoryOwn
				     && attr.memoryType == cudaMemoryTypeDevice
				     && attr.device     == GPU )
				   || ( memory          == hawaii::GPU::memoryMapped
				     && attr.memoryType == cudaMemoryTypeHost ) ) ) {
					
					// leave instance as it is 
					return ;
				}
			}
		}
		
		// choose pool depending on selected actual memory location
		switch( memory ) {
			case hawaii::GPU::memoryOwn: {
				allocateFromPoolSet( this->poolsGpuMat[ GPU ], gpuMatArg, rows, cols, type ) ;
				break ;
			}
			case hawaii::GPU::memoryMapped: {
				cv::gpu::CudaMem cudaMem ;
				allocateFromPoolSet( this->poolsCudaMem, cudaMem, rows, cols, type ) ;
				CV_XADD( cudaMem.refcount, 1 ) ;
				gpuMatArg          = cudaMem.createGpuMatHeader() ;
				gpuMatArg.refcount = cudaMem.refcount ;
				break ;
			}
			case hawaii::GPU::memoryAuto:
			default: {
				HAWAII_ERROR( "This line should never have been reached." ) ;
				break ;
			}
		}
		
	#else
		
		// avoid compiler warnings
		(void)gpuMatArg ;
		(void)rows      ;
		(void)cols      ;
		(void)type      ;
		(void)memory    ;
		
	#endif
}

// replacement for "cv::gpu::CudaMem::create()": modifies the given instance only if necessary
void MemoryPool::operator ()( cv::gpu::CudaMem& cudaMemArg,
                              const int rows, const int cols, const int type ) {
	
	// check input and compile-time GPU support
	HAWAII_ERROR_CONDITIONAL( rows < 1 || cols < 1, "Number of rows and columns must be at least 1." ) ;
	HAWAII_ERROR_NO_CUDA ;
	#ifndef HAWAII_NO_CUDA
		
		// modification necessary?
		if( cudaMemArg.data != nullptr ) {
			
			// correct properties?
			if( cudaMemArg.rows   == rows
			 && cudaMemArg.cols   == cols
			 && cudaMemArg.type() == type ) {
				
				// correct allocation flags?
				unsigned int flags ;
				HAWAII_ERROR_CHECK_CUDA( cudaHostGetFlags( &flags, cudaMemArg.data ) ) ;
				if( flags & cudaHostAllocMapped
				 && flags & cudaHostAllocPortable ) {
					
					// leave instance as it is 
					return ;
				}
			}
		}
		
		// use pool
		allocateFromPoolSet( this->poolsCudaMem, cudaMemArg, rows, cols, type ) ;
		
	#else
		
		// avoid compiler warnings
		(void)cudaMemArg ;
		(void)rows       ;
		(void)cols       ;
		(void)type       ;
		
	#endif
}

// print number of available and lent items of each (non-empty) internal pool
void MemoryPool::printStatus() const {
	
	// buffer to collect text in, ensure at least one empty line above 
	std::ostringstream buf ;
	buf << "\n" ;
	
	// "cv::gpu::GpuMat" pools per GPU
	for( int GPU = 0 ; GPU < hawaii::GPUs ; ++GPU ) {
		const std::string stringifiedPoolSet = stringifyPoolSet( this->poolsGpuMat[ GPU ] ) ;
		if( !stringifiedPoolSet.empty() ) {
			buf << "\n" << "pools for own memory on GPU #" << GPU << ":" << stringifiedPoolSet ;
		}
	}
	
	// "cv::gpu::CudaMem" for all GPUs
	const std::string stringifiedPoolSet = stringifyPoolSet( this->poolsCudaMem ) ;
	if( !stringifiedPoolSet.empty() ) {
		buf << "\n" << "pools for page-locked GPU memory:" << stringifiedPoolSet ;
	}
	
	// print buffer, ensure at least one empty line below 
	buf << "\n\n" ;
	hawaii::cout << buf.str() ;
}

// d'tor
void MemoryPool::clear() {
	
	// actual GPU memory
	if( hawaii::GPUs > 0 ) {
		
		// save caller's active GPU
		const int GPUCall = cv::gpu::getDevice() ;
		
		// activate each GPU before emptying its pools
		for( int GPU = 0 ; GPU < hawaii::GPUs ; ++GPU ) {
			cv::gpu::setDevice( GPU ) ;
			for( size_t poolIdx = 0 ; poolIdx < 64 ; ++poolIdx ) {
				auto& pool = this->poolsGpuMat[ GPU ][ poolIdx ] ;
				boost::lock_guard< boost::mutex > lock( pool.first ) ;
				pool.second.clear() ;
			}
		}
		
		// restore caller's previously active GPU
		cv::gpu::setDevice( GPUCall ) ;
	}
	
	// page-locked CPU memory
	for( size_t poolIdx = 0 ; poolIdx < 64 ; ++poolIdx ) {
		auto& pool = this->poolsCudaMem[ poolIdx ] ;
		boost::lock_guard< boost::mutex > lock( pool.first ) ;
		pool.second.clear() ;
	}
}
MemoryPool::~MemoryPool() {
	this->clear() ;
}

// global instance
MemoryPool memoryPool ;

} } // namespaces "hawaii::GPU"

// undo local defines
#undef CUDAMEM_PAGE_LOCKED
#undef CUDAMEM_MAPPED
#undef CUDAMEM_WRITECOMBINED
