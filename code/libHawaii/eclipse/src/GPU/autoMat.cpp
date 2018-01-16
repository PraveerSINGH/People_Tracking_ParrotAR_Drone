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


// unified image/matrix type for CPUs and GPUs
// ===========================================

#include "hawaii/GPU/autoMat.h"
#include "hawaii/common/hardware.h"
#include "hawaii/common/error.h"

// helper(s) only used here
namespace {
	
	// compile-time GPU support only 
	#ifndef HAWAII_NO_CUDA
		
		// wrap "cv::gpu::CudaMem" as either "cv::Mat" or "cv::gpu::GpuMat" including its reference counter
		// developer note: This is necessary because "hawaii::GPU::poolCudaMem" needs a valid reference counter to decide 
		//                 whether an instance is available or in use.
		inline void wrapInclRefcount( const cv::gpu::CudaMem& pl,
														cv::Mat&          cm ) {
			CV_XADD( pl.refcount, 1 ) ;
			cm          = pl.createMatHeader() ;
			cm.refcount = pl.refcount          ;
		}
		inline void wrapInclRefcount( const cv::gpu::CudaMem& pl,
														cv::gpu::GpuMat&  gm ) {
			CV_XADD( pl.refcount, 1 ) ;
			gm          = pl.createGpuMatHeader() ;
			gm.refcount = pl.refcount             ;
		}
		
		// check if a GPU pointer equals a page-locked CPU pointer having been mapped into the GPU address space
		// user note: Activate the corresponding GPU before calling this!
		// developer note: This function exists because it works without unified addressing as well.
		inline bool isMappedMemory( const cv::gpu::CudaMem& pl,
		                            const cv::gpu::GpuMat&  gm ) {
			
			// Null-pointers can't point to the same memory.
			if( !pl.data || !gm.data ) { return false ; }
			
			// map CPU pointer into current GPU's address space and compare there
			void* gmDataComp ; HAWAII_ERROR_CHECK_CUDA( cudaHostGetDevicePointer( &gmDataComp, pl.data, 0 ) ) ;
			return ( gm.data == gmDataComp ) ;
		}
		
	#endif // #ifdef "compile-time GPU support?"
	
} // anonymous namespace

namespace hawaii {

// "AutoMat": wrapper around both "cv::Mat" and "cv::gpu::GpuMat"
// --------------------------------------------------------------

// c'tors
AutoMat::AutoMat() :
	#ifndef HAWAII_NO_CUDA
		pl( new hawaii::AutoMat::CudaMemValid() ),
	#endif
	rows( this ),
	cols( this ) {
}
AutoMat::AutoMat( const hawaii::AutoMat& thatAM ) :
	#ifndef HAWAII_NO_CUDA
		pl( new hawaii::AutoMat::CudaMemValid() ),
	#endif
	rows( this ),
	cols( this ) {
	*this = thatAM ;
}
AutoMat::AutoMat( const cv::Mat& thatCM ) :
	#ifndef HAWAII_NO_CUDA
		pl( new hawaii::AutoMat::CudaMemValid() ),
	#endif
	rows( this ),
	cols( this ) {
	this->writeCPU() = thatCM ;
}
AutoMat::AutoMat( const cv::gpu::GpuMat& thatGM ) :
	#ifndef HAWAII_NO_CUDA
		pl( new hawaii::AutoMat::CudaMemValid() ),
	#endif
	rows( this ),
	cols( this ) {
	this->writeGPU() = thatGM ;
}

// assignment
AutoMat& AutoMat::operator =( const AutoMat& that ) {
	
	// wrap all members' copy c'tors
	this->cm = that.cm ;
	
	// compile-time GPU support only
	#ifndef HAWAII_NO_CUDA
		*this->pl = *that.pl ;
		for( int GPU = 0 ; GPU < hawaii::GPUs ; ++GPU ) {
			this->gm[ GPU ] = that.gm[ GPU ] ;
		}
	#endif
	
	// return by reference
	return *this ;
}

// initialization
void AutoMat::invalidate() {
	
	// mark all wrapped instances invalid, but don't delete anything
	cm.upToDate = false ;
	
	// GPU support only
	#ifndef HAWAII_NO_CUDA
		pl->upToDate = false ;
		for( int GPU = 0 ; GPU < hawaii::GPUs ; ++GPU ) {
			gm[ GPU ].upToDate = false ;
		}
	#endif
}

// explicit read access from CPU
const cv::Mat& AutoMat::readCPU( cv::gpu::Stream& stream ) const {
	
	// most simple case
	if( cm.valid() ) { return cm ; }
	
	// compile-time GPU support only
	#ifndef HAWAII_NO_CUDA
		
		// check if any valid GPU memory is actually page-locked CPU memory
		const int GPUCall           = cv::gpu::getDevice() ;
		      int GPUValidMemoryOwn = -1 ;
		for( int GPU = 0 ; !pl->valid() && GPU < hawaii::GPUs ; ++GPU ) {
			if( gm[ GPU ].valid() ) {
				
				// activate before test, done on success
				cv::gpu::setDevice( GPU ) ;
				if( isMappedMemory( *pl, gm[ GPU ] ) ) {
					pl->upToDate = true ;
				}
				
				// remember valid but non-mapped GPU for later
				else {
					GPUValidMemoryOwn = GPU ;
				}
			}
		}
		
		// if no valid mapped, but a valid non-mapped GPU memory found...
		if( !pl->valid() && GPUValidMemoryOwn >= 0 ) {
			
			// download it to page-locked memory, block only if default stream used
			cv::gpu::setDevice( GPUValidMemoryOwn ) ;
			hawaii::GPU::memoryPool( *pl, gm[ GPUValidMemoryOwn ].size(), gm[ GPUValidMemoryOwn ].type() ) ;
			stream.enqueueDownload( gm[ GPUValidMemoryOwn ], *pl ) ;
			if( !stream ) { stream.waitForCompletion() ; }
			pl->upToDate = true ;
		}
		
		// restore caller's active GPU
		cv::gpu::setDevice( GPUCall ) ;
		
		// wrap and return page-locked memory
		if( pl->valid() ) {
			wrapInclRefcount( *pl, cm ) ;
			cm.upToDate = true ;
			return cm ;
		}
		
	#endif // #ifdef "compile-time GPU support?"
	
	// error if nothing found
	// developer note: The lines below just avoid compiler warnings.
	HAWAII_ERROR( "not initialized anywhere yet" ) ;
	(void)stream ;
	return cm ;
	
} // method "readCPU( cv::gpu::Stream& stream )"

// explicit read access from GPU
const cv::gpu::GpuMat& AutoMat::readGPU( cv::gpu::Stream&        stream,
                                         hawaii::GPU::MemoryType memory ) const {
	
	// compile-time GPU support or run-time error
	HAWAII_ERROR_NO_CUDA ;
	const int GPUCall = cv::gpu::getDevice() ;
	#ifndef HAWAII_NO_CUDA
		
		// find active GPU's preferred memory type
		if( memory == hawaii::GPU::memoryAuto ) {
			memory = hawaii::GPUMemoryOwn( GPUCall ) ? hawaii::GPU::memoryOwn : hawaii::GPU::memoryMapped ;
		}
		
		// most simple case
		if( gm[ GPUCall ].valid() ) {
			
			// check correct memory type and location
			if( hawaii::GPUUnifiedAddressing( GPUCall ) ) {
				cudaPointerAttributes attr ; HAWAII_ERROR_CHECK_CUDA( cudaPointerGetAttributes( &attr, gm[ GPUCall ].data ) ) ;
				HAWAII_ERROR_CONDITIONAL( attr.memoryType    == cudaMemoryTypeDevice
				                       && attr.device        != GPUCall,
				                          "Current GPU's instance is on the wrong GPU. Did you re-allocate it after activating a different GPU?" ) ;
				HAWAII_ERROR_CONDITIONAL( attr.devicePointer == nullptr,
				                          "Current GPU's instance is inaccessible. I don't know what exactly went wrong." ) ;
				
				// return only if memory types match
				if( ( memory          == hawaii::GPU::memoryOwn
				   && attr.memoryType == cudaMemoryTypeDevice      )
				 || ( memory          == hawaii::GPU::memoryMapped
				   && attr.memoryType == cudaMemoryTypeHost        ) ) {
					return gm[ GPUCall ] ;
				}
			}
			
			// have to guess without unified addressing
			else {
				const bool mapped = isMappedMemory( *pl, gm[ GPUCall ] ) ;
				
				// return only if memory types match
				if( ( memory == hawaii::GPU::memoryOwn    && !mapped )
				 || ( memory == hawaii::GPU::memoryMapped &&  mapped ) ) {
					return gm[ GPUCall ] ;
				}
			}
			
		} // if "most simple case"
		
		// check if any valid GPU memory is not only mapped and can be copied directly to the active GPU
		if( memory == hawaii::GPU::memoryOwn ) {
			for( int GPU = 0 ; GPU < hawaii::GPUs ; ++GPU ) {
				if( gm[ GPU ].valid() ) {
					
					// check correct memory type and location
					if( hawaii::GPUUnifiedAddressing( GPUCall )
					 && hawaii::GPUUnifiedAddressing( GPU     ) ) {
						cudaPointerAttributes attr ; HAWAII_ERROR_CHECK_CUDA( cudaPointerGetAttributes( &attr, gm[ GPU ].data ) ) ;
						HAWAII_ERROR_CONDITIONAL( attr.memoryType    == cudaMemoryTypeDevice
						                       && attr.device        != GPU,
						                          "Other GPU's instance is on the wrong GPU. Did you re-allocate it after activating a different GPU?" ) ;
						
						// if peer-accessible actual GPU memory...
						if( attr.devicePointer != nullptr
						 && attr.memoryType    == cudaMemoryTypeDevice ) {
							
							// copy it to caller's GPU, block only if default stream used
							// developer note: Sadly, there is no "cudaMemcpy2DPeerAsync()" yet...
							hawaii::GPU::memoryPool( gm[ GPUCall ], gm[ GPU ].size(), gm[ GPU ].type(), hawaii::GPU::memoryOwn ) ;
							if(  ( gm[ GPUCall ].step  == gm[ GPU ].step          )
							 && !( gm[ GPUCall ].flags &  cv::Mat::SUBMATRIX_FLAG ) ) {
								cudaMemcpyPeerAsync( gm[ GPUCall ].data, GPUCall,
								                     gm[ GPU     ].data, GPU,
								                     gm[ GPUCall ].rows * gm[ GPUCall ].step,
								                     cv::gpu::StreamAccessor::getStream( stream ) ) ; }
							else {
								for( int row = 0 ; row < gm[ GPUCall ].rows ; ++row ) {
									cudaMemcpyPeerAsync( gm[ GPUCall ].data + gm[ GPUCall ].step * row, GPUCall,
									                     gm[ GPU     ].data + gm[ GPU     ].step * row, GPU,
									                     gm[ GPUCall ].cols * gm[ GPUCall ].elemSize(),
									                     cv::gpu::StreamAccessor::getStream( stream ) ) ; } }
							if( !stream ) { stream.waitForCompletion() ; }
							gm[ GPUCall ].upToDate = true ;
							return gm[ GPUCall ] ;
						}
					}
				} 
			}
		}
		
		// check if valid pageable CPU memory is equal to page-locked one
		if( !pl->valid() && cm.valid() ) {
			pl->upToDate |= ( pl->data     == cm.data     )
			             && ( pl->step     == cm.step     )
			             && ( pl->size()   == cm.size()   )
			             && ( pl->type()   == cm.type()   )
			             && ( pl->refcount == cm.refcount ) ;
		}
			
		// check if any valid GPU memory is actually page-locked CPU memory
		int GPUValidMemoryOwn = -1 ;
		for( int GPU = 0 ; !pl->valid() && GPU < hawaii::GPUs ; ++GPU ) {
			if( gm[ GPU ].valid() ) {
				
				// activate before test, done on success
				cv::gpu::setDevice( GPU ) ;
				if( isMappedMemory( *pl, gm[ GPU ] ) ) {
					pl->upToDate = true ;
				}
				
				// remember valid but non-mapped GPU for later
				else {
					GPUValidMemoryOwn = GPU ;
				}
			}
		}
		
		// if pageable CPU instance is valid, but unequal to page-locked one...
		if( !pl->valid() && cm.valid() ) {
				
			// parallel deep copy within CPU memory
			// developer note #1: Yes, this really scales well and is even faster than one big "memcpy" for the 
			//                    special case of continuous matrices.
			// developer note #2: The number of OpenMP threads assumes each CPU socket (not core!) to have 4 
			//                    memory controllers, which is high-end as of 2012.
			hawaii::GPU::memoryPool( *pl, cm.size(), cm.type() ) ;
			const size_t rows  = pl->rows ;
			const size_t bytes = pl->cols * pl->elemSize() ;
			const uchar* __restrict const cmData = cm.data ;
					uchar* __restrict const plData = pl->data ;
			const ptrdiff_t cmStep = cm.step ;
			const ptrdiff_t plStep = pl->step ;
			#pragma omp parallel for \
				num_threads( hawaii::CPUSockets * 4 ) \
				default( none )
			for( size_t row = 0 ; row < rows ; ++row ) {
				memcpy( plData + plStep * row,
						  cmData + cmStep * row,
						  bytes ) ;
			}
			pl->upToDate = true ;
		}
		
		// if no valid mapped, but a valid non-mapped GPU memory found...
		if( !pl->valid() && GPUValidMemoryOwn >= 0 ) {
			
			// download it to page-locked memory, block only if default stream used
			cv::gpu::setDevice( GPUValidMemoryOwn ) ;
			hawaii::GPU::memoryPool( *pl, gm[ GPUValidMemoryOwn ].size(), gm[ GPUValidMemoryOwn ].type() ) ;
			stream.enqueueDownload( gm[ GPUValidMemoryOwn ], *pl ) ;
			if( !stream ) { stream.waitForCompletion() ; }
			pl->upToDate = true ;
		}
		
		// restore caller's active GPU
		cv::gpu::setDevice( GPUCall ) ;
		
		// either upload or map page-locked CPU memory to GPU, block only if default stream used
		if( pl->valid() ) {
			switch( memory ) {
				case hawaii::GPU::memoryOwn: {
					hawaii::GPU::memoryPool( gm[ GPUCall ], pl->rows, pl->cols, pl->type(), hawaii::GPU::memoryOwn ) ;
					stream.enqueueUpload( *pl, gm[ GPUCall ] ) ;
					if( !stream ) { stream.waitForCompletion() ; }
					gm[ GPUCall ].upToDate = true ;
					return gm[ GPUCall ] ;
					break ;
				}
				case hawaii::GPU::memoryMapped: {
					wrapInclRefcount( *pl, gm[ GPUCall ] ) ;
					gm[ GPUCall ].upToDate = true ;
					return gm[ GPUCall ] ;
					break ;
				}
				case hawaii::GPU::memoryAuto: {
				default:
					HAWAII_ERROR( "This line should never have been reached." ) ;
					break ;
				}
			}
		}
		
	#endif // #ifdef "compile-time GPU support?"	
	
	// error if nothing found
	// developer note: The lines below just avoid compiler warnings.
	HAWAII_ERROR( "not initialized anywhere yet" ) ;
	(void)stream ;
	(void)memory ;
	return gm[ GPUCall ] ;
	
} // method "readGPU( cv::gpu::Stream& stream, ... )"

// explicit write access to CPU
AutoMat::CpuMatValid& AutoMat::writeCPU( const int rows, const int cols, const int type ) {
	
	// mark all GPU images as invalid
	this->invalidate() ;
	cm.upToDate = true ;
	
	// ensure that returned memory can be mapped or quickly copied to any GPU, therefore always obtain, wrap and return 
	// page-locked memory
	if( rows != 0 && cols != 0 ) {
		#ifndef HAWAII_NO_CUDA
			hawaii::GPU::memoryPool( *pl, rows, cols, type ) ;
			wrapInclRefcount( *pl, cm ) ;
		#else
			cm.create( rows, cols, type ) ;
		#endif
	}
	return cm ;
	
} // method "writeCPU( const int rows, ... )"

// explicit write access to GPU
AutoMat::GpuMatValid& AutoMat::writeGPU( const int rows, const int cols, const int type,
                                         hawaii::GPU::MemoryType memory ) {
	
	// compile-time GPU support or run-time error
	HAWAII_ERROR_NO_CUDA ;
	const int GPU = cv::gpu::getDevice() ;
	#ifndef HAWAII_NO_CUDA
		
		// mark all images except its own as invalid
		this->invalidate() ;
		gm[ GPU ].upToDate = true ;
		
		// find active GPU's preferred memory type
		if( memory == hawaii::GPU::memoryAuto ) {
			memory = hawaii::GPUMemoryOwn( GPU ) ? hawaii::GPU::memoryOwn : hawaii::GPU::memoryMapped ;
		}
		
		// modification necessary?
		// developer note #1: unified addressing (requires 64-bit OS and "Fermi" or later GPU) needed to determine 
		//                    anything about wrapped GPU pointer
		// developer note #2: When modifying this, keep in mind that almost the same code also exists in the 
		//                    implementation of "hawaii::GPU::MemoryPool::operator ()( cv::gpu::GpuMat& gpuMatArg, ... )" 
		//                    within "libHawaii/src/GPU/pool.cpp".
		if( GPUUnifiedAddressing( GPU ) == true
		 && gm[ GPU ].data              != nullptr ) {
			
			// correct properties?
			if( gm[ GPU ].rows   == rows
			 && gm[ GPU ].cols   == cols
			 && gm[ GPU ].type() == type ) {
				
				// correct memory type and location
				cudaPointerAttributes attr ; HAWAII_ERROR_CHECK_CUDA( cudaPointerGetAttributes( &attr, gm[ GPU ].data ) ) ;
				if( attr.devicePointer != nullptr
				 && ( ( memory          == hawaii::GPU::memoryOwn
				     && attr.memoryType == cudaMemoryTypeDevice
				     && attr.device     == GPU )
				   || ( memory          == hawaii::GPU::memoryMapped
				     && attr.memoryType == cudaMemoryTypeHost ) ) ) {
					
					// return wrapped instance as it is 
					return gm[ GPU ] ;
				}
			}
		}
		
		// obtain, wrap and return requested memory
		switch( memory ) {
			case hawaii::GPU::memoryOwn: {
				if( rows != 0 && cols != 0 ) {
					hawaii::GPU::memoryPool( gm[ GPU ], rows, cols, type, hawaii::GPU::memoryOwn ) ;
				}
				break ;
			}
			case hawaii::GPU::memoryMapped: {
				if( rows != 0 && cols != 0 ) {
					hawaii::GPU::memoryPool( *pl, rows, cols, type ) ;
					wrapInclRefcount( *pl, gm[ GPU ] ) ;
				}
				break ;
			}
			case hawaii::GPU::memoryAuto: {
			default:
				HAWAII_ERROR( "This line should never have been reached." ) ;
				break ;
			}
		}
		
	#endif // #ifdef "compile-time GPU support?"
	
	// outside statements to avoid compiler warnings
	(void)rows   ;
	(void)cols   ;
	(void)type   ;
	(void)memory ;
	return gm[ GPU ] ;
	
} // method "writeGPU( const int rows, ... )"

// make writing function responsible for allocation: return wrapped instances as they are
AutoMat::CpuMatValid& AutoMat::writeCPU() {
	return this->writeCPU( cm.rows, cm.cols, cm.type() ) ;
}
AutoMat::GpuMatValid& AutoMat::writeGPU() {
	HAWAII_ERROR_NO_CUDA ;
	const int GPU = cv::gpu::getDevice() ;
	return this->writeGPU( gm[ GPU ].rows, gm[ GPU ].cols, gm[ GPU ].type(), hawaii::GPU::memoryAuto ) ;
}

// getters to allow drop-in replacement of wrapped types
#ifndef HAWAII_NO_CUDA
	#define HAWAII_AUTOMAT_GETTER( CALL, TYPE, FALLBACK )        \
		TYPE AutoMat::CALL const {                                \
			if( cm.valid()  ) { return cm.CALL  ; }                \
			if( pl->valid() ) { return pl->CALL ; }                \
			for( int GPU = 0 ; GPU < hawaii::GPUs ; ++GPU ) {      \
				if( gm[ GPU ].valid() ) { return gm[ GPU ].CALL ; } \
			}                                                      \
			FALLBACK ;                                             \
		}
#else
	#define HAWAII_AUTOMAT_GETTER( CALL, TYPE, FALLBACK ) \
		TYPE AutoMat::CALL const {                         \
			if( cm.valid() ) { return cm.CALL ; }           \
			FALLBACK ;                                      \
		}
#endif
HAWAII_AUTOMAT_GETTER( size(),      cv::Size,                                                return cv::Size( 0, 0 ) )
HAWAII_AUTOMAT_GETTER( empty(),     bool,                                                    return true             )
HAWAII_AUTOMAT_GETTER( elemSize(),  size_t,   HAWAII_ERROR( "not initialized anywhere yet" ) return 0                )
HAWAII_AUTOMAT_GETTER( elemSize1(), size_t,   HAWAII_ERROR( "not initialized anywhere yet" ) return 1                )
HAWAII_AUTOMAT_GETTER( type(),      int,      HAWAII_ERROR( "not initialized anywhere yet" ) return 0                )
HAWAII_AUTOMAT_GETTER( depth(),     int,      HAWAII_ERROR( "not initialized anywhere yet" ) return 0                )
HAWAII_AUTOMAT_GETTER( channels(),  int,      HAWAII_ERROR( "not initialized anywhere yet" ) return 1                )
#undef HAWAII_AUTOMAT_GETTER

// extract ROI
AutoMat AutoMat::operator ()( const cv::Range& rowRange, const cv::Range& colRange ) const {
	
	// use size and type to select ROI not only for valid, but for all "compatible" wrapped instances
	AutoMat ret ;
	const cv::Size size = this->size() ;
	const int      type = this->type() ;
	
	// CPU (potentially pageable)
	if( this->cm.size() == size
	 && this->cm.type() == type ) {
		ret.cm          = this->cm( rowRange, colRange ) ;
		ret.cm.upToDate = this->cm.upToDate ;
	}
	
	// compile-time GPU support only
	#ifndef HAWAII_NO_CUDA
		
		// CPU (page-locked)
		if( this->pl->data   != nullptr
		 && this->pl->size() == size
		 && this->pl->type() == type ) {
			*ret.pl           = *this->pl ;
			 ret.pl->upToDate =  this->pl->upToDate ;
			
			// ROI selection not natively supported by "cv::gpu::CudaMem"
			if( rowRange != cv::Range::all() && ( rowRange.start != 0 || rowRange.end != ret.pl->rows ) ) {
				ret.pl->rows   = rowRange.size() ;
				ret.pl->data  += rowRange.start * ret.pl->step ;
				ret.pl->flags |= cv::Mat::SUBMATRIX_FLAG ;
			}
			if( colRange != cv::Range::all() && ( colRange.start != 0 || colRange.end != ret.pl->cols ) ) {
				ret.pl->cols   = colRange.size() ;
				ret.pl->data  += colRange.start * ret.pl->elemSize() ;
				ret.pl->flags |= cv::Mat::SUBMATRIX_FLAG ;
			}
		}
		
		// GPU
		for( int GPU = 0 ; GPU < hawaii::GPUs ; ++GPU ) {
			if( this->gm[ GPU ].size() == size
			 && this->gm[ GPU ].type() == type ) {
				ret.gm[ GPU ]          = this->gm[ GPU ]( rowRange, colRange ) ;
				ret.gm[ GPU ].upToDate = this->gm[ GPU ].upToDate ;
			}
		}
		
	#endif // #ifdef "compile-time GPU support?"
	
	// return by value
	return ret ;
	
} // method "operator ()()"

} // namespace "hawaii"
