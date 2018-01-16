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

#pragma once

#include "hawaii/GPU/CUDA.h"
#include "hawaii/GPU/pool.h"
#include "hawaii/GPU/nvccFixHeaderAbove.h"
#include <opencv2/gpu/gpu.hpp>
#include <boost/scoped_ptr.hpp>
#include "hawaii/GPU/nvccFixHeaderBelow.h"

namespace hawaii {

// wrapper around both "cv::Mat" and "cv::gpu::GpuMat" with the following features:
// - no explicit calls for copying between CPU and GPUs required (except when using streams): The most recently written 
//   data is automatically copied to the device to be read from.
// - support for multiple GPUs, optionally automatic decision between either actual GPU memory or mapped CPU memory
// - asynchronous, high-throughput copying by using page-locked CPU memory and peer-to-peer communication between GPUs
class AutoMat {
	
	// initialization, c'tors and assignment
	// developer note: Versions with "AutoMat" arguments must be explicitly implemented due to the references "rows" and 
	//                 "cols" below.
	public:
	AutoMat() ;
	AutoMat( const hawaii::AutoMat& thatAM ) ;
	AutoMat( const cv::Mat&         thatCM ) ;
	AutoMat( const cv::gpu::GpuMat& thatGM ) ;
	hawaii::AutoMat& operator =( const hawaii::AutoMat& thatAM ) ;
	hawaii::AutoMat& operator =( const cv::Mat&         thatCM ) { this->writeCPU() = thatCM ; return *this ; }
	hawaii::AutoMat& operator =( const cv::gpu::GpuMat& thatGM ) { this->writeGPU() = thatGM ; return *this ; }
	protected:
	void invalidate() ; // mark all wrapped instances invalid, but don't delete anything
	
	// extend wrapped image/matrix types with a flag for their content's up-to-dateness
	// developer note: Aside from the methods below, the "upToDate" flag must be managed explicitly and manually. 
	protected:
	template< typename MatBase >
	class MatValid : public MatBase {
		public:
		bool upToDate ;
		MatValid(                                      ) :                                         upToDate(  false             ) {}
		MatValid(             const MatBase&  matBase  ) : MatBase(             matBase  ),        upToDate(  true              ) {}
		MatValid(             const MatValid& matValid ) : MatBase(             matValid ),        upToDate(  matValid.upToDate ) {}
		MatValid& operator =( const MatBase&  matBase  ) { MatBase::operator =( matBase  ) ; this->upToDate = true              ; return *this ; }
		MatValid& operator =( const MatValid& matValid ) { MatBase::operator =( matValid ) ; this->upToDate = matValid.upToDate ; return *this ; }
		bool valid() const { return ( upToDate && !MatBase::empty() ) ; }
	} ;
	typedef MatValid< cv::Mat          > CpuMatValid  ;
	typedef MatValid< cv::gpu::CudaMem > CudaMemValid ;
	typedef MatValid< cv::gpu::GpuMat  > GpuMatValid  ;
	
	// members: shared CPU instances (pageable and page-locked), device-specific GPU instances
	// developer note #1: "mutable" required for "const"-correctness at public interface.
	// developer note #2: "pl" must be a pointer to leave it un-initialized in case of missing compile-time GPU support. 
	//                    Otherwise, the d'tor of "cv::gpu::CudaMem" unnecessarily throws an exception which cannot be 
	//                    caught within this class.
	protected:
	mutable                    hawaii::AutoMat::CpuMatValid    cm                    ;
	mutable boost::scoped_ptr< hawaii::AutoMat::CudaMemValid > pl                    ;
	mutable                    hawaii::AutoMat::GpuMatValid    gm[ HAWAII_GPUS_MAX ] ;
	
	// automatic conversions to common image/matrix types
	// user note #1: Conversion to a non-const reference is considered writing, all other conversions are considered 
	//               reading. Therefore, write your functions e.g. like "void myFunc( const cv::Mat in, cv::Mat& out )".
	// user note #2: The memory location of GPU types (actual GPU memory or mapped CPU memory) will automatically be 
	//               selected based on the active GPU's properties.
	// user note #3: Potentially necessary copying will use the default stream and therefore be synchronous.
	// user note #4: Your code may re-allocate instances to be written. To prevent global synchronization and improve 
	//               performance, the use of "hawaii::GPU::memoryPool" is recommended for that.
	// user note #5: "cv::_{Input,Output}Array" will contain a "cv::Mat", not an (also possible) "cv::gpu::GpuMat".
	// developer note #1: To make reading conversions precede writing ones as specified above, both non-const reading 
	//                    methods and writing methods with inherited types are required. ("cv::_OutputArray" inherits 
	//                    from "cv::_InputArray", so they are okay.)
	// developer note #2: "hawaii::GPU::streamNull" is used instead of "cv::gpu::Stream::Null()" because it does not 
	//                    crash if compile-time GPU support is disabled.
	public:
	operator const cv::Mat&()               { return                    readCPU()   ; }
	operator const cv::Mat&()         const { return                    readCPU()   ; }
	operator const cv::gpu::GpuMat&()       { return                    readGPU()   ; }
	operator const cv::gpu::GpuMat&() const { return                    readGPU()   ; }
	operator const cv::_InputArray()        { return cv::_InputArray(   readCPU() ) ; }
	operator const cv::_InputArray()  const { return cv::_InputArray(   readCPU() ) ; }
	operator       CpuMatValid&()           { return                   writeCPU()   ; }
	operator       GpuMatValid&()           { return                   writeGPU()   ; }
	operator       cv::_OutputArray()       { return cv::_OutputArray( writeCPU() ) ; }
	
	// explicit read access: optionally specify a stream for asynchronous copying and/or a memory type
	public:
	const cv::Mat&         readCPU( cv::gpu::Stream&        stream = hawaii::GPU::streamNull ) const ;
	const cv::gpu::GpuMat& readGPU( cv::gpu::Stream&        stream = hawaii::GPU::streamNull,
	                                hawaii::GPU::MemoryType memory = hawaii::GPU::memoryAuto ) const ;
	// allow switched arguments
	const cv::gpu::GpuMat& readGPU( hawaii::GPU::MemoryType memory,
	                                cv::gpu::Stream&        stream = hawaii::GPU::streamNull ) const {
		return readGPU( stream, memory ) ;
	}
	
	// explicit write access: invalidate all wrapped instances except the one being written to
	// user note #1: Explicitly specifying the required size and type has performance advantages:
	//               - Potentially required (re-)allocations of a "cv::gpu::GpuMat" will use "hawaii::GPU::memoryPool", 
	//                 while the function writing to this image may cause global synchronization by calling 
	//                 "cv::gpu::GpuMat::create()".
	//               - A returned "cv::Mat" will always wrap page-locked memory, so that its content can later be mapped 
	//                 or quickly copied to any GPU.
	// user note #2: The memory location of GPU types (actual GPU memory or mapped CPU memory) will automatically be 
	//               selected based on the active GPU's properties.
	public:
	hawaii::AutoMat::CpuMatValid& writeCPU( const int rows, const int cols, const int type           ) ;
	hawaii::AutoMat::GpuMatValid& writeGPU( const int rows, const int cols, const int type,
	                                        hawaii::GPU::MemoryType memory = hawaii::GPU::memoryAuto ) ;
	// allow "CvSize" or "cv::Size" instead of rows and columns
	hawaii::AutoMat::CpuMatValid& writeCPU( const cv::Size size,            const int type           ) {
		return writeCPU( size.height, size.width, type         ) ;
	}
	hawaii::AutoMat::GpuMatValid& writeGPU( const cv::Size size,            const int type,
	                                        hawaii::GPU::MemoryType memory = hawaii::GPU::memoryAuto ) {
		return writeGPU( size.height, size.width, type, memory ) ;
	}
	// make writing function responsible for allocation: return wrapped instances as they are (not recommended, see above)
	hawaii::AutoMat::CpuMatValid& writeCPU() ;
	hawaii::AutoMat::GpuMatValid& writeGPU() ;
	
	// getters to allow drop-in replacement of wrapped types
	// user note #1: Unlike OpenCV, querying values that do not make sense for uninitialized instances cause an error.
	// user note #2: "step", "step1()", "flags", "isContinuous()" and "isSubmatrix()" are not available because their 
	//               results may differ between CPU and GPUs. "dims" is not available either because GPU types only 
	//               support 2D data.
	// developer note: Since references cannot be re-assigned, the little nasty hack below is required to emulate read 
	//                 access to OpenCV's directly accessible "rows" and "cols" members. Directly writing them is not 
	//                 only impossible here, but also a very bad idea in general.
	protected:
	struct Rows {
		const hawaii::AutoMat* const membee ;
		Rows( const hawaii::AutoMat* const membeeArg ) : membee( membeeArg ) {}
		operator int() const { return membee->size().height ; }
	} ;
	struct Cols {
		const hawaii::AutoMat* const membee ;
		Cols( const hawaii::AutoMat* const membeeArg ) : membee( membeeArg ) {}
		operator int() const { return membee->size().width ; }
	} ;
	public:
	Rows     rows              ;
	Cols     cols              ;
	cv::Size size()      const ;
	bool     empty()     const ;
	size_t   elemSize()  const ; // causes error if uninitialized
	size_t   elemSize1() const ; // "
	int      type()      const ; // "
	int      depth()     const ; // "
	int      channels()  const ; // "
	
	// extract ROI
	// user note: This wraps not only valid wrapped images/matrices, but any memory with the same size and type as the 
	//            valid instance(s). It however does not affect any future allocations, i.e. selecting a ROI of a 
	//            CPU-only "AutoMat" and then reading from the GPU will obtain independent GPU memory with only the 
	//            ROI's size, which the original whole "AutoMat" will not know about.
	public:
	hawaii::AutoMat operator ()( const cv::Range& rowRange, const cv::Range& colRange = cv::Range::all() ) const ;
	hawaii::AutoMat operator ()( const cv::Rect& rect                 ) const { return this->operator ()( cv::Range( rect.y,   rect.y + rect.height ), cv::Range( rect.x,   rect.x + rect.width ) ) ; }
	hawaii::AutoMat row(         const int row                        ) const { return this->operator ()( cv::Range( row,      row + 1              ), cv::Range::all()                           ) ; }
	hawaii::AutoMat rowRange(    const int rowStart, const int rowEnd ) const { return this->operator ()( cv::Range( rowStart, rowEnd               ), cv::Range::all()                           ) ; }
	hawaii::AutoMat rowRange(    const cv::Range& rowRange            ) const { return this->operator ()( rowRange,                                    cv::Range::all()                           ) ; }
	hawaii::AutoMat col(         const int col                        ) const { return this->operator ()( cv::Range::all(),                            cv::Range( col,      col + 1             ) ) ; }
	hawaii::AutoMat colRange(    const int colStart, const int colEnd ) const { return this->operator ()( cv::Range::all(),                            cv::Range( colStart, colEnd              ) ) ; }
	hawaii::AutoMat colRange(    const cv::Range& colRange            ) const { return this->operator ()( cv::Range::all(),                            colRange                                   ) ; }
	
} ; // class "AutoMat"

} // namespace "hawaii"
