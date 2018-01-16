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

#pragma once

#include "hawaii/common/hardware.h"
#include "hawaii/common/sync.h"
#include "hawaii/GPU/nvccFixHeaderAbove.h"
#include <opencv2/gpu/gpu.hpp>
#include <boost/thread.hpp>
#include "hawaii/GPU/nvccFixHeaderBelow.h"

namespace hawaii {
namespace GPU {

// types of GPU memory for use with the methods below
enum MemoryType {
	memoryOwn,    // GPU's own memory (will cause unnecessary copies in case it does not have any)
	memoryMapped, // page-locked CPU memory being mapped into the GPU address space (not write-combined)
	memoryAuto    // auto-select type based on hardware and/or an existing instance
} ;

// common pool for "cv::gpu::GpuMat" (both in actual GPU and in mapped CPU memory) and "cv::gpu::CudaMem"
// user note: just use the global instance "hawaii::GPU::memoryPool"
class MemoryPool {
	
	// replacement for "cv::gpu::GpuMat::create()": additionally allows specifying memory type (see above), modifies 
	//                                              the given instance only if necessary
	public:
	void operator ()( cv::gpu::GpuMat& gpuMatArg,
	                  const int rows, const int cols, const int type,
	                  MemoryType memory = memoryAuto ) ;
	void operator ()( cv::gpu::GpuMat& gpuMatArg,
	                  const cv::Size size,            const int type,
	                  MemoryType memory = memoryAuto ) {
		this->operator ()( gpuMatArg,
		                   size.height, size.width, type,
		                   memory ) ;
	}
	
	// replacement for "cv::gpu::CudaMem::create()": makes the given instance page-locked and mappable into GPU 
	//                                               address space (but not write-combined), modifies it only if 
	//                                               necessary
	public:
	void operator ()( cv::gpu::CudaMem& cudaMemArg,
	                  const int rows, const int cols, const int type ) ;
	void operator ()( cv::gpu::CudaMem& cudaMemArg,
	                  const cv::Size size,            const int type ) {
		this->operator ()( cudaMemArg,
		                   size.height, size.width, type ) ;
	}
	
	// print number of available and lent items of each (non-empty) internal pool
	public:
	void printStatus() const ;
	
	// actual pools: Each of the 64 size bins contains allocations of "2^n" bytes and serves requests between 
	//               "2^(n-1)+1" and "2^n" bytes. Additionally, pools for "cv::gpu::GpuMat" are device-specific 
	//               while those for "cv::gpu::CudaMem" are not.
	// developer note #1: When requesting a "cv::gpu::GpuMat", own GPU memory will come from a "PoolGpuMat" but 
	//                    mapped CPU memory will come from a "PoolCudaMem".
	// developer note #2: "std::pair::first" lock the pool's mutex, "std::pair::second" use the pool.
	protected:
	typedef hawaii::common::SyncPair< boost::mutex, std::vector< cv::gpu::GpuMat  > > PoolGpuMat  ;
	typedef hawaii::common::SyncPair< boost::mutex, std::vector< cv::gpu::CudaMem > > PoolCudaMem ;
	PoolGpuMat  poolsGpuMat[  HAWAII_GPUS_MAX ][ 64 ] ;
	PoolCudaMem poolsCudaMem[                    64 ] ;
	
	// d'tor
	// developer note: Default is insufficient because pooled instances must be destroyed while their corresponding 
	//                 GPU is active.
	public:
	void clear() ;
	~MemoryPool() ;
} ;

// global instance
// user note: At the end of your program, call "hawaii::GPU::reset()" or "hawaii::GPU::memoryPool.clear()" to avoid a 
//            runtime error in this instance's d'tor. It happens because the d'tors of "cv::gpu::{GpuMat,CudaMem}" from 
//            OpenCV as a shared library have otherwise already been unloaded once they are needed here.
extern MemoryPool memoryPool ;

} } // namespaces "hawaii::GPU"
