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


// information on this system's CPUs and GPUs
// ==========================================

#include "hawaii/common/hardware.h"
#include "hawaii/common/minMax.h"
#include "hawaii/common/error.h"
#include "hawaii/GPU/CUDA.h"
#include <cstdlib>
#include <cstdio>

// helper(s) only used here
namespace {
	
	// return the output of a console command as an integer
	int consoleToInt( const char* command ) {
		char buf[ 64 ] ;
		FILE* pipe = popen( command, "r" ) ;
		if( fgets( buf, 64, pipe ) == 0 ) { HAWAII_ERROR( "failed to read from pipe" ) ; }
		pclose( pipe ) ;
		return atoi( buf ) ;
	}
	
	// use above function to get CPU information
	// developer note: The number of cores per socket cannot be determined by a single command because of 2 cases:
	//                 a) For multiple silicon dice within one case, evaluating by "core id" only counts one die.
	//                 b) For "modules" with multiple processing units, evaluating by "cpu cores" only counts one unit.
	int CPUSocketsImpl()          { return           consoleToInt( "cat /proc/cpuinfo | grep \"physical id\" | sort -u | wc -l"            )   ; }
	int CPUCoresPerSocketImpl()   { return std::max( consoleToInt( "cat /proc/cpuinfo | grep \"core id\"     | sort -u | wc -l"            ),
	                                                 consoleToInt( "cat /proc/cpuinfo | grep \"cpu cores\"   | sort -u | awk '{print $4}'" ) ) ; }
	int CPUThreadsPerSocketImpl() { return           consoleToInt( "cat /proc/cpuinfo | grep \"siblings\"    | sort -u | awk '{print $3}'" )   ; }
	
	// get number of GPUs present in the system
	// user note: 0 if compile-time GPU support is disabled, limited by HAWAII_GPUSMAX below
	int GPUsImpl() {
		#ifndef HAWAII_NO_CUDA
			int GPUs ; if( cudaSuccess != cudaGetDeviceCount( &GPUs ) ) { return 0 ; }
			if( GPUs > HAWAII_GPUS_MAX ) { GPUs = HAWAII_GPUS_MAX ; }
			return GPUs ;
		#else
			return 0 ;
		#endif
	}
	
	// check if GPU has own instead of sharing CPU memory
	bool GPUMemoryOwnImpl( int GPU ) {
		#ifndef HAWAII_NO_CUDA
			if( HAWAII_UNLIKELY( (uint)GPU >= (uint)GPUsImpl() ) ) { return false ; }
			int integrated       ; if( cudaSuccess != cudaDeviceGetAttribute( &integrated,       cudaDevAttrIntegrated,       GPU ) ) { return false ; }
			int canMapHostMemory ; if( cudaSuccess != cudaDeviceGetAttribute( &canMapHostMemory, cudaDevAttrCanMapHostMemory, GPU ) ) { return false ; }
			return !( integrated && canMapHostMemory ) ;
		#else
			(void)GPU ;
			return false ;
		#endif
	}
	
	// get the minimum required alignment for textures or surfaces
	size_t GPUAlignmentImpl( int GPU ) {
		#ifndef HAWAII_NO_CUDA
			if( HAWAII_UNLIKELY( (uint)GPU >= (uint)GPUsImpl() ) ) { return 1 ; }
			cudaDeviceProp prop ; if( cudaSuccess != cudaGetDeviceProperties( &prop, GPU ) ) { return 1 ; }
			return hawaii::max( prop.textureAlignment, prop.texturePitchAlignment, prop.surfaceAlignment ) ;
		#else
			(void)GPU ;
			return 1 ;
		#endif
	}
	
	// check GPU's support for unified addressing
	bool GPUUnifiedAddressingImpl( int GPU ) {
		#ifndef HAWAII_NO_CUDA
			if( HAWAII_UNLIKELY( (uint)GPU >= (uint)GPUsImpl() ) ) { return false ; }
			int unifiedAddressing ; if( cudaSuccess != cudaDeviceGetAttribute( &unifiedAddressing, cudaDevAttrUnifiedAddressing, GPU ) ) { return false ; }
			return ( unifiedAddressing > 0 ) ;
		#else
			(void)GPU ;
			return false ;
		#endif
	}
	
	// cache results from the functions above
	// developer note: Don't rely on initialization order!
	const bool   GPUMemoryOwnArray[         HAWAII_GPUS_MAX ] = { GPUMemoryOwnImpl(          0 ),
	                                                              GPUMemoryOwnImpl(          1 ),
	                                                              GPUMemoryOwnImpl(          2 ),
	                                                              GPUMemoryOwnImpl(          3 ),
	                                                              GPUMemoryOwnImpl(          4 ),
	                                                              GPUMemoryOwnImpl(          5 ),
	                                                              GPUMemoryOwnImpl(          6 ),
	                                                              GPUMemoryOwnImpl(          7 ),
	                                                              GPUMemoryOwnImpl(          8 ),
	                                                              GPUMemoryOwnImpl(          9 ),
	                                                              GPUMemoryOwnImpl(         10 ),
	                                                              GPUMemoryOwnImpl(         11 ),
	                                                              GPUMemoryOwnImpl(         12 ),
	                                                              GPUMemoryOwnImpl(         13 ),
	                                                              GPUMemoryOwnImpl(         14 ),
	                                                              GPUMemoryOwnImpl(         15 ),
	                                                              GPUMemoryOwnImpl(         16 ),
	                                                              GPUMemoryOwnImpl(         17 ),
	                                                              GPUMemoryOwnImpl(         18 ),
	                                                              GPUMemoryOwnImpl(         19 ),
	                                                              GPUMemoryOwnImpl(         20 ),
	                                                              GPUMemoryOwnImpl(         21 ),
	                                                              GPUMemoryOwnImpl(         22 ),
	                                                              GPUMemoryOwnImpl(         23 ),
	                                                              GPUMemoryOwnImpl(         24 ),
	                                                              GPUMemoryOwnImpl(         25 ),
	                                                              GPUMemoryOwnImpl(         26 ),
	                                                              GPUMemoryOwnImpl(         27 ),
	                                                              GPUMemoryOwnImpl(         28 ),
	                                                              GPUMemoryOwnImpl(         29 ),
	                                                              GPUMemoryOwnImpl(         30 ),
	                                                              GPUMemoryOwnImpl(         31 ),
	                                                              GPUMemoryOwnImpl(         32 ),
	                                                              GPUMemoryOwnImpl(         33 ),
	                                                              GPUMemoryOwnImpl(         34 ),
	                                                              GPUMemoryOwnImpl(         35 ),
	                                                              GPUMemoryOwnImpl(         36 ),
	                                                              GPUMemoryOwnImpl(         37 ),
	                                                              GPUMemoryOwnImpl(         38 ),
	                                                              GPUMemoryOwnImpl(         39 ),
	                                                              GPUMemoryOwnImpl(         40 ),
	                                                              GPUMemoryOwnImpl(         41 ),
	                                                              GPUMemoryOwnImpl(         42 ),
	                                                              GPUMemoryOwnImpl(         43 ),
	                                                              GPUMemoryOwnImpl(         44 ),
	                                                              GPUMemoryOwnImpl(         45 ),
	                                                              GPUMemoryOwnImpl(         46 ),
	                                                              GPUMemoryOwnImpl(         47 ),
	                                                              GPUMemoryOwnImpl(         48 ),
	                                                              GPUMemoryOwnImpl(         49 ),
	                                                              GPUMemoryOwnImpl(         50 ),
	                                                              GPUMemoryOwnImpl(         51 ),
	                                                              GPUMemoryOwnImpl(         52 ),
	                                                              GPUMemoryOwnImpl(         53 ),
	                                                              GPUMemoryOwnImpl(         54 ),
	                                                              GPUMemoryOwnImpl(         55 ),
	                                                              GPUMemoryOwnImpl(         56 ),
	                                                              GPUMemoryOwnImpl(         57 ),
	                                                              GPUMemoryOwnImpl(         58 ),
	                                                              GPUMemoryOwnImpl(         59 ),
	                                                              GPUMemoryOwnImpl(         60 ),
	                                                              GPUMemoryOwnImpl(         61 ),
	                                                              GPUMemoryOwnImpl(         62 ),
	                                                              GPUMemoryOwnImpl(         63 ) } ;
	const size_t GPUAlignmentArray[         HAWAII_GPUS_MAX ] = { GPUAlignmentImpl(          0 ),
	                                                              GPUAlignmentImpl(          1 ),
	                                                              GPUAlignmentImpl(          2 ),
	                                                              GPUAlignmentImpl(          3 ),
	                                                              GPUAlignmentImpl(          4 ),
	                                                              GPUAlignmentImpl(          5 ),
	                                                              GPUAlignmentImpl(          6 ),
	                                                              GPUAlignmentImpl(          7 ),
	                                                              GPUAlignmentImpl(          8 ),
	                                                              GPUAlignmentImpl(          9 ),
	                                                              GPUAlignmentImpl(         10 ),
	                                                              GPUAlignmentImpl(         11 ),
	                                                              GPUAlignmentImpl(         12 ),
	                                                              GPUAlignmentImpl(         13 ),
	                                                              GPUAlignmentImpl(         14 ),
	                                                              GPUAlignmentImpl(         15 ),
	                                                              GPUAlignmentImpl(         16 ),
	                                                              GPUAlignmentImpl(         17 ),
	                                                              GPUAlignmentImpl(         18 ),
	                                                              GPUAlignmentImpl(         19 ),
	                                                              GPUAlignmentImpl(         20 ),
	                                                              GPUAlignmentImpl(         21 ),
	                                                              GPUAlignmentImpl(         22 ),
	                                                              GPUAlignmentImpl(         23 ),
	                                                              GPUAlignmentImpl(         24 ),
	                                                              GPUAlignmentImpl(         25 ),
	                                                              GPUAlignmentImpl(         26 ),
	                                                              GPUAlignmentImpl(         27 ),
	                                                              GPUAlignmentImpl(         28 ),
	                                                              GPUAlignmentImpl(         29 ),
	                                                              GPUAlignmentImpl(         30 ),
	                                                              GPUAlignmentImpl(         31 ),
	                                                              GPUAlignmentImpl(         32 ),
	                                                              GPUAlignmentImpl(         33 ),
	                                                              GPUAlignmentImpl(         34 ),
	                                                              GPUAlignmentImpl(         35 ),
	                                                              GPUAlignmentImpl(         36 ),
	                                                              GPUAlignmentImpl(         37 ),
	                                                              GPUAlignmentImpl(         38 ),
	                                                              GPUAlignmentImpl(         39 ),
	                                                              GPUAlignmentImpl(         40 ),
	                                                              GPUAlignmentImpl(         41 ),
	                                                              GPUAlignmentImpl(         42 ),
	                                                              GPUAlignmentImpl(         43 ),
	                                                              GPUAlignmentImpl(         44 ),
	                                                              GPUAlignmentImpl(         45 ),
	                                                              GPUAlignmentImpl(         46 ),
	                                                              GPUAlignmentImpl(         47 ),
	                                                              GPUAlignmentImpl(         48 ),
	                                                              GPUAlignmentImpl(         49 ),
	                                                              GPUAlignmentImpl(         50 ),
	                                                              GPUAlignmentImpl(         51 ),
	                                                              GPUAlignmentImpl(         52 ),
	                                                              GPUAlignmentImpl(         53 ),
	                                                              GPUAlignmentImpl(         54 ),
	                                                              GPUAlignmentImpl(         55 ),
	                                                              GPUAlignmentImpl(         56 ),
	                                                              GPUAlignmentImpl(         57 ),
	                                                              GPUAlignmentImpl(         58 ),
	                                                              GPUAlignmentImpl(         59 ),
	                                                              GPUAlignmentImpl(         60 ),
	                                                              GPUAlignmentImpl(         61 ),
	                                                              GPUAlignmentImpl(         62 ),
	                                                              GPUAlignmentImpl(         63 ) } ;
	const size_t GPUAlignmentMaxInstance = hawaii::max(           GPUAlignmentImpl(          0 ),
	                                                              GPUAlignmentImpl(          1 ),
	                                                              GPUAlignmentImpl(          2 ),
	                                                              GPUAlignmentImpl(          3 ),
	                                                              GPUAlignmentImpl(          4 ),
	                                                              GPUAlignmentImpl(          5 ),
	                                                              GPUAlignmentImpl(          6 ),
	                                                              GPUAlignmentImpl(          7 ),
	                                                              GPUAlignmentImpl(          8 ),
	                                                              GPUAlignmentImpl(          9 ),
	                                                              GPUAlignmentImpl(         10 ),
	                                                              GPUAlignmentImpl(         11 ),
	                                                              GPUAlignmentImpl(         12 ),
	                                                              GPUAlignmentImpl(         13 ),
	                                                              GPUAlignmentImpl(         14 ),
	                                                              GPUAlignmentImpl(         15 ),
	                                                              GPUAlignmentImpl(         16 ),
	                                                              GPUAlignmentImpl(         17 ),
	                                                              GPUAlignmentImpl(         18 ),
	                                                              GPUAlignmentImpl(         19 ),
	                                                              GPUAlignmentImpl(         20 ),
	                                                              GPUAlignmentImpl(         21 ),
	                                                              GPUAlignmentImpl(         22 ),
	                                                              GPUAlignmentImpl(         23 ),
	                                                              GPUAlignmentImpl(         24 ),
	                                                              GPUAlignmentImpl(         25 ),
	                                                              GPUAlignmentImpl(         26 ),
	                                                              GPUAlignmentImpl(         27 ),
	                                                              GPUAlignmentImpl(         28 ),
	                                                              GPUAlignmentImpl(         29 ),
	                                                              GPUAlignmentImpl(         30 ),
	                                                              GPUAlignmentImpl(         31 ),
	                                                              GPUAlignmentImpl(         32 ),
	                                                              GPUAlignmentImpl(         33 ),
	                                                              GPUAlignmentImpl(         34 ),
	                                                              GPUAlignmentImpl(         35 ),
	                                                              GPUAlignmentImpl(         36 ),
	                                                              GPUAlignmentImpl(         37 ),
	                                                              GPUAlignmentImpl(         38 ),
	                                                              GPUAlignmentImpl(         39 ),
	                                                              GPUAlignmentImpl(         40 ),
	                                                              GPUAlignmentImpl(         41 ),
	                                                              GPUAlignmentImpl(         42 ),
	                                                              GPUAlignmentImpl(         43 ),
	                                                              GPUAlignmentImpl(         44 ),
	                                                              GPUAlignmentImpl(         45 ),
	                                                              GPUAlignmentImpl(         46 ),
	                                                              GPUAlignmentImpl(         47 ),
	                                                              GPUAlignmentImpl(         48 ),
	                                                              GPUAlignmentImpl(         49 ),
	                                                              GPUAlignmentImpl(         50 ),
	                                                              GPUAlignmentImpl(         51 ),
	                                                              GPUAlignmentImpl(         52 ),
	                                                              GPUAlignmentImpl(         53 ),
	                                                              GPUAlignmentImpl(         54 ),
	                                                              GPUAlignmentImpl(         55 ),
	                                                              GPUAlignmentImpl(         56 ),
	                                                              GPUAlignmentImpl(         57 ),
	                                                              GPUAlignmentImpl(         58 ),
	                                                              GPUAlignmentImpl(         59 ),
	                                                              GPUAlignmentImpl(         60 ),
	                                                              GPUAlignmentImpl(         61 ),
	                                                              GPUAlignmentImpl(         62 ),
	                                                              GPUAlignmentImpl(         63 ) ) ;
	const bool   GPUUnifiedAddressingArray[ HAWAII_GPUS_MAX ] = { GPUUnifiedAddressingImpl(  0 ),
	                                                              GPUUnifiedAddressingImpl(  1 ),
	                                                              GPUUnifiedAddressingImpl(  2 ),
	                                                              GPUUnifiedAddressingImpl(  3 ),
	                                                              GPUUnifiedAddressingImpl(  4 ),
	                                                              GPUUnifiedAddressingImpl(  5 ),
	                                                              GPUUnifiedAddressingImpl(  6 ),
	                                                              GPUUnifiedAddressingImpl(  7 ),
	                                                              GPUUnifiedAddressingImpl(  8 ),
	                                                              GPUUnifiedAddressingImpl(  9 ),
	                                                              GPUUnifiedAddressingImpl( 10 ),
	                                                              GPUUnifiedAddressingImpl( 11 ),
	                                                              GPUUnifiedAddressingImpl( 12 ),
	                                                              GPUUnifiedAddressingImpl( 13 ),
	                                                              GPUUnifiedAddressingImpl( 14 ),
	                                                              GPUUnifiedAddressingImpl( 15 ),
	                                                              GPUUnifiedAddressingImpl( 16 ),
	                                                              GPUUnifiedAddressingImpl( 17 ),
	                                                              GPUUnifiedAddressingImpl( 18 ),
	                                                              GPUUnifiedAddressingImpl( 19 ),
	                                                              GPUUnifiedAddressingImpl( 20 ),
	                                                              GPUUnifiedAddressingImpl( 21 ),
	                                                              GPUUnifiedAddressingImpl( 22 ),
	                                                              GPUUnifiedAddressingImpl( 23 ),
	                                                              GPUUnifiedAddressingImpl( 24 ),
	                                                              GPUUnifiedAddressingImpl( 25 ),
	                                                              GPUUnifiedAddressingImpl( 26 ),
	                                                              GPUUnifiedAddressingImpl( 27 ),
	                                                              GPUUnifiedAddressingImpl( 28 ),
	                                                              GPUUnifiedAddressingImpl( 29 ),
	                                                              GPUUnifiedAddressingImpl( 30 ),
	                                                              GPUUnifiedAddressingImpl( 31 ),
	                                                              GPUUnifiedAddressingImpl( 32 ),
	                                                              GPUUnifiedAddressingImpl( 33 ),
	                                                              GPUUnifiedAddressingImpl( 34 ),
	                                                              GPUUnifiedAddressingImpl( 35 ),
	                                                              GPUUnifiedAddressingImpl( 36 ),
	                                                              GPUUnifiedAddressingImpl( 37 ),
	                                                              GPUUnifiedAddressingImpl( 38 ),
	                                                              GPUUnifiedAddressingImpl( 39 ),
	                                                              GPUUnifiedAddressingImpl( 40 ),
	                                                              GPUUnifiedAddressingImpl( 41 ),
	                                                              GPUUnifiedAddressingImpl( 42 ),
	                                                              GPUUnifiedAddressingImpl( 43 ),
	                                                              GPUUnifiedAddressingImpl( 44 ),
	                                                              GPUUnifiedAddressingImpl( 45 ),
	                                                              GPUUnifiedAddressingImpl( 46 ),
	                                                              GPUUnifiedAddressingImpl( 47 ),
	                                                              GPUUnifiedAddressingImpl( 48 ),
	                                                              GPUUnifiedAddressingImpl( 49 ),
	                                                              GPUUnifiedAddressingImpl( 50 ),
	                                                              GPUUnifiedAddressingImpl( 51 ),
	                                                              GPUUnifiedAddressingImpl( 52 ),
	                                                              GPUUnifiedAddressingImpl( 53 ),
	                                                              GPUUnifiedAddressingImpl( 54 ),
	                                                              GPUUnifiedAddressingImpl( 55 ),
	                                                              GPUUnifiedAddressingImpl( 56 ),
	                                                              GPUUnifiedAddressingImpl( 57 ),
	                                                              GPUUnifiedAddressingImpl( 58 ),
	                                                              GPUUnifiedAddressingImpl( 59 ),
	                                                              GPUUnifiedAddressingImpl( 60 ),
	                                                              GPUUnifiedAddressingImpl( 61 ),
	                                                              GPUUnifiedAddressingImpl( 62 ),
	                                                              GPUUnifiedAddressingImpl( 63 ) } ;
	      double GPUPowerArray[             HAWAII_GPUS_MAX ] = { NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN,
	                                                              NAN } ; // ...Batman!!!
} // anonymous namespace

namespace hawaii {

// number of physical CPUs, real cores and register sets
// developer note: Don't rely on initialization order!
const int    CPUSockets          = CPUSocketsImpl()                             ;
const int    CPUCores            = CPUSocketsImpl() * CPUCoresPerSocketImpl()   ;
const int    CPUCoresPerSocket   =                    CPUCoresPerSocketImpl()   ;
const int    CPUThreads          = CPUSocketsImpl() * CPUThreadsPerSocketImpl() ;
const int    CPUThreadsPerSocket =                    CPUThreadsPerSocketImpl() ;
      double CPUPower            = NAN                                          ;

// number of GPUs present in the system
extern const int GPUs = GPUsImpl() ;

// check maximum number of GPUs supported by "libHawaii"
static_assert( HAWAII_GPUS_MAX == 64, "Changing \"HAWAII_GPU_GPUSMAX\" requires further modifications." ) ;

// copy data to the GPU (if it has own memory) or just map it into the GPU's address space (if it shared CPU memory)?
// user note: A negative index means the currently active GPU.
bool GPUMemoryOwn( int GPU ) {
	HAWAII_ERROR_NO_CUDA ;
	#ifndef HAWAII_NO_CUDA
		if( GPU < 0 ) { HAWAII_ERROR_CHECK_CUDA(  cudaGetDevice( &GPU )                        ) ; }
		else          { HAWAII_ERROR_CONDITIONAL( (uint)GPU >= (uint)GPUs, "invalid GPU index" ) ; }
	#endif
	return GPUMemoryOwnArray[ GPU ] ;
}

// get the minimum required alignment for textures or surfaces for a specific GPU or the maximum among all GPUs
// user note: A negative index means the currently active GPU.
size_t GPUAlignment( int GPU ) {
	HAWAII_ERROR_NO_CUDA ;
	#ifndef HAWAII_NO_CUDA
		if( GPU < 0 ) { HAWAII_ERROR_CHECK_CUDA(  cudaGetDevice( &GPU )                        ) ; }
		else          { HAWAII_ERROR_CONDITIONAL( (uint)GPU >= (uint)GPUs, "invalid GPU index" ) ; }
	#endif
	return GPUAlignmentArray[ GPU ] ;
}
size_t GPUAlignmentMax() {
	HAWAII_ERROR_NO_CUDA ;
	return GPUAlignmentMaxInstance ;
}

// check GPU's support for unified addressing
// user note: A negative index means the currently active GPU.
bool GPUUnifiedAddressing( int GPU ) {
	HAWAII_ERROR_NO_CUDA ;
	#ifndef HAWAII_NO_CUDA
		if( GPU < 0 ) { HAWAII_ERROR_CHECK_CUDA(  cudaGetDevice( &GPU )                        ) ; }
		else          { HAWAII_ERROR_CONDITIONAL( (uint)GPU >= (uint)GPUs, "invalid GPU index" ) ; }
	#endif
	return GPUUnifiedAddressingArray[ GPU ] ;
}

// thermal design power of each GPU
// user note #1: Currently you need to set this yourself
// user note #2: A negative index means the currently active GPU.
double& GPUPower( int GPU ) {
	HAWAII_ERROR_NO_CUDA ;
	#ifndef HAWAII_NO_CUDA
		if( GPU < 0 ) { HAWAII_ERROR_CHECK_CUDA(  cudaGetDevice( &GPU )                        ) ; }
		else          { HAWAII_ERROR_CONDITIONAL( (uint)GPU >= (uint)GPUs, "invalid GPU index" ) ; }
	#endif
	return GPUPowerArray[ GPU ] ;
}

} // namespace "hawaii"
