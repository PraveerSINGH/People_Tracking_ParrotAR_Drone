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


// error handling
// ==============
// display cause, then throw an exception

#pragma once

#include "hawaii/common/ostream.h"
#include "hawaii/common/optimization.h"
#include "hawaii/GPU/CUDA.h"
#include <stdexcept>

// automatically print function/method name and signature, class name and namespace(s)
// developer note: This would not have worked with a function instead of a macro.
#define HAWAII_ERROR( description ) {                               \
	const std::string message = std::string( "error in "         )   \
	                          + std::string( __PRETTY_FUNCTION__ )   \
	                          + std::string( ": "                )   \
	                          + std::string( description         ) ; \
	hawaii::cerr << message << hawaii::endl ;                        \
	throw std::runtime_error( message ) ;                            \
/*	std::abort() ; */                                                \
}

// include a condition for the error to occur, which is considered unlikely for branch prediction
#define HAWAII_ERROR_CONDITIONAL( condition, description ) { \
	if( HAWAII_UNLIKELY( condition ) ) {                      \
		HAWAII_ERROR( description ) ;                          \
	}                                                         \
}

// like "HAWAII_ERROR_CONDITIONAL()", but with description auto-generated from return value of CUDA call 
#ifndef HAWAII_NO_CUDA
	#define HAWAII_ERROR_CHECK_CUDA( cudaCall ) {                                       \
		const cudaError_t error = cudaCall ;                                             \
		if( HAWAII_UNLIKELY( error != cudaSuccess ) ) {                                  \
			const std::string message = std::string( "CUDA runtime API error in "     )   \
			                          + std::string( __PRETTY_FUNCTION__              )   \
			                          + std::string( " (or any previous CUDA call): " )   \
			                          + std::string( _cudaGetErrorEnum( error )       ) ; \
			hawaii::cerr << message << hawaii::endl ;                                     \
			throw std::runtime_error( message ) ;                                         \
		}                                                                                \
	}
	#define HAWAII_ERROR_CHECK_CUDA_DRIVER( cudaCall ) {                                \
		const CUresult error = cudaCall ;                                                \
		if( HAWAII_UNLIKELY( error != CUDA_SUCCESS ) ) {                                 \
			const std::string message = std::string( "CUDA driver API error in "      )   \
			                          + std::string( __PRETTY_FUNCTION__              )   \
			                          + std::string( " (or any previous CUDA call): " )   \
			                          + std::string( _cudaGetErrorEnum( error )       ) ; \
			hawaii::cerr << message << hawaii::endl ;                                     \
			throw std::runtime_error( message ) ;                                         \
		}                                                                                \
	}
	#define HAWAII_ERROR_CHECK_CUBLAS( cudaCall ) {                                     \
		const cublasStatus_t error = cudaCall ;                                          \
		if( HAWAII_UNLIKELY( error != CUBLAS_STATUS_SUCCESS ) ) {                        \
			const std::string message = std::string( "cuBLAS error in "               )   \
			                          + std::string( __PRETTY_FUNCTION__              )   \
			                          + std::string( " (or any previous CUDA call): " )   \
			                          + std::string( _cudaGetErrorEnum( error )       ) ; \
			hawaii::cerr << message << hawaii::endl ;                                     \
			throw std::runtime_error( message ) ;                                         \
		}                                                                                \
	}
	#define HAWAII_ERROR_CHECK_CUFFT( cudaCall ) {                                      \
		const cufftResult error = cudaCall ;                                             \
		if( HAWAII_UNLIKELY( error != CUFFT_SUCCESS ) ) {                                \
			const std::string message = std::string( "cuFFT error in "                )   \
			                          + std::string( __PRETTY_FUNCTION__              )   \
			                          + std::string( " (or any previous CUDA call): " )   \
			                          + std::string( _cudaGetErrorEnum( error )       ) ; \
			hawaii::cerr << message << hawaii::endl ;                                     \
			throw std::runtime_error( message ) ;                                         \
		}                                                                                \
	}
	#define HAWAII_ERROR_CHECK_CUSPARSE( cudaCall ) {                                   \
		const cusparseStatus_t error = cudaCall ;                                        \
		if( HAWAII_UNLIKELY( error != CUSPARSE_STATUS_SUCCESS ) ) {                      \
			const std::string message = std::string( "cuSPARSE error in "             )   \
			                          + std::string( __PRETTY_FUNCTION__              )   \
			                          + std::string( " (or any previous CUDA call): " )   \
			                          + std::string( _cudaGetErrorEnum( error )       ) ; \
			hawaii::cerr << message << hawaii::endl ;                                     \
			throw std::runtime_error( message ) ;                                         \
		}                                                                                \
	}
	#define HAWAII_ERROR_CHECK_CURAND( cudaCall ) {                                     \
		const curandStatus_t error = cudaCall ;                                          \
		if( HAWAII_UNLIKELY( error != CURAND_STATUS_SUCCESS ) ) {                        \
			const std::string message = std::string( "cuRAND error in "               )   \
			                          + std::string( __PRETTY_FUNCTION__              )   \
			                          + std::string( " (or any previous CUDA call): " )   \
			                          + std::string( _cudaGetErrorEnum( error )       ) ; \
			hawaii::cerr << message << hawaii::endl ;                                     \
			throw std::runtime_error( message ) ;                                         \
		}                                                                                \
	}
	#define HAWAII_ERROR_CHECK_NPP( cudaCall ) {                                        \
		const NppStatus error = cudaCall ;                                               \
		if( HAWAII_UNLIKELY( error != NPP_SUCCESS ) ) {                                  \
			const std::string message = std::string( "NPP error in "                  )   \
			                          + std::string( __PRETTY_FUNCTION__              )   \
			                          + std::string( " (or any previous CUDA call): " )   \
			                          + std::string( _cudaGetErrorEnum( error )       ) ; \
			hawaii::cerr << message << hawaii::endl ;                                     \
			throw std::runtime_error( message ) ;                                         \
		}                                                                                \
	}
#endif

// cause a run-time error due to missing compile-time GPU support
#ifndef HAWAII_NO_CUDA
	#define HAWAII_ERROR_NO_CUDA
#else
	#define HAWAII_ERROR_NO_CUDA HAWAII_ERROR( "compiled without GPU support" )
#endif
