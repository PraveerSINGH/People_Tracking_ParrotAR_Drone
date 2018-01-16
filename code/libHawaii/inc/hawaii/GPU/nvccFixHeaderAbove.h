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


// fix NVCC's compatibility with certain headers e.g. from Boost or OpenCV
// =======================================================================
// user note: include this right above an incompatible header

// don't affect Eclipse's indexer
#ifndef __CDT_PARSER__
	#ifdef __CUDACC__
		
		// If optimization is enabled, "{x,e}mmintrin.h" use some built-ins which NVCC lacks.
		#ifdef __OPTIMIZE__
			#define RESTORE_OPTIMIZE __OPTIMIZE__
			#undef __OPTIMIZE__
		#endif

		// prevent "boost/config/compiler/gcc.hpp" from assuming "__int128" to be available
		#ifdef __SIZEOF_INT128__
			#define RESTORE_SIZEOF_INT128 __SIZEOF_INT128__
			#undef __SIZEOF_INT128__
		#endif
		
	#endif
#endif
