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
// user note: include this right below an incompatible header

// don't affect Eclipse's indexer
#ifndef __CDT_PARSER__
	#ifdef __CUDACC__
		
		// undo all changes made by "hawaii/GPU/nvccHeaderAbove.h"
		#ifdef RESTORE_OPTIMIZE
			#define __OPTIMIZE__ RESTORE_OPTIMIZE
			#undef RESTORE_OPTIMIZE
		#endif
		#ifdef RESTORE_SIZEOF_INT128
			#define __SIZEOF_INT128__ RESTORE_SIZEOF_INT128
			#undef RESTORE_SIZEOF_INT128
		#endif
		
	#endif
#endif
