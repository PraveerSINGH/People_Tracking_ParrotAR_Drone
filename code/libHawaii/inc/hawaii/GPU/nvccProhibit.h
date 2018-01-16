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


// prevent the including header from being compiled with NVCC
// ==========================================================
// This is supposed to replace less readable compiler errors that would have followed otherwise.

// don't crash Eclipse's indexer
#ifndef __CDT_PARSER__
	#ifdef __CUDACC__
		#error "This header is incompatible with NVCC, so it must not be included by .cu files."
	#endif
#else
	#undef __CUDACC__
#endif
