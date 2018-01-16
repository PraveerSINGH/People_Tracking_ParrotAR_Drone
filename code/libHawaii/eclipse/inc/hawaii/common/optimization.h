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


// various optimization helpers
// ============================

#pragma once

// hints to aid branch prediction: use like "if( HAWAII_UNLIKELY( column == 0 ) ) { ... }"
#ifdef __GNUC__
	#define HAWAII_LIKELY(   condition ) __builtin_expect( !!( condition ), 1 )
	#define HAWAII_UNLIKELY( condition ) __builtin_expect( !!( condition ), 0 )
#else
	#define HAWAII_LIKELY(   condition )                 ( !!( condition )    )
	#define HAWAII_UNLIKELY( condition )                 ( !!( condition )    )
#endif
