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


// partitioning of ranges and images
// =================================

#pragma once

#include <opencv2/core/core.hpp>
#include <vector>

namespace hawaii {

// partition [0,"total") into equally sized ranges ["cv::Range::start,end")
// user note #1: The first "total % partitions" ranges will be 1 bigger than the others.
// user note #2: I know the verb is "partition", just like the noun. The functions below are named "partitionize" to 
//               allow distinguishing them from a partition they have created.
std::vector< cv::Range > partitionize( const int total,
                                       const int partitions,
                                       const int overlap    = 0 ) ;
// partition [0,"total") into ranges proportional to "ratios"
// user note: This implements a two-step method known i.a. from elections. For details see 
//            "en.wikipedia.org/wiki/Largest_remainder_method".
std::vector< cv::Range > partitionize( const int                   total,
                                       const std::vector< double > ratios,
                                       const int                   overlap = 0 ) ;

// partition a range into multiple ones
// user note: "PartsOrRatios" can be either "int" or "std::vector< double >".
template< typename PartsOrRatios >
std::vector< cv::Range > partitionize( const cv::Range     total,
                                       const PartsOrRatios partsOrRatios,
                                       const int           overlap       = 0 ) ;

// partition a "hawaii::AutoMat", "cv::Mat" or "cv::gpu::GpuMat" horizontally, vertically and both
// user note #1: Data is only wrapped, not deeply copied.
// user note #2: "Horiz" means horizontal stripes which are vertically stacked, "Verti" is the other way round.
template< typename Mat, typename PartsOrRatios >
std::vector< Mat > partitionizeHoriz( const Mat&          total,
                                      const PartsOrRatios partsOrRatios,
                                      const int           overlap       = 0 ) ;
template< typename Mat, typename PartsOrRatios >
std::vector< Mat > partitionizeVerti( const Mat&          total,
                                      const PartsOrRatios partsOrRatios,
                                      const int           overlap       = 0 ) ;
template< typename Mat, typename PartsOrRatiosHoriz, typename PartsOrRatiosVerti >
std::vector< Mat > partitionizeGrid( const Mat&               total,
                                     const PartsOrRatiosHoriz partsOrRatiosHoriz,
                                     const PartsOrRatiosVerti partsOrRatiosVerti,
                                     const int                overlapHoriz       = 0,
                                     const int                overlapVerti       = 0 ) ;
} // namespace "hawaii"
