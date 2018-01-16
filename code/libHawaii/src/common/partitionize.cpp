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

#include "hawaii/common/partitionize.h"
#include "hawaii/common/error.h"
#include "hawaii/GPU/autoMat.h"
#include <opencv2/gpu/gpu.hpp>
#include <cmath>
#include <numeric>

namespace hawaii {

// partition [0,"total") into almost equally sized ranges ["cv::Range::start,end")
std::vector< cv::Range > partitionize( const int total,
                                       const int partitions,
                                       const int overlap    ) {
	// check and prepare
	HAWAII_ERROR_CONDITIONAL( ( total - overlap ) < 0, "too much overlap for given range" ) ;
	const int totalNoOverlap = total - overlap ;
	const int sizeBase    = totalNoOverlap / partitions + overlap ;
	      int partsBigger = totalNoOverlap % partitions ;
	
	// partition
	std::vector< cv::Range > ret ; ret.reserve( partitions ) ;
	int start = 0, size ;
	for( int part = 0 ; part < partitions ; ++part ) {
		size = sizeBase ;
		if( partsBigger > 0 ) { ++size ; --partsBigger ; }
		ret.push_back( cv::Range( start, start + size ) ) ;
		start += size - overlap ;
	}
	return ret ;
}

// partition [0,"total") into ranges proportional to "ratios"
std::vector< cv::Range > partitionize( const int                   total,
                                       const std::vector< double > ratios,
                                       const int                   overlap ) {
	// check and prepare
	HAWAII_ERROR_CONDITIONAL( ( total - overlap ) < 0, "too much overlap for given range" ) ;
	const int    totalNoOverlap = total - overlap ;
	const double quotaInv       = totalNoOverlap / std::accumulate( ratios.cbegin(), ratios.cend(), 0.0 ) ;
	
	// determine exact partition sizes, split into integer and fractional parts
	std::vector< int    > sizes      ; sizes.reserve(      ratios.size() ) ;
	std::vector< double > remainders ; remainders.reserve( ratios.size() ) ;
	for( const auto& ratio : ratios ) {
		const double sizeExact = ratio * quotaInv + overlap ;
		const double sizeBase  = floor( sizeExact ) ;
		const double remainder = sizeExact - sizeBase ;
		sizes.push_back(      sizeBase  ) ;
		remainders.push_back( remainder ) ;
	}
	
	// sort partition indices by the size of their corresponding remainders
	// developer note: adapted from "stackoverflow.com/a/12399290"
	std::vector< size_t > indices ; indices.reserve( remainders.size() ) ;
	for( size_t index = 0 ; index < remainders.size() ; ++index ) { indices.push_back( index ) ; }
	std::sort( indices.begin(), indices.end(),
	           [ &remainders ]( size_t index1, size_t index2 ) {
		           return ( remainders[ index1 ] > remainders[ index2 ] ) ;
	           } ) ;
	
	// increase size of partitions with biggest remainders until required total is matched
	const size_t totalPlusOverlap = totalNoOverlap + overlap * ratios.size() ;
	const size_t partsBigger = totalPlusOverlap - std::accumulate( sizes.begin(), sizes.end(), 0 ) ;
	for( size_t indexIndex = 0 ; indexIndex < partsBigger ; ++indexIndex ) {
		++sizes[ indices[ indexIndex ] ] ;
	}
	
	// convert sizes to ranges
	std::vector< cv::Range > ret ; ret.reserve( sizes.size() ) ;
	int partStart = 0 ;
	for( const auto& size : sizes ) {
		ret.push_back( cv::Range( partStart, partStart + size ) ) ;
		partStart += size - overlap ;
	}
	return ret ;
}

// partition a range into multiple ones
template< typename PartsOrRatios >
std::vector< cv::Range > partitionize( const cv::Range     total,
                                       const PartsOrRatios partsOrRatios,
                                       const int           overlap       ) {
	// partition
	auto ret = partitionize( total.size(), partsOrRatios, overlap ) ;
	
	// add offset
	if( total.start != 0 ) {
		for( auto& range : ret ) {
			range.start += total.start ;
			range.end   += total.start ;
		}
	}
	return ret ;
}
template std::vector< cv::Range > partitionize( const cv::Range, const int,                   const int ) ;
template std::vector< cv::Range > partitionize( const cv::Range, const std::vector< double >, const int ) ;

// partition a "hawaii::AutoMat", "cv::Mat" or "cv::gpu::GpuMat" horizontally
template< typename Mat, typename PartsOrRatios >
std::vector< Mat > partitionizeHoriz( const Mat&          total,
                                      const PartsOrRatios partsOrRatios,
                                      const int           overlap       ) {
	// partition
	const auto rangesHor = partitionize( total.rows, partsOrRatios, overlap ) ;
	
	// apply
	std::vector< Mat > ret ; ret.reserve( rangesHor.size() ) ;
	for( const auto& rangeHor : rangesHor ) {
		ret.push_back( total.rowRange( rangeHor ) ) ;
	}
	return ret ;
}
template std::vector< cv::Mat         > partitionizeHoriz( const cv::Mat&,         const int,                   const int ) ;
template std::vector< cv::Mat         > partitionizeHoriz( const cv::Mat&,         const std::vector< double >, const int ) ;
template std::vector< cv::gpu::GpuMat > partitionizeHoriz( const cv::gpu::GpuMat&, const int,                   const int ) ;
template std::vector< cv::gpu::GpuMat > partitionizeHoriz( const cv::gpu::GpuMat&, const std::vector< double >, const int ) ;
template std::vector< hawaii::AutoMat > partitionizeHoriz( const hawaii::AutoMat&, const int,                   const int ) ;
template std::vector< hawaii::AutoMat > partitionizeHoriz( const hawaii::AutoMat&, const std::vector< double >, const int ) ;

// partition a "hawaii::AutoMat", "cv::Mat" or "cv::gpu::GpuMat" vertically
template< typename Mat, typename PartsOrRatios >
std::vector< Mat > partitionizeVerti( const Mat&          total,
                                      const PartsOrRatios partsOrRatios,
                                      const int           overlap       ) {
	// partition
	const auto rangesVer = partitionize( total.cols, partsOrRatios, overlap ) ;
	
	// apply
	std::vector< Mat > ret ; ret.reserve( rangesVer.size() ) ;
	for( const auto& rangeVer : rangesVer ) {
		ret.push_back( total.colRange( rangeVer ) ) ;
	}
	return ret ;
}
template std::vector< cv::Mat         > partitionizeVerti( const cv::Mat&,         const int,                   const int ) ;
template std::vector< cv::Mat         > partitionizeVerti( const cv::Mat&,         const std::vector< double >, const int ) ;
template std::vector< cv::gpu::GpuMat > partitionizeVerti( const cv::gpu::GpuMat&, const int,                   const int ) ;
template std::vector< cv::gpu::GpuMat > partitionizeVerti( const cv::gpu::GpuMat&, const std::vector< double >, const int ) ;
template std::vector< hawaii::AutoMat > partitionizeVerti( const hawaii::AutoMat&, const int,                   const int ) ;
template std::vector< hawaii::AutoMat > partitionizeVerti( const hawaii::AutoMat&, const std::vector< double >, const int ) ;

// partition a "hawaii::AutoMat", "cv::Mat" or "cv::gpu::GpuMat" both horizontally and vertically
template< typename Mat, typename PartsOrRatiosHoriz, typename PartsOrRatiosVerti >
std::vector< Mat > partitionizeGrid( const Mat&               total,
                                     const PartsOrRatiosHoriz partsOrRatiosHoriz,
                                     const PartsOrRatiosVerti partsOrRatiosVerti,
                                     const int                overlapHoriz,
                                     const int                overlapVerti       ) {
	// partition
	const auto rangesHor = partitionize( total.rows, partsOrRatiosHoriz, overlapHoriz ) ;
	const auto rangesVer = partitionize( total.cols, partsOrRatiosVerti, overlapVerti ) ;
	
	// apply
	std::vector< Mat > ret ; ret.reserve( rangesHor.size() * rangesVer.size() ) ;
	for( const auto& rangeHor : rangesHor ) {
	for( const auto& rangeVer : rangesVer ) {
		ret.push_back( total( rangeHor, rangeVer ) ) ;
	} }
	return ret ;
}
template std::vector< cv::Mat         > partitionizeGrid( const cv::Mat&,         const int,                   const int,                   const int, const int ) ;
template std::vector< cv::Mat         > partitionizeGrid( const cv::Mat&,         const int,                   const std::vector< double >, const int, const int ) ;
template std::vector< cv::Mat         > partitionizeGrid( const cv::Mat&,         const std::vector< double >, const int,                   const int, const int ) ;
template std::vector< cv::Mat         > partitionizeGrid( const cv::Mat&,         const std::vector< double >, const std::vector< double >, const int, const int ) ;
template std::vector< cv::gpu::GpuMat > partitionizeGrid( const cv::gpu::GpuMat&, const int,                   const int,                   const int, const int ) ;
template std::vector< cv::gpu::GpuMat > partitionizeGrid( const cv::gpu::GpuMat&, const int,                   const std::vector< double >, const int, const int ) ;
template std::vector< cv::gpu::GpuMat > partitionizeGrid( const cv::gpu::GpuMat&, const std::vector< double >, const int,                   const int, const int ) ;
template std::vector< cv::gpu::GpuMat > partitionizeGrid( const cv::gpu::GpuMat&, const std::vector< double >, const std::vector< double >, const int, const int ) ;
template std::vector< hawaii::AutoMat > partitionizeGrid( const hawaii::AutoMat&, const int,                   const int,                   const int, const int ) ;
template std::vector< hawaii::AutoMat > partitionizeGrid( const hawaii::AutoMat&, const int,                   const std::vector< double >, const int, const int ) ;
template std::vector< hawaii::AutoMat > partitionizeGrid( const hawaii::AutoMat&, const std::vector< double >, const int,                   const int, const int ) ;
template std::vector< hawaii::AutoMat > partitionizeGrid( const hawaii::AutoMat&, const std::vector< double >, const std::vector< double >, const int, const int ) ;

} // namespace "hawaii"
