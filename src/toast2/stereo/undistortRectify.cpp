// Copyright (C) 2013 by - FZI Forschungszentrum Informatik am Karlsruher Institut fuer Technologie
//                       - Institut Eurécom
//                       - Télécom ParisTech
// 
// Author: Benjamin Ranft (benjamin.ranft@web.de)
// 
// This file is part of demoARDrone.
// 
// demoARDrone is free software: you can redistribute it and/or modify it under the terms of the GNU General Public 
// License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later 
// version.
// 
// demoARDrone is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with libHawaii. If not, see 
// <http://www.gnu.org/licenses/>.


// undistortion, rectification and cropping of raw camera images
// =============================================================

#include "toast2/stereo/undistortRectify.h"
#include "hawaii/common/partitionize.h"
#include "hawaii/common/error.h"
#include <opencv2/calib3d/calib3d.hpp>

namespace toast2 {
namespace stereo {

// camera matrix
Camera::Camera( const double focalLengthUArg,
                const double focalLengthVArg,
                const double principalPointUArg,
                const double principalPointVArg ) :
	cv::Matx33d( focalLengthUArg, 0.0,             principalPointUArg,
	             0.0,             focalLengthVArg, principalPointVArg,
	             0.0,             0.0,             1.0                ) {}
            Camera::Camera(     const cv::Matx33d that ) :        cv::Matx33d(             that ) {}
cv::Matx33d Camera::operator =( const cv::Matx33d that ) { return cv::Matx33d::operator =( that ) ; }

// distortion parameters
// developer note: There are only "cv::Vec{2,3,4,6}d" but no "5"...
Distortion::Distortion( const double radial1Arg,
                        const double radial2Arg,
                        const double tangential1Arg,
                        const double tangential2Arg,
                        const double radial3Arg     ) :
	cv::Vec< double, 5 >( radial1Arg, radial2Arg, tangential1Arg, tangential2Arg, radial3Arg ) {}
                     Distortion::Distortion( const cv::Vec< double, 5 > that ) :        cv::Vec< double, 5 >(             that ) {}
cv::Vec< double, 5 > Distortion::operator =( const cv::Vec< double, 5 > that ) { return cv::Vec< double, 5 >::operator =( that ) ; }

// generic remapping: perform transformation on CPU
void Remapper::CPU( const cv::Mat  src,
                          cv::Mat& dst,
                          cv::Rect roi,
                    const int      cores ) const {
	
	// prevent writing, but allow other readers
	boost::shared_lock< boost::shared_mutex > readLock( this->mutex ) ;
	
	// check state
	HAWAII_ERROR_CONDITIONAL( this->mapPix.empty()
	                       || this->mapSubPix.empty(),
	                          "transformation maps not initialized" ) ;
	HAWAII_ERROR_CONDITIONAL( this->mapPix.size()
	                       != this->mapSubPix.size(),
	                          "transformation maps inconsistent" ) ;
	HAWAII_ERROR_CONDITIONAL( roi.area() != 0
	                       && ( roi.x < 0 || ( roi.x + roi.width  ) > this->mapPix.cols
	                         || roi.y < 0 || ( roi.y + roi.height ) > this->mapPix.rows ),
	                          "Region of interest exceeds result size."                   ) ;
	// handle default ROI
	if( roi.area() == 0 ) {
		roi.x = 0 ; roi.width  = this->mapPix.cols ;
		roi.y = 0 ; roi.height = this->mapPix.rows ;
	}
	
	// allocate result
	dst.create( roi.size(), src.type() ) ;
	
	// partition LUTs and output
	const auto mapPixParts    = hawaii::partitionizeHoriz( this->mapPix(    roi ), cores ) ;
	const auto mapSubPixParts = hawaii::partitionizeHoriz( this->mapSubPix( roi ), cores ) ;
	const auto dstParts       = hawaii::partitionizeHoriz( dst,                    cores ) ;
	
	// remap each partition
	#pragma omp parallel for                \
		if( cores > 1 ) num_threads( cores ) \
		default( none )
	for( int core = 0 ; core < cores ; ++core ) {
		cv::remap( src, dstParts[ core ],
		           mapPixParts[ core ], mapSubPixParts[ core ],
		           cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0    ) ;
	}
}

// generic remapping: perform transformation on GPU
void Remapper::GPU( const cv::gpu::GpuMat  src,
                          cv::gpu::GpuMat& dst,
                          cv::Rect         roi,
                          cv::gpu::Stream& stream ) const {
	
	// check compile-time GPU support
	#ifndef HAWAII_NO_CUDA
		
		// prevent writing, but allow other readers
		boost::shared_lock< boost::shared_mutex > readLock( this->mutex ) ;
		
		// check state
		HAWAII_ERROR_CONDITIONAL( this->mapPix.empty()
		                       || this->mapSubPix.empty(),
		                          "transformation maps not initialized" ) ;
		HAWAII_ERROR_CONDITIONAL( this->mapPix.size()
		                       != this->mapSubPix.size(),
		                          "transformation maps inconsistent" ) ;
		HAWAII_ERROR_CONDITIONAL( roi.area() != 0
		                       && ( roi.x < 0 || ( roi.x + roi.width  ) > this->mapPix.cols
		                         || roi.y < 0 || ( roi.y + roi.height ) > this->mapPix.rows ),
		                          "Region of interest exceeds result size."                   ) ;
		// handle default ROI
		if( roi.area() == 0 ) {
			roi.x = 0 ; roi.width  = this->mapPix.cols ;
			roi.y = 0 ; roi.height = this->mapPix.rows ;
		}
		
		// allocate result
		hawaii::GPU::memoryPool( dst, roi.size(), src.type() ) ;
		
		// remap the image, specify stream in case copying is necessary
		cv::gpu::remap( src, dst,
		                this->mapU.readGPU( stream )( roi ),
		                this->mapV.readGPU( stream )( roi ),
		                cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0,
		                stream                                   ) ;
		
	// cause run-time error, but avoid compile-time warnings
	#else
		HAWAII_ERROR_NO_CUDA ;
		(void)src    ;
		(void)dst    ;
		(void)roi    ;
		(void)stream ;
	#endif
}

// generic remapping: update maps
void Remapper::update( const cv::Mat mapUArg,
                       const cv::Mat mapVArg ) {
	
	// check input
	HAWAII_ERROR_CONDITIONAL( mapUArg.type() != CV_32FC1
	                       || mapVArg.type() != CV_32FC1,
	                          "Type must be single-channel float." ) ;
	HAWAII_ERROR_CONDITIONAL( mapUArg.size()
	                       != mapVArg.size(),
	                          "Sizes must be identical." ) ;
	
	// convert to index-based LUTs
	cv::Mat mapPixArg, mapSubPixArg ;
	cv::convertMaps( mapUArg, mapVArg, mapPixArg, mapSubPixArg, CV_16SC2, false ) ;
	
	// get exclusive write access and replace maps
	boost::lock_guard< boost::shared_mutex > writeLock( this->mutex ) ;
	this->mapU      = mapUArg      ;
	this->mapV      = mapVArg      ;
	this->mapPix    = mapPixArg    ;
	this->mapSubPix = mapSubPixArg ;
}

// undistort: c'tor
Undistorter::Undistorter( const Camera     cameraArg,
                          const Distortion distortionArg,
                          const cv::Size   sizeSrcArg,
                          const cv::Size   sizeDstArg,
                          const bool       equalizeFocalLengthsArg ) :
	distortion( distortionArg ),
	cameraSrc( cameraArg           ),
	cameraDst( this->cameraDstReal ),
	sizeSrc( sizeSrcArg ),
	sizeDst( sizeDstArg ),
	sizeDstEqualFocalLength( this->sizeDstEqualFocalLengthReal ),
	equalizeFocalLengths( equalizeFocalLengthsArg ) {
	
	// handle default result size
	if( this->sizeDst.width  == 0 ) { this->sizeDst.width  = this->sizeSrc.width  ; }
	if( this->sizeDst.height == 0 ) { this->sizeDst.height = this->sizeSrc.height ; }
	
	// initialize if sufficient non-default information is given
	if( this->cameraSrc(0,0) > 0.0
	 && this->cameraSrc(1,1) > 0.0
	 && this->cameraSrc(0,2) > 0.0
	 && this->cameraSrc(1,2) > 0.0
	 && this->sizeSrc.width  > 0 && this->sizeDst.width  > 0
	 && this->sizeSrc.height > 0 && this->sizeDst.height > 0 ) {
		this->update() ;
	}
}

// undistort: (re-)initialize
void Undistorter::update() {
	
	// handle default result size
	if( this->sizeDst.width  == 0 ) { this->sizeDst.width  = this->sizeSrc.width  ; }
	if( this->sizeDst.height == 0 ) { this->sizeDst.height = this->sizeSrc.height ; }
	
	// check for sufficient information
	HAWAII_ERROR_CONDITIONAL( this->cameraSrc(0,0) <= 0.0
	                       || this->cameraSrc(1,1) <= 0.0
	                       || this->cameraSrc(0,2) <= 0.0
	                       || this->cameraSrc(1,2) <= 0.0
	                       || this->sizeSrc.width  <= 0 || this->sizeDst.width  <= 0
	                       || this->sizeSrc.height <= 0 || this->sizeDst.height <= 0,
	                          "insufficient information" ) ;
	
	// find result camera matrix to maximize result image angle of view avoiding any invalid pixels
	cv::Rect roiValid ;
	this->cameraDstReal = cv::getOptimalNewCameraMatrix( this->cameraSrc,  // input
	                                                     this->distortion, // input
	                                                     this->sizeSrc,    // input
	                                                     0.0,              // parameter: 0.0 (avoid invalid pixels) ... 1.0 (include all source pixels)
	                                                     this->sizeDst,    // input (optional)
	                                                     &roiValid,        // result: region of valid pixels in case parameter above is not zero
	                                                     false ) ;         // parameter: move principal point into the result image's center
	
	// if requested, up-scale axis with shorter focal length
	this->sizeDstEqualFocalLengthReal = this->sizeDst ;
	if( this->equalizeFocalLengths ) {
		const double ratio = this->cameraDst(0,0) / this->cameraDst(1,1) ;
		if( ratio < 1.0 ) { this->cameraDstReal(0,0)                 /= ratio ;
		                    this->cameraDstReal(0,2)                 /= ratio ;
		                    this->sizeDstEqualFocalLengthReal.width  /= ratio ; }
		else {              this->cameraDstReal(1,1)                 *= ratio ;
		                    this->cameraDstReal(1,2)                 *= ratio ;
		                    this->sizeDstEqualFocalLengthReal.height *= ratio ; }
	}
	
	// compute floating point LUTs
	cv::Mat mapUNew, mapVNew ;
	cv::initUndistortRectifyMap( this->cameraSrc,               // input
	                             this->distortion,              // input
	                             cv::Mat(),                     // input (optional): homography e.g. for rectification
	                             this->cameraDst,               // input: result camera matrix to implement
	                             this->sizeDstEqualFocalLength, // input
	                             CV_32FC1,                      // parameter: type of maps below
	                             mapUNew,                       // result: U-coordinate if above is "CV_32FC1", pixel index if "CV_16SC2"
	                             mapVNew ) ;                    // result: V-coordinate ",                      sub-pixel index "
	// update base remapper
	Remapper::update( mapUNew, mapVNew ) ;
	
} // method "Undistorter::update()"

// rectify: c'tor
Rectifier::Rectifier( const cv::Matx33d rotationArg,
                      const cv::Vec3d   translationArg,
                      const Camera      cameraLeftArg,
                      const Camera      cameraRightArg,
                      const Distortion  distortionLeftArg,
                      const Distortion  distortionRightArg,
                      const cv::Size    sizeSrcArg,
                      const cv::Size    sizeDstArg         ) :
	rotation(    rotationArg    ),
	translation( translationArg ),
   distortionLeft(  distortionLeftArg  ),
   distortionRight( distortionRightArg ),
   cameraSrcLeft(  cameraLeftArg  ),
   cameraSrcRight( cameraRightArg ),
	cameraDst( this->cameraDstReal ),
   sizeSrc( sizeSrcArg ),
   sizeDst( sizeDstArg ) {
	
	// initialize if sufficient non-default information is given
	if( !isnan( this->rotation(0,0) ) && !isnan( this->rotation(0,1) ) && !isnan( this->rotation(0,2) ) && !isnan( this->translation(0) )
	 && !isnan( this->rotation(1,0) ) && !isnan( this->rotation(1,1) ) && !isnan( this->rotation(1,2) ) && !isnan( this->translation(1) )
	 && !isnan( this->rotation(2,0) ) && !isnan( this->rotation(2,1) ) && !isnan( this->rotation(2,2) ) && !isnan( this->translation(2) )
	 && this->cameraSrcLeft(0,0) > 0.0 && this->cameraSrcRight(0,0) > 0.0
	 && this->cameraSrcLeft(1,1) > 0.0 && this->cameraSrcRight(1,1) > 0.0
	 && this->cameraSrcLeft(0,2) > 0.0 && this->cameraSrcRight(0,2) > 0.0
	 && this->cameraSrcLeft(1,2) > 0.0 && this->cameraSrcRight(1,2) > 0.0
	 && this->sizeSrc.width  > 0 && this->sizeDst.width  > 0
	 && this->sizeSrc.height > 0 && this->sizeDst.height > 0 ) {
		this->update() ;
	}
}

// rectify: (re-)initialize
void Rectifier::update() {
	
	// handle default result size
	if( this->sizeDst.width  == 0 ) { this->sizeDst.width  = this->sizeSrc.width  ; }
	if( this->sizeDst.height == 0 ) { this->sizeDst.height = this->sizeSrc.height ; }
	
	// check for sufficient information
	HAWAII_ERROR_CONDITIONAL( isnan( this->rotation(0,0) ) || isnan( this->rotation(0,1) ) || isnan( this->rotation(0,2) ) || isnan( this->translation(0) )
	                       || isnan( this->rotation(1,0) ) || isnan( this->rotation(1,1) ) || isnan( this->rotation(1,2) ) || isnan( this->translation(1) )
	                       || isnan( this->rotation(2,0) ) || isnan( this->rotation(2,1) ) || isnan( this->rotation(2,2) ) || isnan( this->translation(2) )
	                       || this->cameraSrcLeft(0,0) <= 0.0 || this->cameraSrcRight(0,0) <= 0.0
	                       || this->cameraSrcLeft(1,1) <= 0.0 || this->cameraSrcRight(1,1) <= 0.0
	                       || this->cameraSrcLeft(0,2) <= 0.0 || this->cameraSrcRight(0,2) <= 0.0
	                       || this->cameraSrcLeft(1,2) <= 0.0 || this->cameraSrcRight(1,2) <= 0.0
	                       || this->sizeSrc.width  <= 0 || this->sizeDst.width  <= 0
	                       || this->sizeSrc.height <= 0 || this->sizeDst.height <= 0,
	                          "insufficient information"  ) ;
	
	// find rectification homographies
	cv::Matx33d homL, homR ;
	cv::Matx34d prjL, prjR ;
	cv::Matx44d dispToDepth ;
	cv::stereoRectify( this->cameraSrcLeft,         // input
	                   this->distortionLeft,        // input
	                   this->cameraSrcRight,        // input
	                   this->distortionRight,       // input
	                   this->sizeSrc,               // input
	                   this->rotation,              // input
	                   this->translation,           // input
	                   homL, homR,                  // results: rectification homographies to be used below
	                   prjL, prjR,                  // results: projection matrices to be used below
	                   dispToDepth,                 // result: helper for converting disparities to 3D points, e.g. via "cv::reprojectImageTo3D()"
	                   cv::CALIB_ZERO_DISPARITY,    // parameter: guarantee identical principal point after rectification
	                   0.0,                         // parameter: 0.0 (avoid invalid pixels) ... 1.0 (include all source pixels)
	                   this->sizeDst            ) ; // parameter (optional): size of result image, defaults to size of input image
	
	// update common result camera parameters
	for( int row = 0 ; row < 3 ; ++row ) {
	for( int col = 0 ; col < 3 ; ++col ) {
		this->cameraDstReal( row, col ) = prjL( row, col ) ;
	} }
	
	// compute floating-point LUTs
	// developer note: see "Undistorter::update()" for explanation of arguments to "cv::initUndistortRectifyMap()"
	cv::Mat mapULeftNew,  mapVLeftNew,
	        mapURightNew, mapVRightNew ;
	cv::initUndistortRectifyMap( this->cameraSrcLeft,  this->distortionLeft,  homL, prjL, this->sizeDst, CV_32FC1, mapULeftNew,  mapVLeftNew  ) ;
	cv::initUndistortRectifyMap( this->cameraSrcRight, this->distortionRight, homR, prjR, this->sizeDst, CV_32FC1, mapURightNew, mapVRightNew ) ;
	
	// update wrapped remappers
	this->remapperLeft.update(  mapULeftNew,  mapVLeftNew  ) ;
	this->remapperRight.update( mapURightNew, mapVRightNew ) ;
	
} // method "Rectifier::update"

} } // namespaces "toast2::stereo"
