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

#pragma once

#include "hawaii/common/hardware.h"
#include "hawaii/GPU/autoMat.h"
#include "hawaii/GPU/nvccFixHeaderAbove.h"
#include <boost/thread/shared_mutex.hpp>
#include "hawaii/GPU/nvccFixHeaderBelow.h"

namespace toast2 {
namespace stereo {

// camera matrix
struct Camera : public cv::Matx33d {
	Camera( const double focalLengthUArg    = -1.0,
	        const double focalLengthVArg    = -1.0,
	        const double principalPointUArg = -1.0,
	        const double principalPointVArg = -1.0 ) ;
	                         Camera(     const cv::Matx33d that ) ;
	cv::Matx< double, 3, 3 > operator =( const cv::Matx33d that ) ;
} ;

// distortion parameters
// developer note: There are only "cv::Vec{2,3,4,6}d" but no "5"...
struct Distortion : public cv::Vec< double, 5 > {
	Distortion( const double radial1Arg     = 0.0,
	            const double radial2Arg     = 0.0,
	            const double tangential1Arg = 0.0,
	            const double tangential2Arg = 0.0,
	            const double radial3Arg     = 0.0 ) ;
	                     Distortion( const cv::Vec< double, 5 > that ) ;
	cv::Vec< double, 5 > operator =( const cv::Vec< double, 5 > that ) ;
} ;

// generic remapping
class Remapper {
	
	// perform transformation on either CPU or GPU, optionally crop to ROI
	public:
	void CPU( const cv::Mat  src,
	                cv::Mat& dst,
	                cv::Rect roi   = cv::Rect(),
	          const int      cores = hawaii::CPUCores ) const ;
	void GPU( const cv::gpu::GpuMat  src,
	                cv::gpu::GpuMat& dst,
	                cv::Rect         roi    = cv::Rect(),
	                cv::gpu::Stream& stream = hawaii::GPU::streamNull ) const ;
	
	// LUTs from result to source coordinates
	// developer note #1: The GPU directly uses floating point "map{U,v}" via texture memory; "map(Sub)Pix" is a faster 
	//                    index-based representation for CPU use only, and is automatically derived from the LUTs given 
	//                    to "update()".
	// developer note #2: The mutex allows multiple readers - i.e. concurrent calls to "CPU()" and "GPU()" - or one 
	//                    writer - i.e. a call to "update()" - at any given time.
	public:
	void update( const cv::Mat mapUArg,
	             const cv::Mat mapVArg ) ;
	protected:
	hawaii::AutoMat mapU, mapV ;
	cv::Mat mapPix, mapSubPix ;
	mutable boost::shared_mutex mutex ;
} ;

// undistort images from a mono-camera
class Undistorter : public Remapper {
	
	// (re-)initialize after setting the parameters below
	public:
	void update() ;
	
	// input and result (read-only) camera calibration parameters
	public:
	      Distortion distortion ;
	      Camera  cameraSrc  ;
	const Camera& cameraDst  ;
	protected:
	Camera cameraDstReal ;
		
	// input and result image sizes, guarantee equal focal lengths
	// user note: Equal focal lengths are achieved by up-scaling the result image along the shorter one's axis, so the 
	//            actual results may be bigger than specified in "sizeDst". If "equalizeFocalLengths" is false,
	//            "sizeDstEqualFocalLength" will be identical to "sizeDst".
	public:
	      cv::Size  sizeSrc,
	                sizeDst ;
	const cv::Size& sizeDstEqualFocalLength ;
	bool equalizeFocalLengths ;
	protected:
	cv::Size sizeDstEqualFocalLengthReal ;
	
	// initialize only if sufficient non-default information is given
	public:
	Undistorter( const Camera     cameraArg               = Camera(),
	             const Distortion distortionArg           = Distortion(),
	             const cv::Size   sizeSrcArg              = cv::Size(),
	             const cv::Size   sizeDstArg              = cv::Size(),
	             const bool       equalizeFocalLengthsArg = false        ) ;
} ;

// rectify images from stereo-cameras, including undistortion
class Rectifier {
	
	// (re-)initialize after setting the parameters below
	public:
	void update() ;
	
	// relative position of right w.r.t. left camera
	public:
	cv::Matx33d rotation    ;
	cv::Vec3d   translation ;
	
	// input and result (read-only, identical for both) camera calibration parameters
	public:
	      Distortion distortionLeft,
	                 distortionRight ;
	      Camera  cameraSrcLeft,
	              cameraSrcRight  ;
	const Camera& cameraDst ;
	protected:
	Camera cameraDstReal ;
	
	// input and result image sizes
	public:
	cv::Size sizeSrc ;
	cv::Size sizeDst ;
	
	// initialize only if sufficient non-default information is given
	public:
	Rectifier( const cv::Matx33d rotationArg        = cv::Matx< double, 3, 3 >( NAN ),
	           const cv::Vec3d   translationArg     = cv::Vec<  double, 3    >( NAN ),
	           const Camera      cameraLeftArg      = Camera(),
	           const Camera      cameraRightArg     = Camera(),
	           const Distortion  distortionLeftArg  = Distortion(),
	           const Distortion  distortionRightArg = Distortion(),
	           const cv::Size    sizeSrcArg         = cv::Size(),
	           const cv::Size    sizeDstArg         = cv::Size()                      ) ;
	
	// perform transformation of both images on either CPU or GPU
	void CPU( const cv::Mat  srcL,
	          const cv::Mat  srcR,
	                cv::Mat& dstL,
	                cv::Mat& dstR,
	                cv::Rect roi   = cv::Rect(),
	          const int      cores = hawaii::CPUCores ) const {
		this->remapperLeft.CPU(  srcL, dstL, roi, cores ) ;
		this->remapperRight.CPU( srcR, dstR, roi, cores ) ;
	}
	void GPU( const cv::gpu::GpuMat  srcL,
	          const cv::gpu::GpuMat  srcR,
	                cv::gpu::GpuMat& dstL,
	                cv::gpu::GpuMat& dstR,
	                cv::Rect         roi    = cv::Rect(),
	                cv::gpu::Stream& stream = hawaii::GPU::streamNull ) const {
		this->remapperLeft.GPU(  srcL, dstL, roi, stream ) ;
		this->remapperRight.GPU( srcR, dstR, roi, stream ) ;
	}
	
	// wrapped remappers to actually do the processing
	protected:
	Remapper remapperLeft,
	         remapperRight ;
} ;

} } // namespaces "toast2::stereo"
