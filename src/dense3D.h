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


// monocular dense 3D reconstruction (from images before and after height change)
// ==============================================================================

#pragma once

#include "commands.h"
#include "hawaii/common/tracker.h"
#include <opencv2/core/core.hpp>
#include <boost/thread/thread.hpp>
#include <vector>

class VisualOdometryMono ;

// wrapper around moving object segmentation, rectification and dense 3D reconstruction
class Dense3D {
	
	// c'tor with camera properties
	// user note: Before passing images, you need to undistort and stretch them such that the focal lengths along the u- 
	//            and v-axes are identical. "DroneAppBase" already does that.
	public:
	Dense3D( const double focalLength,
	         const double principalPointU,
	         const double principalPointV ) ;
	protected:
	const cv::Matx33d camera ;
	
	// height change in meters (upwards positive), forward/backward and left/right stabilization mode during height change
	public:
	double altitudeDelta ;
	enum Stabilization {
		stabilizationNone,
		stabilizationVisual,
		stabilizationOnboard
	} stabilization ;
	
	// times in seconds to hover in place before and after height change to mask moving objects, number of images to 
	// buffer during those times (Currently however, only 1st and last each are used.)
	public:
	double timeHoverBefore,
	       timeHoverAfter  ;
	size_t numberHoverBefore,
	       numberHoverAfter ;
	protected:
	hawaii::common::Timer timerHover ;
	size_t indexReferenceBefore,
	       indexReferenceAfter  ;
	
	// internal state
	protected:
	enum State {
		stateBeforeHeightChange, // capture images before the height change to mask moving objects
		stateDuringHeightChange, // perform height change via closed-loop control to "build" a stereo camera
		stateAfterHeightChange,  // capture images after the height change to mask moving objects
		stateProcessing,         // asynchronously perform dense 3D reconstruction based on the captured images
		stateInactive            // don't do anything, particularly don't set flight control commands
	} state ;
	
	// buffer images and poses from drone's on-board odometry before and after height change
	// user note: If available, pass a grayscale image to avoid internal conversion. 
	public:
	void processImageFront( const cv::Mat   image,
	                        const cv::Vec3d rotationGlobal,
	                        const cv::Vec3d translationGlobal ) ;
	protected:
	std::vector< cv::Mat   > imagesBefore,       imagesAfter       ;
	std::vector< cv::Vec3d > rotationsBefore,    rotationsAfter    ;
	std::vector< cv::Vec3d > translationsBefore, translationsAfter ;
	
	// get control commands depending on state: When inactive, "commands" is not modified and "false" is returned. 
	public:
	bool getCommands(       DroneCommands& commands,
	                  const cv::Vec3d      rotationGlobal,
	                  const cv::Vec3d      translationGlobal ) ;
	
	// trigger the scan flight maneuver: returns "false" if still busy with previous scan
	public:
	bool trigger() ;
	
	// compute and return distance map and moving object mask: "getResults()" returns "false" if the results are not yet 
	//                                                         available. The computation runs asynchronously directly 
	//                                                         after the last image has been buffered.
	public:
	bool getResults( cv::Mat& distanceArg,            // distances in meters, single-precision floating-point 
	                 cv::Mat& motionMaskArg ) const ; // likelihood of moving object, in [0,255]
	protected:
	cv::Mat distance,
	        motionMask ;
	void computeResults() ;
	boost::thread threadComputeResults ;
	
	// use visual odometry to compensate for forward/backward and left/right motions during the height change, wrap its 
	// implementation via "pimpl" pattern
	// developer note: "std::auto_ptr" is officially deprecated, but "boost::scoped_ptr" or "std::unique_ptr" would 
	//                 require the d'tor of "impl" to be defined (!) in the source file of this class. This sucks...
	protected:
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
   std::auto_ptr< VisualOdometryMono > visoPtr ;
	#pragma GCC diagnostic pop
	void visoInit( const cv::Mat   imageBefore,
	               const cv::Vec3d rotationGlobal,
	               const cv::Vec3d translationGlobal ) ;
	void visoProcess( const cv::Mat   imageDuring,
	                  const cv::Vec3d rotationGlobal,
	                  const cv::Vec3d translationGlobal ) ;
	hawaii::common::Tracker1DPT1 errorVisoForward,
	                             errorVisoLeft    ;
	bool errorVisoValid ;
	
} ; // class "Dense3D"
