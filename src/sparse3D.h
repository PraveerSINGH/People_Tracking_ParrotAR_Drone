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


// monocular sparse 3D reconstruction (via visual odometry)
// ========================================================

#pragma once

#include "commands.h"
#include "hawaii/common/timer.h"
#include "hawaii/common/tracker.h"
#include <opencv2/core/core.hpp>
#include <memory>

class VisualOdometryMono ;

// extensions to monocular visual odometry from "libviso2"
class Sparse3D {
	
	// c'tor with camera properties
	// user note: Before passing images, you need to undistort and stretch them such that the focal lengths along the u- 
	//            and v-axes are identical. "DroneAppBase" already does that.
	public:
	Sparse3D( const double focalLength,
	          const double principalPointU,
	          const double principalPointV ) ;
	
	// flight parameters: Only forward motion and yaw rotation will be based on 3D points. Vertical and sideways motion 
	//                    are phase-shifted vertical and sideways oscillations to ensure sufficient motion for sparse 
	//                    3D reconstruction. Combining both results in a "corkscrew"-shaped flight trajectory.
	// user note: These parameters need to be tuned together. "DroneAppDevel" allows tuning all except frequency via the 
	//            "q/w", "a/s" and "y/x" keys.
	float corkscrewAmplitudeY,  // amplitude of the horizontal oscillation in [0.0 1.0]
	      corkscrewAmplitudeZ ; // amplitude of the vertical oscillation in meters
	double corkscrewFrequency ; // frequency of both oscillations in hertz
	double corkscrewPhase ; // phase between both oscillations in degrees
	protected:
	hawaii::common::Timer corkscrewTimer ;
	
	// process the latest image, together with the drone's on-board odometry pose at the time is was captured
	// user note: If available, pass a grayscale image to avoid internal conversion.
	public:
	bool processImageFront( const cv::Mat   image,
	                        const cv::Vec3d rotationGlobal,
	                        const cv::Vec3d translationGlobal ) ;
	protected:
	cv::Size sizeImage ;
	double scaleFactor ;
	
	// get control commands: See flight parameters above. Vertical and sideways motion are always set - therefore "true" 
	//                       is always returned. Forward and yaw motion occur only after successful 3D reconstruction.
	public:
	bool getCommands(       DroneCommands& commands,
	                  const cv::Vec3d      translationGlobal ) const ;
	
	// get results: - intermediately-computed 3D points for each matched feature
	//              - relative motion from previous successfully-used image to current one, in previous image's coordinates
	//              - accumulated pose since last call to "resetPose()", after fusion of visual and on-board odometry
	public:
	void getPoints3D( cv::Mat&             points3D,
	                  std::vector< bool >& inliers  ) const ;
	void getMotion( cv::Matx33d& rotationLocal,
	                cv::Vec3d&   translationLocal ) const ;
	void getPose(   cv::Matx33d& rotationGlobal,
	                cv::Vec3d&   translationGlobal ) const ;
	void resetPose() ;
	protected:
	cv::Matx44d pose ;
	
	// draw matches and projected 3D points onto image
	public:
	void visualize( cv::Mat& visualization ) const ;
	protected:
	mutable hawaii::common::Tracker1DPT1 trackerAngleDelta,
	                                     trackerForwardSpeed ;
	
	// wrap visual odometry implementation via "pimpl" pattern, results of previous calls
	// developer note: "std::auto_ptr" is officially deprecated, but "boost::scoped_ptr" or "std::unique_ptr" would 
	//                 require the d'tor of "impl" to be defined (!) in the source file of this class. This sucks...
	protected:
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
   std::auto_ptr< VisualOdometryMono > visoPtr ;
	#pragma GCC diagnostic pop
	bool   visoSuccessPrev ;
	size_t visoFailsConsecMax,
	       visoFailsConsec    ;
	
	// odometry based on drone's additional sensors as a second guess for visual odometry scale estimation, pose of 
	// previous image successfully used by visual odometry
	protected:
	cv::Vec3d rotationGlobalOnboardPrev,
	          translationGlobalOnboardPrev ;
	
} ; // class "Sparse3D"
