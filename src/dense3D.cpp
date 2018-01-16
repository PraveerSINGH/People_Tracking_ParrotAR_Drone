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

#include "dense3D.h"
#include "odometryDrone.h"
#include "hawaii/common/helpers.h"
#include "hawaii/common/error.h"
#include "viso_mono.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// c'tor with camera properties
Dense3D::Dense3D( const double focalLength,
                  const double principalPointU,
                  const double principalPointV ) :
	
	// camera matrix
	camera( focalLength, 0.0,         principalPointU,
	        0.0,         focalLength, principalPointV,
	        0.0,         0.0,         1.0             ),
	
	// height change, stabilization mode
	altitudeDelta( 0.15 ),
	stabilization( stabilizationNone ),
	
	// times to hover in place before and after height change, number of images to buffer
	timeHoverBefore( 0.75 ),
	timeHoverAfter(  0.75 ),
	numberHoverBefore( 3 ),
	numberHoverAfter(  3 ),
	indexReferenceBefore( numberHoverBefore / 2 ),
	indexReferenceAfter(  numberHoverAfter  / 2 ),
	
	// internal state
	state( stateInactive ),
	
	// forward/backward and left/right motions during the height change
	errorVisoForward( 0.05 ),
	errorVisoLeft(    0.05 ),
	errorVisoValid( false ) {
}

// buffer images and poses from drone's on-board odometry before and after height change
void Dense3D::processImageFront( const cv::Mat   image,
                                 const cv::Vec3d rotationGlobal,
                                 const cv::Vec3d translationGlobal ) {
	// check input
	HAWAII_ERROR_CONDITIONAL( image.empty(),
	                          "Image must not be empty." ) ;
	HAWAII_ERROR_CONDITIONAL( image.type() != CV_8UC1
	                       && image.type() != CV_8UC3,
	                          "Image type must be \"CV_8UC1\" or \"CV_8UC3\"." ) ;
	
	// convert to grayscale if necessary
	cv::Mat imageGray ;
	if( image.type() == CV_8UC1 ) { imageGray = image ; }
	else { cv::cvtColor( image, imageGray, cv::COLOR_BGR2GRAY ) ; }
	
	std::cout <<this->state;
	// buffer some images while hovering before a height change
	//if( this->state == stateBeforeHeightChange) {
	//	std::cout << "INFO: dense 3D: problem, can't access state HEIGHT CHANGE" << std::endl ;
	//}
	if( this->state == stateBeforeHeightChange
	 && ( this->timerHover.toc() / 1000.0 > this->timeHoverBefore / this->numberHoverBefore ) ) {
		
		// buffer all data
		this->imagesBefore.push_back(       imageGray         ) ;
		this->rotationsBefore.push_back(    rotationGlobal    ) ;
		this->translationsBefore.push_back( translationGlobal ) ;
		this->timerHover.tic() ;
		std::cout << "INFO: dense 3D: buffered image BEFORE height change" << std::endl ;
		
		// initialize visual odometry with reference image before height change
		if( this->stabilization == stabilizationVisual
		 && this->imagesBefore.size() - 1 == this->indexReferenceBefore ) {
			this->visoInit( imageGray,
			                rotationGlobal,
			                translationGlobal ) ;
		}
			
		// once buffer is full...
		if( this->imagesBefore.size() >= this->numberHoverBefore ) {
			
			// change state
			this->state = this->stateDuringHeightChange ;
			std::cout << "INFO: dense 3D: changed state to HEIGHT CHANGE" << std::endl ;
		}
	}
	
	// while performing height change...
	if( this->state == stateDuringHeightChange ) {
		
		// use visual odometry to update relative pose w.r.t. last image before height change
		if( this->stabilization == stabilizationVisual ) {
			this->visoProcess( imageGray,
			                   rotationGlobal,
			                   translationGlobal ) ;
		}
	}
	
	// buffer some images while hovering after a height change
	if( this->state == stateAfterHeightChange
	 && ( this->timerHover.toc() / 1000.0 > this->timeHoverAfter / this->numberHoverAfter ) ) {
		
		// buffer all data
		this->imagesAfter.push_back(       imageGray             ) ;
		this->rotationsAfter.push_back(    rotationGlobal    ) ;
		this->translationsAfter.push_back( translationGlobal ) ;
		this->timerHover.tic() ;
		std::cout << "INFO: dense 3D: buffered image AFTER height change" << std::endl ;
		
		// once buffer is full...
		if( this->imagesAfter.size() >= this->numberHoverAfter ) {
			
			// change state
			this->state = this->stateProcessing ;
			std::cout << "INFO: dense 3D: changed state to PROCESSING BUFFERS" << std::endl ;
			
			// launch thread to asynchronously process the buffered data
			this->threadComputeResults = boost::thread( &Dense3D::computeResults, this ) ;
		}
	}
	
} // method "Dense3D::processImageFront()"

// get control commands depending on state
bool Dense3D::getCommands(       DroneCommands& commands,
                           const cv::Vec3d      rotationGlobal,
                           const cv::Vec3d      translationGlobal ) {
	
	// handle all states independently
	switch( this->state ) {
			
		// hover in place
		case stateBeforeHeightChange :
		case stateAfterHeightChange  : {
			
			// engage on-board stabilization
			commands.hover() ;
			
		} break ;
		
		// build a vertical stereo camera by changing height
		case stateDuringHeightChange : {
			
			// closed-loop control of forward/backward and left/right motions based on visual odometry
			double errorForward = 0.0,
			       errorLeft    = 0.0 ;
			switch( this->stabilization ) {
				
				// try not to move forward/backward or left/right via open-loop control
				case stabilizationNone : {
					commands.movement.linear.x = 0.0f ;
					commands.movement.linear.y = 0.0f ;
				} break ;
				
				// compute and use errors from on-board odometry
				case stabilizationOnboard : {
					const cv::Vec3d translDeltaGlobal = translationGlobal - this->translationsBefore[ this->indexReferenceBefore ] ;
					const cv::Vec3d translDeltaLocal = OdometryDrone::rotateTranslation( translDeltaGlobal, rotationGlobal( 1 ) ) ;
					errorForward = - translDeltaLocal( 2 ) ;
					errorLeft    =   translDeltaLocal( 0 ) ;
					commands.controlLinX( errorForward ) ;
					commands.controlLinY( errorLeft    ) ;
				} break ;
				
				// use errors from visual odometry
				case stabilizationVisual : {
					errorForward = this->errorVisoForward ;
					errorLeft    = this->errorVisoLeft    ;
					commands.controlLinX( errorForward ) ;
					commands.controlLinY( errorLeft    ) ;
				} break ;
			}
			
			// closed-loop control of altitude and yaw always based on on-board sensors
			const double altitudeBefore = - this->translationsBefore.back()( 1 ),
			             altitudeCurr   = -       translationGlobal(         1 ),
			             yawBefore      = - this->rotationsBefore.back()(    1 ),
			             yawCurr        = -       rotationGlobal(            1 ) ;
			double errorUp  = altitudeBefore - altitudeCurr + this->altitudeDelta,
			       errorYaw = yawBefore      - yawCurr                            ;
			if( errorYaw >   CV_PI ) { errorYaw -= 2.0 * CV_PI ; }
			if( errorYaw < - CV_PI ) { errorYaw += 2.0 * CV_PI ; }
			commands.controlLinZ( errorUp  ) ;
			commands.controlAngZ( errorYaw ) ;
			
			// crank height control up if error is large
			#if 1
				commands.controllerLinZ.disableIntegrator = false ;
				if( 0.0     < this->altitudeDelta
				 && errorUp > this->altitudeDelta * 0.25
				 && commands.movement.linear.z <   0.75f ) {
					commands.controllerLinZ.disableIntegrator = true ;
					commands.movement.linear.z =    0.75f ;
				}
				if( 0.0     > this->altitudeDelta
				 && errorUp < this->altitudeDelta * 0.25
				 && commands.movement.linear.z > - 0.75f ) {
					commands.controllerLinZ.disableIntegrator = true ;
					commands.movement.linear.z =  - 0.75f ;
				}
			#endif
			
			// show the errors
			HAWAII_PRINT( errorForward ) ;
			HAWAII_PRINT( errorLeft    ) ;
			HAWAII_PRINT( errorUp      ) ;
			HAWAII_PRINT( errorYaw     ) ;
			std::cout << std::endl ; //*/
			
			// suppress any specific flight maneuver
			commands.maneuver = DroneCommands::maneuverNone ;
			
			// check if height change has been completed
			if( ( this->stabilization != stabilizationVisual || this->errorVisoValid )
			 && ( this->stabilization == stabilizationNone   || fabs( errorForward ) < 0.20 * this->altitudeDelta ) // greater impact than others
			 && ( this->stabilization == stabilizationNone   || fabs( errorLeft    ) < 0.20 * this->altitudeDelta )
			 &&                                                 fabs( errorUp      ) < 0.20 * this->altitudeDelta
			 &&                                                 fabs( errorYaw     ) < 5.0 / 180.0 * CV_PI ) {
				
				// change state
				this->state = stateAfterHeightChange ;
				this->timerHover.tic() ;
				std::cout << "INFO: dense 3D: changed state to HOVER AFTER height change" << std::endl ;
			}
			
		} break ;
		
		// do not modify "commands"
		case stateProcessing :
		case stateInactive   : {
			
		} break ;
		
	} // switch "all states"
	
	// return "true" only if "commands" has been modified
	return ( this->state != stateInactive
	      && this->state != stateProcessing ) ;
	
} // method "Dense3D::getCommands()"

// trigger the scan flight maneuver
bool Dense3D::trigger() {
	
	// indicate that still busy with previous scan
	if( this->state == stateProcessing ) {
		return false ;
	}
	
	// clear all buffers and results for next iteration
	this->imagesBefore.clear() ;
	this->imagesAfter.clear()  ;
	this->rotationsBefore.clear() ;
	this->rotationsAfter.clear()  ;
	this->translationsBefore.clear() ;
	this->translationsAfter.clear()  ;
	this->distance.release() ;
	this->motionMask.release() ;
	
	// change state
	this->state = stateBeforeHeightChange ;
	this->timerHover.tic() ;
	std::cout << "INFO: dense 3D: changed state to HOVER BEFORE height change" << std::endl ;
	return true ;
}

// return distance map and moving object mask
bool Dense3D::getResults( cv::Mat& distanceArg,
                          cv::Mat& motionMaskArg ) const {
	
	// only return previously computed results while next scan not yet triggered
	if( this->state != stateProcessing
	 && !this->distance.empty()
	 && !this->motionMask.empty() ) {
		
		// set arguments
		distanceArg   = this->distance   ;
		motionMaskArg = this->motionMask ;
		return true ;
	}
	
	// indicate unmodified arguments
	return false ;
}

// compute distance map and moving object mask
void Dense3D::computeResults() {
	
	// save content of buffers of YAML file for off-line processing  
	static size_t fileCounter = 0 ;
	std::ostringstream stream ;
	stream << "data/dense3D_" << this->altitudeDelta << "m_" << fileCounter++ << ".yml" ;
	cv::FileStorage fileStorage( stream.str(), cv::FileStorage::WRITE ) ;
	fileStorage << "camera"              << (cv::Mat)this->camera                  ;
	fileStorage << "rotationsBefore0"    << (cv::Mat)this->rotationsBefore[    0 ] ;
	fileStorage << "rotationsBefore1"    << (cv::Mat)this->rotationsBefore[    1 ] ;
	fileStorage << "rotationsBefore2"    << (cv::Mat)this->rotationsBefore[    2 ] ;
	fileStorage << "rotationsAfter0"     << (cv::Mat)this->rotationsAfter[     0 ] ;
	fileStorage << "rotationsAfter1"     << (cv::Mat)this->rotationsAfter[     1 ] ;
	fileStorage << "rotationsAfter2"     << (cv::Mat)this->rotationsAfter[     2 ] ;
	fileStorage << "translationsBefore0" << (cv::Mat)this->translationsBefore[ 0 ] ;
	fileStorage << "translationsBefore1" << (cv::Mat)this->translationsBefore[ 1 ] ;
	fileStorage << "translationsBefore2" << (cv::Mat)this->translationsBefore[ 2 ] ;
	fileStorage << "translationsAfter0"  << (cv::Mat)this->translationsAfter[  0 ] ;
	fileStorage << "translationsAfter1"  << (cv::Mat)this->translationsAfter[  1 ] ;
	fileStorage << "translationsAfter2"  << (cv::Mat)this->translationsAfter[  2 ] ;
	fileStorage << "imagesBefore0"       <<          this->imagesBefore[       0 ] ;
	fileStorage << "imagesBefore1"       <<          this->imagesBefore[       1 ] ;
	fileStorage << "imagesBefore2"       <<          this->imagesBefore[       2 ] ;
	fileStorage << "imagesAfter0"        <<          this->imagesAfter[        0 ] ;
	fileStorage << "imagesAfter1"        <<          this->imagesAfter[        1 ] ;
	fileStorage << "imagesAfter2"        <<          this->imagesAfter[        2 ] ;
	fileStorage.release() ;
	
	// show the buffered images
	HAWAII_IMSHOW( this->imagesBefore[ 0 ] ) ;
	HAWAII_IMSHOW( this->imagesBefore[ 1 ] ) ;
	HAWAII_IMSHOW( this->imagesBefore[ 2 ] ) ;
	HAWAII_IMSHOW( this->imagesAfter[  0 ] ) ;
	HAWAII_IMSHOW( this->imagesAfter[  1 ] ) ;
	HAWAII_IMSHOW( this->imagesAfter[  2 ] ) ; //*/
	
	// change state when done
	this->state = stateInactive ;
	std::cout << "INFO: dense 3D: changed state to INACTIVE" << std::endl ;
}

// initialize visual odometry with last image before height change
void Dense3D::visoInit( const cv::Mat   imageBefore,
                        const cv::Vec3d rotationGlobal,
                        const cv::Vec3d translationGlobal ) {
	
	// instantiate wrapped visual odometry with required parameters
	VisualOdometryMono::parameters params ;
	params.inlier_threshold =   0.00010 ; //   0.00001
	params.motion_threshold = 100.0     ; // 100.0 TODO test
	params.match.match_binsize   =  32 ; //  50 TODO test
	params.match.match_radius    = 384 ; // 200
	params.match.half_resolution =   1 ; //   1
	params.match.refinement      =   1 ; //   1
	params.bucket.max_features  =  2 ; //  2 TODO test
	params.bucket.bucket_width  = 32 ; // 50 TODO test
	params.bucket.bucket_height = 32 ; // 50 TODO test
	params.calib.f  = ( this->camera( 0, 0 ) + this->camera( 1, 1 ) ) * 0.5 ;
	params.calib.cu = this->camera( 0, 2 ) ;
	params.calib.cv = this->camera( 1, 2 ) ;
	this->visoPtr.reset( new VisualOdometryMono( params ) ) ;
	
	// update ground plane parameters used to resolve the monocular scale ambiguity
	this->visoPtr->param.height = 0.04 - translationGlobal( 1 ) ; // sign different from default "computer vision coordinates", camera higher than sensor
	this->visoPtr->param.pitch = rotationGlobal( 0 ) ;
	this->visoPtr->param.roll  = rotationGlobal( 2 ) ;
	
	// add the image as both previous and current for good measure
	int32_t dims[] = { imageBefore.cols,
	                   imageBefore.rows,
	                   imageBefore.step } ;
	this->visoPtr->process( imageBefore.data, dims, false ) ;
	this->visoPtr->process( imageBefore.data, dims, false ) ;
}

// compute relative position between last image before and each image during height change
void Dense3D::visoProcess( const cv::Mat   imageDuring,
                           const cv::Vec3d rotationGlobal,
                           const cv::Vec3d translationGlobal ) {
	
	// update ground plane parameters used to resolve the monocular scale ambiguity
	this->visoPtr->param.height = 0.04 - translationGlobal( 1 ) ; // sign different from default "computer vision coordinates", camera higher than sensor
	this->visoPtr->param.pitch = rotationGlobal( 0 ) ;
	this->visoPtr->param.roll  = rotationGlobal( 2 ) ;
	
	// always keep the initially set last image before the height change
	int32_t dims[] = { imageDuring.cols,
	                   imageDuring.rows,
	                   imageDuring.step } ;
	if( this->visoPtr->process( imageDuring.data, dims, true ) ) {
		
		// if odometry has been successful, get the relative motion w.r.t. the last image before the height change
		const Matrix motionDeltaViso = this->visoPtr->getMotion() ;
		
		// use on-board odometry to correct the scale: While on-board odometry is more susceptible to noise, visual 
		//                                             odometry may pick a wrong set of 3D points when fitting the 
		//                                             ground plane. Therefore, we fully trust visual odometry if its 
		//                                             translation is similar enough to on-board odometry. Otherwise we 
		//                                             scale visual odometry results to make translation lengths or 
		//                                             altitude components match.
		// developer note: Almost the same code exists in "Sparse3D::processImageFront()" - keep them synchronized!
		const cv::Vec3d translDeltaGlobalOnboard = translationGlobal - this->translationsBefore.back() ;
		const double lengthOnboard = sqrt( translDeltaGlobalOnboard( 0 ) * translDeltaGlobalOnboard( 0 )
		                                 + translDeltaGlobalOnboard( 1 ) * translDeltaGlobalOnboard( 1 )
		                                 + translDeltaGlobalOnboard( 2 ) * translDeltaGlobalOnboard( 2 ) ),
		             lengthViso    = sqrt( motionDeltaViso.val[ 0 ][ 3 ] * motionDeltaViso.val[ 0 ][ 3 ]
		                                 + motionDeltaViso.val[ 1 ][ 3 ] * motionDeltaViso.val[ 1 ][ 3 ]
		                                 + motionDeltaViso.val[ 2 ][ 3 ] * motionDeltaViso.val[ 2 ][ 3 ] ) ;
		double scaleFactor = lengthOnboard / lengthViso ;
		if( scaleFactor > 0.9
		 && scaleFactor < 1.1 ) {
			scaleFactor = 1.0 ;
		}
		this->errorVisoForward(   motionDeltaViso.val[ 2 ][ 3 ] * scaleFactor ) ;
		this->errorVisoLeft(    - motionDeltaViso.val[ 0 ][ 3 ] * scaleFactor ) ;
		this->errorVisoValid = true ;
		std::cout << "INFO: dense 3D: visual odometry SUCCEEDED"
//		          << ", scaleFactor = " << scaleFactor
//		          << ", lengthOnboard = " << lengthOnboard
//		          << ", lengthViso = " << lengthViso
		          << std::endl ;
		
		// visualize results
/*		cv::Mat visu ;
		cv::addWeighted( imageDuring, 0.5, this->imagesBefore.back(), 0.5, 0.0, visu ) ;
		cv::cvtColor( visu, visu, CV_GRAY2BGR ) ;
		std::vector< Matcher::p_match > matches = this->visoPtr->getMatches() ;
		for( const auto& match : matches ) {
			cv::line( visu,
			          cv::Point( match.u1c, match.v1c ),
			          cv::Point( match.u1p, match.v1p ),
			          cv::Scalar( 0, 127, 255 ), 2, 4 ) ;
		}
		HAWAII_IMSHOW( visu ) ; //*/
		
	} else {
		
		// indicate if odometry has failed, fade the error trackers down towards zero
		this->errorVisoValid = false ;
		this->errorVisoForward( 0.0 ) ;
		this->errorVisoLeft(    0.0 ) ;
		std::cout << "INFO: dense 3D: visual odometry FAILED" << std::endl ;
	}
	
} // method "Dense3D::visoProcess()"
