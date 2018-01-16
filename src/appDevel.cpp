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


// development area
// ================

#include "appDevel.h"
#include "mydemo/ImageProcessing/imageProcessing.h"
#include "hawaii/common/optimization.h"
#include "hawaii/common/helpers.h"
#include <SDL/SDL_keysym.h>

// c'tor, always use front camera, initialize added members
DroneAppDevel::DroneAppDevel(bool IsUsingDense3D) :

// always use front camera
DroneAppBase( false ),
// initialize sub-modules with camera properties
sparse3D( this->focalLengthFrontU,
		this->principalPointFrontU,
		this->principalPointFrontV ),
		dense3D(  this->focalLengthFrontU,
				this->principalPointFrontU,
				this->principalPointFrontV ),
				navigator( this->focalLengthFrontU,
						this->focalLengthFrontV,
						this->principalPointFrontU,
						this->principalPointFrontV ),

						// detect when no commands have been set for a given time, hover in place afterwards
						timerCommandsLastSet( true ),
						timeCommandsLastSetMax( 0.25 ) {
	this->mIsUsingDense3D = IsUsingDense3D;
}

// process front image
void DroneAppDevel::processImageFront( const cv_bridge::CvImage imageFront ) {
	
	// keep original image for visualization, create GPU-suitable copies in color and (slightly blurred) grayscale
	cv::Mat visualization = imageFront.image ;
	hawaii::AutoMat imageBGR, imageGray, imageGraySmooth ;
	imageFront.image.copyTo( imageBGR.writeCPU( imageFront.image.size(), imageFront.image.type() ) ) ;
	cv::cvtColor(      imageBGR, imageGray.writeCPU( imageBGR.size(), CV_8UC1 ), cv::COLOR_BGR2GRAY ) ; // TODO multi-core
//	cv::gpu::cvtColor( imageBGR, imageGray.writeGPU( imageBGR.size(), CV_8UC1 ), cv::COLOR_BGR2GRAY ) ; // TODO profile
	cv::GaussianBlur(      imageGray, imageGraySmooth.writeCPU( imageBGR.size(), CV_8UC1 ), cv::Size( 3, 3 ), 0.0, 0.0 ) ; // TODO multi-core
//	cv::gpu::GaussianBlur( imageGray, imageGraySmooth.writeGPU( imageBGR.size(), CV_8UC1 ), cv::Size( 3, 3 ), 0.0, 0.0 ) ; // TODO profile
	cv::imshow("asd", visualization);
	// go through sub-modules with decreasing priority
	bool commandsSet = false ;
	
	// dense 3D reconstruction: pass on-board odometry pose at time of image to "processImageFront()", but most recent 
	//                          pose to "getCommands()"
	if (this->mIsUsingDense3D) {
		if( 1 && !commandsSet ) {
				std::cout<<"access dense3d";
				this->dense3D.processImageFront( imageGraySmooth,
				                                 this->rotationImageFront,
				                                 this->translationImageFront ) ;
				commandsSet = this->dense3D.getCommands( this->commands,
				                                         this->odoDrone.getRotation(),
				                                         this->odoDrone.getTranslation() ) ;
			}
	}
	
	// landmark-based navigation
	if( 1 && !commandsSet ) {
		this->navigator.processImageFront( imageBGR,
		                                   this->rotationImageFront ) ;
		commandsSet = this->navigator.getCommands( this->commands,
		                                           this->odoDrone.getRotation() ) ;
	}
	
	// continue visual odometry and sparse 3D reconstruction no matter if commands are already set, but do "corkscrew" 
	// flight with lowest priority: see note at dense 3D
	if( 1 ) {
		if( this->sparse3D.processImageFront( imageGraySmooth,
			                                   this->rotationImageFront,
			                                   this->translationImageFront ) ) {
		}
		this->sparse3D.visualize( visualization ) ;
		if( !commandsSet ) {
			commandsSet = this->sparse3D.getCommands( this->commands,
			                                          this->odoDrone.getTranslation() ) ;
		}
	}
	
	// hover in place if no commands have been set for a given time
	if( commandsSet ) { this->timerCommandsLastSet.tic() ; }
	if( this->timerCommandsLastSet.toc() / 1000.0 > this->timeCommandsLastSetMax ) {
		this->commands.reset() ;
	}
	
	// finally show the visualization image
	if( 1
	 || this->keystates[ SDLK_v ]
	 || this->buttons[ buttonBlue ] ) {
		HAWAII_IMSHOW( visualization ) ;
	}
	cv::waitKey( 1 ) ;
}

// process keystrokes and gamepad input
void DroneAppDevel::processKeystrokes() {
	
	// trigger a dense 3D scan with either the "D" key or the yellow gamepad button
	if( this->keystates[ SDLK_d ]
	 || this->buttons[ buttonYellow ] ) {
		this->dense3D.trigger() ;
	}
	
	// adjust "corkscrew" flight parameters during runtime
	if( this->keystates[ SDLK_1 ] ) { this->sparse3D.corkscrewFrequency  -= 0.02 ; HAWAII_PRINT( this->sparse3D.corkscrewFrequency  ) ; usleep( 100000 ) ; }
	if( this->keystates[ SDLK_2 ] ) { this->sparse3D.corkscrewFrequency  += 0.02 ; HAWAII_PRINT( this->sparse3D.corkscrewFrequency  ) ; usleep( 100000 ) ; }
	if( this->keystates[ SDLK_q ] ) { this->sparse3D.corkscrewAmplitudeY -= 0.02 ; HAWAII_PRINT( this->sparse3D.corkscrewAmplitudeY ) ; usleep( 100000 ) ; }
	if( this->keystates[ SDLK_w ] ) { this->sparse3D.corkscrewAmplitudeY += 0.02 ; HAWAII_PRINT( this->sparse3D.corkscrewAmplitudeY ) ; usleep( 100000 ) ; }
	if( this->keystates[ SDLK_a ] ) { this->sparse3D.corkscrewAmplitudeZ -= 0.02 ; HAWAII_PRINT( this->sparse3D.corkscrewAmplitudeZ ) ; usleep( 100000 ) ; }
	if( this->keystates[ SDLK_s ] ) { this->sparse3D.corkscrewAmplitudeZ += 0.02 ; HAWAII_PRINT( this->sparse3D.corkscrewAmplitudeZ ) ; usleep( 100000 ) ; }
	if( this->keystates[ SDLK_y ] ) { this->sparse3D.corkscrewPhase      -= 5.0  ; HAWAII_PRINT( this->sparse3D.corkscrewPhase      ) ; usleep( 100000 ) ; }
	if( this->keystates[ SDLK_x ] ) { this->sparse3D.corkscrewPhase      += 5.0  ; HAWAII_PRINT( this->sparse3D.corkscrewPhase      ) ; usleep( 100000 ) ; }
}
