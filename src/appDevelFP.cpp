// Copyright (C) 2013 by:- Institut Eurécom
//                       - Télécom ParisTech
// 
// Author: Aakanksha Rana (rana.aakanksha@gmail.com) and Praveer Singh (praveersingh1990@gmail.com)
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

#include "appDevelFP.h"
#include "mydemo/ImageProcessing/imageProcessing.h"
#include "hawaii/common/optimization.h"
#include "hawaii/common/helpers.h"
#include <SDL/SDL_keysym.h>

// c'tor, always use front camera, initialize added members
DroneAppDevelFP::DroneAppDevelFP(bool IsUsingDense3D) :

// always use front camera
DroneAppBase( false ),
// initialize sub-modules with camera properties

appfollowpersonfp( this->focalLengthFrontU,
		this->principalPointFrontU,
		this->principalPointFrontV),
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
void DroneAppDevelFP::processImageFront( const cv_bridge::CvImage imageFront ) {
	
	// keep original image for visualization, create GPU-suitable copies in color and (slightly blurred) grayscale
	cv::Mat visualization = imageFront.image ;
	hawaii::AutoMat imageBGR, imageGray, imageGraySmooth ;
	//imageFront.image.copyTo( imageBGR.writeCPU( imageFront.image.size(), imageFront.image.type() ) ) ;
	//cv::cvtColor(      imageBGR, imageGray.writeCPU( imageBGR.size(), CV_8UC1 ), cv::COLOR_BGR2GRAY ) ; // TODO multi-core
//	cv::gpu::cvtColor( imageBGR, imageGray.writeGPU( imageBGR.size(), CV_8UC1 ), cv::COLOR_BGR2GRAY ) ; // TODO profile
	//cv::GaussianBlur(      imageGray, imageGraySmooth.writeCPU( imageBGR.size(), CV_8UC1 ), cv::Size( 3, 3 ), 0.0, 0.0 ) ; // TODO multi-core
//	cv::gpu::GaussianBlur( imageGray, imageGraySmooth.writeGPU( imageBGR.size(), CV_8UC1 ), cv::Size( 3, 3 ), 0.0, 0.0 ) ; // TODO profile
	//cv::imshow("asd", visualization);
	cv::resize( visualization, visualization,cv::Size( 0, 0),.5,.5 ) ;
	// go through sub-modules with decreasing priority
	bool commandsSet = false ;
	
	// dense 3D reconstruction: pass on-board odometry pose at time of image to "processImageFront()", but most recent 
	//                          pose to "getCommands()"
	/*if (this->mIsUsingDense3D) {   
		if( 1 && !commandsSet ) {
				std::cout<<"access dense3d";
				this->dense3D.processImageFront( imageGraySmooth,
				                                 this->rotationImageFront,
				                                 this->translationImageFront ) ;
				commandsSet = this->dense3D.getCommands( this->commands,
				                                         this->odoDrone.getRotation(),
				                                         this->odoDrone.getTranslation() ) ;
			}
	}*/
	

	//person tracking using a quadcopter
	if( 1 ) {
		if( this->appfollowpersonfp.processImageFront( visualization,
			                                   this->rotationImageFront,
			                                   this->translationImageFront
			                                          ) ) {
		}
		if( !commandsSet ) {
			commandsSet = this->appfollowpersonfp.getCommands( this->commands, this->odoDrone.getRotation(), 
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

// process keystrokes and gamepad input ???????? NEED 
void DroneAppDevelFP::processKeystrokes() {
	
	// trigger a dense 3D scan with either the "D" key or the yellow gamepad button
	if( this->keystates[ SDLK_d ]
	 || this->buttons[ buttonYellow ] ) {
		this->dense3D.trigger() ;
	}	
	
}
