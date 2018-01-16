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
// You should have received a copy of the GNU General Public License along with demoARDrone. If not, see 
// <http://www.gnu.org/licenses/>.


// follow a red line on the ground
// ===============================

#include "appFollowLine.h"
#include "hawaii/common/helpers.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <SDL/SDL_keysym.h>
#include <SDL/SDL_joystick.h>

// helpers only used here
namespace {
	
	// find the most prominent line's offset and angle
	bool findLine( const cv::Mat  imageBottom,
	                     cv::Mat& visualization,
	                     double&  offsetPix,
	                     double&  angleDeg,
	               const double   principalPointU ) {
		
		// scale down
		constexpr double scaleFactor = 0.5 ;
		cv::Mat imageBottomScaled ;
		cv::resize( imageBottom, imageBottomScaled,
		            cv::Size( 0, 0 ), scaleFactor, scaleFactor ) ;
		
		// convert to HSV and apply thresholds
		cv::Mat hsv ;
		cv::cvtColor( imageBottomScaled, hsv, CV_BGR2HSV ) ;
		
		// threshold
		cv::Mat orange, violet, red ;
		cv::inRange( hsv,
		             cv::Scalar(   0,  64,  96 ),
		             cv::Scalar(  20, 255, 255 ),
		             orange                      ) ;
		cv::inRange( hsv,
		             cv::Scalar( 160,  64,  96 ),
		             cv::Scalar( 180, 255, 255 ),
		             violet                      ) ;
		red = orange | violet ;
		
		// visualize segmentation
		cv::cvtColor( red, visualization, CV_GRAY2BGR ) ;
		visualization &= imageBottomScaled ;
		
		// detect lines: 1px distance resolution, 1° angle resolution, 100 pixels in the original image required
		std::vector< cv::Vec2f > lines ;
		cv::HoughLines( red, lines, 1.0, CV_PI / 180.0, 100 * scaleFactor * scaleFactor ) ;
		if( !lines.empty() ) {
			
			// extract parameters
			float rho   = lines[ 0 ][ 0 ],
			      theta = lines[ 0 ][ 1 ] ;
			
			// convert to outside representation
			offsetPix = ( rho / cos( theta ) / scaleFactor ) - principalPointU ;
			angleDeg  = theta * 180.0 / CV_PI ; if( angleDeg > 90.0 ) { angleDeg -= 180.0 ; }
			
			// draw line
			cv::Point pt1, pt2 ;
			double a = cos( theta ),
			       b = sin( theta ) ;
			double x0 = a * rho,
			       y0 = b * rho ;
			pt1.x = cvRound( x0 + 1000 * ( -b ) ) ;
			pt1.y = cvRound( y0 + 1000 * (  a ) ) ;
			pt2.x = cvRound( x0 - 1000 * ( -b ) ) ;
			pt2.y = cvRound( y0 - 1000 * (  a ) ) ;
			cv::line( visualization, pt1, pt2, cv::Scalar( 0, 0, 255 ), 2, CV_AA ) ;
			
			// success
			return true ;
			
		// if no line found, leave values unchanged and return
		} else { return false ; }
		
	} // function "findLine()"
	
} // anonymous namespace

// filter line detection results and determine flight commands
void DroneAppFollowLine::processImageBottom( const cv_bridge::CvImage imageBottom ) {
	
	// detect and evaluate the line, assume height of 1.0m if drone is carried and therefore sensor is off
	cv::Mat visualization ;
	double offsetPix = 0.0, angleDeg = 0.0 ;
	const bool found = findLine( imageBottom.image, visualization,
	                             offsetPix, angleDeg,
	                             this->principalPointBottomU ) ;
	if( found ) {
		this->trkAngle( angleDeg * CV_PI / 180.0 ) ;
		this->trkOffset( sin( atan( offsetPix / this->focalLengthBottomU ) - ( this->trkRoll ) )
		               * ( this->navdata.altd != 0.0 ? (double)this->trkHeight : 1.0 ) ) ;
	}
	
	// show input and line visualization
	HAWAII_IMSHOW( imageBottom.image ) ;
	HAWAII_IMSHOW( visualization ) ;
	cv::waitKey( 1 ) ;
	
	// increase height if line is far away or if not found at all
	this->targetHeight = std::min( 1.5, found ?
	                                    1.0 + 0.10 * abs( this->trkOffset ) :
	                                    this->targetHeight + 0.0025 ) ;
	this->commands.controlLinZ( targetHeight - this->trkHeight ) ;
	
	// update other commands only with new line information, keep doing the same thing otherwise
	if( found ) {
		
		// sideways and yaw
		this->commands.controlLinY( 0.0                     - this->trkOffset ) ;
		this->commands.controlAngZ( -0.25 * this->trkOffset - this->trkAngle  ) ;
		
		// go forward only within rest of shared "movement capacity"
		geometry_msgs::Twist& movement = this->commands.movement ;
		double capacity = 1.0 ;
		capacity -= pow( ( movement.angular.z > 0.0 ?
		                   movement.angular.z / this->commands.controllerAngZ.uMax :
		                   movement.angular.z / this->commands.controllerAngZ.uMin ), 2 ) ;
		capacity -= pow( ( movement.linear.y  > 0.0 ?
		                   movement.linear.y  / this->commands.controllerLinY.uMax :
		                   movement.linear.y  / this->commands.controllerLinY.uMin ), 2 ) ;
		movement.linear.x = ( capacity > 0.0 ? sqrt( capacity ) : 0.0 ) * this->commands.controllerLinX.uMax ;
		
		// further reduce forward motion if flying away from line
		movement.linear.x *= std::max( 0.0, std::min( 1.0, 1.0 - this->trkOffset * this->trkAngle / 0.05 ) ) ;
	}
	
	// adjust forward speed via the keyboard's M/N keys or the gamepad's cross 
	if( this->keystates[ SDLK_m ]
	 || ( this->cross & SDL_HAT_UP ) ) {
		this->commands.controllerLinX.uMax = std::min(  1.00, this->commands.controllerLinX.uMax + 0.05 ) ;
		this->commands.controllerLinX.uMin = std::max( -1.00, this->commands.controllerLinX.uMin - 0.05 ) ;
		std::cout << "INFO: u{Max,Min} = "
		          << this->commands.controllerLinX.uMax << " / "
		          << this->commands.controllerLinX.uMin << std::endl ;
	}
	if( this->keystates[ SDLK_n ]
	 || ( this->cross & SDL_HAT_DOWN ) ) {
		this->commands.controllerLinX.uMax = std::max(  0.05, this->commands.controllerLinX.uMax - 0.05 ) ;
		this->commands.controllerLinX.uMin = std::min( -0.05, this->commands.controllerLinX.uMin + 0.05 ) ;
		std::cout << "INFO: u{Max,Min} = "
		          << this->commands.controllerLinX.uMax << " / "
		          << this->commands.controllerLinX.uMin << std::endl ;
	}
	
} // method "DroneAppFollowLine::processImageBottom()"
