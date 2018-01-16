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


// navigate based on landmarks at corridor crossings
// =================================================

#include "navigator.h"
#include "mydemo/ImageProcessing/imageProcessing.h"

// c'tor with required properties of undistorted (!) camera images
Navigator::Navigator( const double focalLengthUArg,
                      const double focalLengthVArg,
                      const double principalPointUArg,
                      const double principalPointVArg ) :
	
	// camera properties
	focalLengthU( focalLengthUArg ),
	focalLengthV( focalLengthVArg ),
	principalPointU( principalPointUArg ),
	principalPointV( principalPointVArg ),
	
	// target symbol distance
	symbolDistForwardTarget( 1.5 ),
	
	// initial control state
	state( stateInactive ),
	
	// landmark detection results
	symbolType(  0 ),
	symbolValue( 0 ),
	symbolDistForward( NAN ),
	symbolDistUp(      NAN ),
	symbolAngleLeft(   NAN ),
	droneYawTarget( NAN ) {
	
	// initialize default mapping from landmark type and value to approaching distance and further flight angle
	for( size_t type  = 0 ; type  <  32 ; ++type  ) {
	for( size_t value = 0 ; value < 128 ; ++value ) {
		this->droneYawRelativeMap[ type ][ value ] = NAN ;
	} }
	
	// crossing at "Laboratoire SOC"
	this->droneYawRelativeMap[ 4 ][ 34 ] =   150.0 ; // coming from dead end, turn around
	this->droneYawRelativeMap[ 4 ][ 18 ] =    30.0 ; // coming from meeting room "Dirac", continue
	this->droneYawRelativeMap[ 4 ][  0 ] = -  90.0 ; // coming from the lab, turn left
	
	// crossing at dead end
	this->droneYawRelativeMap[ 8 ][ 30 ] =     0.0 ; // coming from dead end, continue
	this->droneYawRelativeMap[ 8 ][ 50 ] = - 180.0 ; // coming from meeting room "Dirac", turn around
	this->droneYawRelativeMap[ 1 ][  0 ] =    90.0 ; // coming from outside, turn right
}

// detect the landmark, buffer its type and relative position
void Navigator::processImageFront( const cv::Mat   image,
                                   const cv::Vec3d rotationGlobal ) {
	// serialize
	boost::lock_guard< boost::mutex > lock( this->mutex ) ;
	
	// landmark detection
	float symbolPosU, symbolPosV, symbolDiag ;
	detectLandmark( image,
	                this->symbolType, this->symbolValue,
	                symbolPosU, symbolPosV, symbolDiag ) ;
	
	// convert pixel coordinates to 3D, implicitly propagate "NAN" if no symbol detected
	const double horizonV = this->principalPointV + sin( rotationGlobal( 0 ) ) * this->focalLengthV ;
	this->symbolDistForward = 257.5 / symbolDiag ; // 257.5 for DIN-A3, 182.1 for DIN-A4
	this->symbolDistUp      = ( horizonV - symbolPosV ) / this->focalLengthV * this->symbolDistForward ;
	this->symbolAngleLeft   = atan2( this->principalPointU - symbolPosU, this->focalLengthU ) ;
	
	// switch between "inactive" and "approaching" if landmark is detected or lost
	if( this->state == stateInactive    && this->symbolType != 0 ) { this->state = stateApproaching ; }
	if( this->state == stateApproaching && this->symbolType == 0 ) { this->state = stateInactive    ; }
	
	// switch from "approaching" to "rotating"...
	if( this->state == stateApproaching ) {
		
		// ...if landmark has been recognized and approached...
		if( this->symbolType != 256
		 && 0.2 < fabs( this->symbolDistForward - this->symbolDistForwardTarget ) ) {
			
			// ...and a valid relative angle is specified
			const double droneYawRelative = this->droneYawRelativeMap[ this->symbolType ][ this->symbolValue ] ;
			if( !isnan( droneYawRelative ) ) {
				
				// find absolute target angle in [-180.0°,180.0°]
				const double droneYaw = - rotationGlobal( 1 ) * 180.0 / CV_PI ;
				this->droneYawTarget = droneYaw + droneYawRelative ;
				this->droneYawTarget -= 360.0 * ( this->droneYawTarget >   180.0 ) ;
				this->droneYawTarget += 360.0 * ( this->droneYawTarget < - 180.0 ) ;
				
				// change state
				this->state = stateRotating ;
			}
		}
	}
	
	// switch from "rotating" to "inactive"...
	if( this->state == stateRotating ) {
		
		// ...if target yaw angle has been reached
		const double droneYaw = - rotationGlobal( 1 ) * 180.0 / CV_PI ;
		if( fabs( droneYaw - this->droneYawTarget ) < 10.0 ) {
			
			// change state
			this->state = stateInactive ;
		}
	}
}

// get control commands depending on state
bool Navigator::getCommands(       DroneCommands& commands,
                             const cv::Vec3d      rotationGlobal ) {
	// serialize
	boost::lock_guard< boost::mutex > lock( this->mutex ) ;
	
	// handle all states independently
	switch( this->state ) {
			
		// fly towards landmark
		case stateApproaching : {
			
			// closed loop control of distance via pitch, height via rotor speed and angle via yaw rotation
			commands.controlLinX( this->symbolDistForward - this->symbolDistForwardTarget ) ;
			commands.controlLinZ( this->symbolDistUp      - 0.0                           ) ;
			commands.controlAngZ( this->symbolAngleLeft   - 0.0                           ) ;
			
			// print control in- and outputs
			printf( "errorForward = %5.2f, ctrl = %5.2f\n",   this->symbolDistForward - this->symbolDistForwardTarget, commands.movement.linear.x  ) ;
			printf( "errorUp      = %5.2f, ctrl = %5.2f\n",   this->symbolDistUp      - 0.0,                           commands.movement.linear.z  ) ;
			printf( "errorAngle   = %5.2f, ctrl = %5.2f\n\n", this->symbolAngleLeft   - 0.0,                           commands.movement.angular.z ) ;
			
			// do not move sideways or perform a specific flight maneuver
			commands.movement.linear.y = 0.0f ;
			commands.maneuver = DroneCommands::maneuverNone ;
			
		} break ;
			
		// turn towards symbol-specific further flight direction
		case stateRotating : {
			
			// map angle difference to the interval [-180°,180°]
			const double droneYaw = - rotationGlobal( 1 ) * 180.0 / CV_PI ;
			double errorYaw = this->droneYawTarget - droneYaw ;
			errorYaw -= 360.0 * ( errorYaw >   180.0 ) ;
			errorYaw += 360.0 * ( errorYaw < - 180.0 ) ;
			
			// closed-loop control of yaw rotation
			commands.controlAngZ( errorYaw / 180.0 * CV_PI ) ;
			printf( "errorYaw = %5.2f, ctrl = %5.2f\n\n", errorYaw, commands.movement.angular.z ) ;
			
			// do nothing else except rotating
			commands.movement.linear.x = 0.0f ;
			commands.movement.linear.y = 0.0f ;
			commands.movement.linear.z = 0.0f ;
			commands.maneuver = DroneCommands::maneuverNone ;
			
		} break ;
		
		// do not modify "commands"
		case stateInactive : {
			
		} break ;
		
	} // switch "all states"
	
	// return "true" only if "commands" has been modified
	return ( this->state != stateInactive ) ;
	
} // method "Navigator::getCommands()"
