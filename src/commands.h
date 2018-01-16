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


// commands to be sent to AR.Drone 2.0
// ===================================

#pragma once

#include "controller.h"
#include <geometry_msgs/Twist.h>

// struct to include regular flight movement, specific maneuvers and LED animations
struct DroneCommands {
	
	// default initialization, making the drone hover in place with on-board stabilization 
	DroneCommands() ;
	
	// reset commands, but keep parameters
	// user note #1: "hover()" keeps LED animations, "reset()" also disables those.
	// user note #2: This does not reset the included controllers (see below).
	void hover() ;
	void reset() ;
	
	// regular flight movement: Directly set relative commands [-1.0f,1.0f] for each axis. "movement.linear.{x,y,z}" 
	//                          are forward, left and upward velocity, "movement.angular.z" is counter-clockwise yaw 
	//                          rate, "movement.angular.{x,y}" are impossible to use due to the quadrotor's design.
	geometry_msgs::Twist movement ;
	
	// regular flight movement: If you control flight based on errors, i.e. target minus actual positions (in meters, 
	//                          forward/left/up positive) or angles (in radians, top-view counter-clockwise positive), 
	//                          feel free to use the provided controllers like "this->controlLinZ( 1.5 - actualHeight )".
	void controlLinX( const double error ) ;
	void controlLinY( const double error ) ;
	void controlLinZ( const double error ) ;
	void controlAngZ( const double error ) ;
	ControllerPID controllerLinX,
	              controllerLinY,
	              controllerLinZ,
	              controllerAngZ ;
	
	// regular flight movement: reset, disable or enable the integrator of all controllers in one call
	void controllersReset()             ;
	void controllersIntegratorDisable() ;
	void controllersIntegratorEnable()  ;
	
	// specific maneuvers
	enum {
		maneuverNone                = 255,
		maneuverTakeOff             = 253,
		maneuverLand                = 254,
		maneuverPhiMinus30          =   0,
		maneuverPhiPlus30           =   1,
		maneuverThetaMinus30        =   2,
		maneuverThetaPlus30         =   3,
		maneuverTheta20YawPlus200   =   4,
		maneuverTheta20YawMinus200  =   5,
		maneuverTurnAround          =   6,
		maneuverTurnAroundGoDown    =   7,
		maneuverYawShake            =   8,
		maneuverYawDance            =   9,
		maneuverPhiDance            =  10,
		maneuverThetaDance          =  11,
		maneuverVzDance             =  12,
		maneuverWave                =  13,
		maneuverPhiThetaMixed       =  14,
		maneuverDoublePhiThetaMixed =  15,
		maneuverFlipAhead           =  16,
		maneuverFlipBehind          =  17,
		maneuverFlipLeft            =  18,
		maneuverFlipRight           =  19,
	} maneuver ;
	
	// LED animations
	enum {
		ledNone              = 255,
		ledBlinkGreenRed     =   0,
		ledBlinkGreen        =   1,
		ledBlinkRed          =   2,
		ledBlinkOrange       =   3,
		ledSnakeGreenRed     =   4,
		ledFire              =   5,
		ledStandard          =   6,
		ledRed               =   7,
		ledGreen             =   8,
		ledRedSnake          =   9,
		ledBlank             =  10,
		ledLeftGreenRightRed =  11,
		ledLeftRedRightGreen =  12,
		ledBlinkStandard     =  13,
	} ledAnimation ;
	float         ledFrequency ; // Hertz
	unsigned char ledDuration  ; // seconds
	
} ; // struct "DroneCommands"
