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

#include "commands.h"

// default initialization, making the drone hover in place with on-board stabilization 
DroneCommands::DroneCommands() :
	
	// regular flight movement parameters
	// developer note #1: parameters initially tuned according to "en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method" 
	//                    using the no overshoot rule, i.e. kP = 0.2 * kU, kI = 2 * kP / TU, kD = kP * TU / 3
	// developer note #2: Note #1 was way too much for "Lin{X,Y}", therefore we're using 1/2 for each parameter.
	controllerLinX( 0.10, 0.05, 0.15, 0.5 ), // kU = 1.20, TU = 4.65
	controllerLinY( 0.10, 0.05, 0.15, 0.5 ), // kU = 1.20, TU = 4.65
	controllerLinZ( 0.60, 0.50, 0.50, 1.0 ), // kU = 2.95, TU = 2.50
	controllerAngZ( 1.00, 2.00, 0.30, 1.0 ), // kU = 5.40, TU = 0.95
	
	// LED animation parameters
	ledFrequency( 4.0 ),
	ledDuration(  5   ) {
	
	// all commands rather than parameters
	this->reset() ;
}

// reset commands, but keep parameters
void DroneCommands::hover() {
	this->movement.linear.x  = 0.0 ;
	this->movement.linear.y  = 0.0 ;
	this->movement.linear.z  = 0.0 ;
	this->movement.angular.x = 0.0 ;
	this->movement.angular.y = 0.0 ;
	this->movement.angular.z = 0.0 ;
	this->maneuver = maneuverNone ;
}
void DroneCommands::reset() {
	this->hover() ;
	this->ledAnimation = ledNone ;
}

// regular flight movement: abbreviated use of controllers
void DroneCommands::controlLinX( const double error ) { this->movement.linear.x  = this->controllerLinX( error ) ; }
void DroneCommands::controlLinY( const double error ) { this->movement.linear.y  = this->controllerLinY( error ) ; }
void DroneCommands::controlLinZ( const double error ) { this->movement.linear.z  = this->controllerLinZ( error ) ; }
void DroneCommands::controlAngZ( const double error ) { this->movement.angular.z = this->controllerAngZ( error ) ; }
void DroneCommands::controllersReset() {
	this->controllerLinX.reset() ;
	this->controllerLinY.reset() ;
	this->controllerLinZ.reset() ;
	this->controllerAngZ.reset() ;
}
void DroneCommands::controllersIntegratorDisable() {
	this->controllerLinX.disableIntegrator = true ;
	this->controllerLinY.disableIntegrator = true ;
	this->controllerLinZ.disableIntegrator = true ;
	this->controllerAngZ.disableIntegrator = true ;
}
void DroneCommands::controllersIntegratorEnable() {
	this->controllerLinX.disableIntegrator = false ;
	this->controllerLinY.disableIntegrator = false ;
	this->controllerLinZ.disableIntegrator = false ;
	this->controllerAngZ.disableIntegrator = false ;
}
