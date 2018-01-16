// Copyright (C) 2013 by:- Institut Eurécom
//                       - Télécom ParisTech
// 
//Author: Aakanksha Rana (rana.aakanksha@gmail.com) and Praveer Singh (praveersingh1990@gmail.com)
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

#pragma once


#include "appBase.h"
#include "appFollowPersonFP.h"
#include "sparse3D.h"
#include "dense3D.h"
#include "navigator.h"

// generic development drone application
class DroneAppDevelFP : public DroneAppBase {
	
	// c'tor, always use front camera, initialize added members
	public:
	DroneAppDevelFP(bool IsUsingDense3D = false); // ?????
	
	// local implementations of processing functions
	protected:
	virtual void processImageFront( const cv_bridge::CvImage imageFront ) override ;
	virtual void processKeystrokes(                                     ) override ;
	
	// sub-modules
	protected:
	bool mIsUsingDense3D;//mode switch between dense3d and sparse3d ?????
	appFollowPersonFP appfollowpersonfp ;
	//Sparse3D  sparse3D  ;
	Dense3D   dense3D   ;//????
	Navigator navigator ;//???
	
	// detect when no commands have been set for a given time, hover in place afterwards
	protected:
	hawaii::common::Timer timerCommandsLastSet ;
	double timeCommandsLastSetMax ;
} ;
