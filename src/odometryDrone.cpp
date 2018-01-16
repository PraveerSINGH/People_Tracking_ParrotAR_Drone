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


// odometry using additional sensors of AR.Drone 2.0
// =================================================

#include "odometryDrone.h"

// helpers to convert rotation or translation vectors to a 
cv::Vec3d OdometryDrone::rotateRotation( const cv::Vec3d rotation,
                                         const double    yawZero  ) {
	cv::Vec3d ret ;
	ret( 0 ) = rotation( 0 )           ;
	ret( 1 ) = rotation( 1 ) - yawZero ; 
	ret( 2 ) = rotation( 2 )           ;
	return ret ;
}
cv::Vec3d OdometryDrone::rotateTranslation( const cv::Vec3d translation,
                                            const double    yawZero     ) {
	cv::Vec3d ret ;
	ret( 0 ) = translation( 0 ) * cos( yawZero )
				- translation( 2 ) * sin( yawZero ) ;
	ret( 1 ) = translation( 1 )                  ;
	ret( 2 ) = translation( 0 ) * sin( yawZero )
				+ translation( 2 ) * cos( yawZero ) ;
	return ret ;
}

// default c'tor
OdometryDrone::OdometryDrone() :
	tmPrev( NAN ),
	vxTracker( 0.1 ),
	vyTracker( 0.1 ),
	rotationGlobal(    cv::Vec3d::all( 0.0 ) ),
	translationGlobal( cv::Vec3d::all( 0.0 ) ) {
}

// integrate most recent "Navdata" message, buffer previous velocities to apply trapezoidal rule
void OdometryDrone::processNavdata( const ardrone_autonomy::Navdata navdata ) {
	
	// ignore current "navdata" if broken, i.e. past time stamp or unbelievable fast velocities (30 km/h = 8333 mm/s)
	if( ( !isnanf( this->tmPrev ) && navdata.tm < this->tmPrev )
	 || fabsf( navdata.vx ) > 8333.0f
	 || fabsf( navdata.vy ) > 8333.0f ) {
		return ;
	}
	
	// update velocity trackers
	this->vxTracker( navdata.vx ) ;
	this->vyTracker( navdata.vy ) ;
	
	// start trapezoidal integration with 2nd set of inputs
	if( !isnanf( this->tmPrev ) ) {
		// just convert directly available rotations and altitude
		// developer note: Both units and coordinate systems differ between the drone's sensors and default computer 
		//                 vision, including "libviso2":
		//                   | drone    visual odometry
		//                 --+-------------------------
		//                 x | forward  right
		//                 y | left     down
		//                 z | up       forward
		this->rotationGlobal(    0 ) = - navdata.rotY * CV_PI / 180.0 ; // pitch: ° to rad, opposite direction
		this->rotationGlobal(    1 ) = - navdata.rotZ * CV_PI / 180.0 ; // yaw:   ° to rad, opposite direction 
		this->rotationGlobal(    2 ) =   navdata.rotX * CV_PI / 180.0 ; // roll:  ° to rad, same direction
		this->translationGlobal( 1 ) = - navdata.altd * 0.001         ; // altitude: mm to m, opposite direction
		
		// average velocities (in drone's coordinates) and time delta for trapezoidal integration
		const double tmDelta = ( navdata.tm - this->tmPrev ) * 0.000001 ; // μs to s
		const double sxDelta = this->vxTracker * 0.001 * tmDelta,  // mm(/s) to m(/s)
		             syDelta = this->vyTracker * 0.001 * tmDelta ; // mm(/s) to m(/s)
		
		// integration in global "computer vision coordinates"
		const double sinYaw = sin( this->rotationGlobal( 1 ) ),
		             cosYaw = cos( this->rotationGlobal( 1 ) ) ; 
		this->translationGlobal( 0 ) += sxDelta * sinYaw - syDelta * cosYaw ;
		this->translationGlobal( 2 ) += sxDelta * cosYaw + syDelta * sinYaw ;
		
/*		printf( "Txyz = [ %6.2f, %6.2f, %6.2f ], Rxyz = [ %6.2f, %6.2f, %6.2f ]\n",
		        this->translationGlobal( 0 ),
		        this->translationGlobal( 1 ),
		        this->translationGlobal( 2 ),
		        this->rotationGlobal( 0 ),
		        this->rotationGlobal( 1 ),
		        this->rotationGlobal( 2 ) ) ; //*/
	}
	
	// set previous values for next iteration
	this->tmPrev = navdata.tm ;
}

// get current pose with optional reference yaw angle
cv::Vec3d OdometryDrone::getRotation(    const double yawZero ) const { return OdometryDrone::rotateRotation(    this->rotationGlobal,    yawZero ) ; }
cv::Vec3d OdometryDrone::getTranslation( const double yawZero ) const { return OdometryDrone::rotateTranslation( this->translationGlobal, yawZero ) ; }
