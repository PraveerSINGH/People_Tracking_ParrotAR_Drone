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

#include "sparse3D.h"
#include "odometryDrone.h"
#include "hawaii/GPU/autoMat.h"
#include "hawaii/common/helpers.h"
#include "hawaii/common/error.h"
#include "viso_mono.h"

// helpers only used here
namespace{
	
	// convert a matrix from "libviso2" format to fixed-size "OpenCV" format
	template< typename Type, int rows, int cols >
	void convertMatrix2Matx( const Matrix                        matrix,
	                               cv::Matx< Type, rows, cols >& matx   ) {
		// check dimensions
		HAWAII_ERROR_CONDITIONAL( matrix.m != matx.rows
		                       || matrix.n != matx.cols,
		                          "dimension mismatch"  ) ;
		
		// copy element by element
		for( int row = 0 ; row < rows ; ++row ) {
		for( int col = 0 ; col < cols ; ++col ) {
			matx( row, col ) = matrix.val[ row ][ col ] ;
		} }
	}
	
	// convert a distance to a color from red over yellow and green to cyan
	class DistanceToColor {
		
		// c'tor using distance limits
		public:
		DistanceToColor( const double distRedArg,
		                 const double distCyanArg ) :
			distRed(  distRedArg  ),
			distCyan( distCyanArg ) {
		}
		protected:
		double distRed,
		       distCyan ;
		
		// convert a distance to a color
		public:
		cv::Vec3b operator ()( double dist ) {
			double distRel = 3.0 * (       dist     - this->distRed )
			                     / ( this->distCyan - this->distRed ) ;
			     if( distRel < 0.0 ) { return cv::Vec3b(                       0,             0,                     255 ) ; } // all red
			else if( distRel < 1.0 ) { return cv::Vec3b(                       0, distRel * 255,                     255 ) ; } // from red to yellow
			else if( distRel < 2.0 ) { return cv::Vec3b(                       0,           255, ( 2.0 - distRel ) * 255 ) ; } // from yellow to green
			else if( distRel < 3.0 ) { return cv::Vec3b( ( distRel - 2.0 ) * 255,           255,                       0 ) ; } // from green to cyan
			else                     { return cv::Vec3b(                     255,           255,                       0 ) ; } // all cyan
		}
	} ;
	
} // anonymous namespace

// c'tor with properties of undistorted camera images
Sparse3D::Sparse3D( const double focalLength,
                    const double principalPointU,
                    const double principalPointV ) :
	
	// control commands: "corkscrew" flight parameters
	corkscrewAmplitudeY( 0.10 ), // amplitude of the horizontal oscillation in [0.0 1.0]
	corkscrewAmplitudeZ( 0.30 ), // amplitude of the vertical oscillation in meters
	corkscrewFrequency( 0.33 ), // frequency of both oscillations in hertz
	corkscrewPhase( 45.0 ), // phase between both oscillations in degrees
	corkscrewTimer( true ), // start timer immediately
	
	// size of input images, additional scale correction based on on-board odometry
	sizeImage( 0, 0 ),
	scaleFactor( 1.0 ),
	
	// initial pose
	pose( cv::Matx44d::eye() ),
	
	// autonomous flight set points
	trackerAngleDelta(   0.5 ),
	trackerForwardSpeed( 0.5 ),
	
	// visual odometry results
	visoSuccessPrev( false ),
	visoFailsConsecMax( 5 ),
	visoFailsConsec( visoFailsConsecMax + 1 ),
	
	// pose of previous image successfully used by visual odometry
	rotationGlobalOnboardPrev(    cv::Vec3d::all( 0.0 ) ),
	translationGlobalOnboardPrev( cv::Vec3d::all( 0.0 ) ) {
	
	// instantiate wrapped visual odometry with required parameters
	VisualOdometryMono::parameters params ;
	params.inlier_threshold =   0.00010 ; //   0.00001
	params.motion_threshold = 150.0     ; // 100.0 TODO test
	params.match.match_binsize   =  32 ; //  50 TODO test
	params.match.match_radius    = 200 ; // 200
	params.match.half_resolution =   1 ; //   1
	params.match.refinement      =   2 ; //   1
	params.bucket.max_features  =  2 ; //  2 TODO test
	params.bucket.bucket_width  = 32 ; // 50 TODO test
	params.bucket.bucket_height = 32 ; // 50 TODO test
	params.calib.f = focalLength ;
	params.calib.cu = principalPointU ;
	params.calib.cv = principalPointV ;
	params.bucket.max_features = 4 ;
   params.bucket.bucket_width  = 64 ;
   params.bucket.bucket_height = 64 ;
	this->visoPtr.reset( new VisualOdometryMono( params ) ) ;
}

// process front image
bool Sparse3D::processImageFront( const cv::Mat   image,
                                  const cv::Vec3d rotationGlobal,
                                  const cv::Vec3d translationGlobal ) {
	// check input
	HAWAII_ERROR_CONDITIONAL( image.empty(),
	                          "Image must not be empty." ) ;
	HAWAII_ERROR_CONDITIONAL( image.type() != CV_8UC1
	                       && image.type() != CV_8UC3,
	                          "Image type must be \"CV_8UC1\" or \"CV_8UC3\"." ) ;
	// set image size
	this->sizeImage = image.size() ;
	
	// convert to grayscale if necessary
	cv::Mat imageGray ;
	if( image.type() == CV_8UC1 ) { imageGray = image ; }
	else { cv::cvtColor( image, imageGray, cv::COLOR_BGR2GRAY ) ; }
	
	// update ground plane parameters used to resolve the monocular scale ambiguity
	this->visoPtr->param.height = 0.04 - translationGlobal( 1 ) ; // sign different from default "computer vision coordinates", camera higher than sensor
	this->visoPtr->param.pitch = rotationGlobal( 0 ) ;
	this->visoPtr->param.roll  = rotationGlobal( 2 ) ;
	
	// try to perform visual odometry, remember if succeeded or failed (e.g. because of too little motion)
	// developer note: The visual odometry implementation keeps two images - a current and a previous one. If the 
	//                 "replace" argument is true, the current image is replaced and the previous image is kept. 
	//                 Otherwise, the old current becomes the new previous image. We keep the previous image until 
	//                 there has been enough motion to perform visual odometry, but give up if it fails consecutively 
	//                 for a fixed number of frames.
	int32_t dims[] = { imageGray.cols,
	                   imageGray.rows,
	                   imageGray.step } ;
	const bool replace = !this->visoSuccessPrev && this->visoFailsConsec < this->visoFailsConsecMax ;
	this->visoSuccessPrev = this->visoPtr->process( imageGray.data, dims, replace ) ;
	     if( this->visoSuccessPrev ) { this->visoFailsConsec  = 0 ; }
	else if( !replace              ) { this->visoFailsConsec  = 1 ; }
	else                             { this->visoFailsConsec += 1 ; }
	
	// print debug information
/*	if( this->visoSuccessPrev ) { hawaii::cout << "INFO: visual odometry succeeded with "
	                                           << this->visoPtr->getNumberOfInliers() << " inliers out of "
	                                           << this->visoPtr->getNumberOfMatches() << " total matches"
	                                           << hawaii::endl ; }
	else                        { hawaii::cout << "INFO: visual odometry failed"
	                                           << hawaii::endl ; } //*/
	// only on success...
	if( this->visoSuccessPrev ) {
		
		// get motion purely based on visual odometry
		cv::Matx44d motionDeltaViso ;
		convertMatrix2Matx( this->visoPtr->getMotion(), motionDeltaViso ) ;
		
		// TODO only scale to same height change if on-board height change is big enough?
		// use on-board odometry to correct the scale: While on-board odometry is more susceptible to noise, visual 
		//                                             odometry may pick a wrong set of 3D points when fitting the 
		//                                             ground plane. Therefore, we fully trust visual odometry if its 
		//                                             translation is similar enough to on-board odometry. Otherwise we 
		//                                             scale visual odometry results to make translation lengths or 
		//                                             altitude components match.
		// developer note: Almost the same code exists in "Dense3D::visoProcess()" - keep them synchronized!
		const cv::Vec3d translDeltaGlobalOnboard = translationGlobal - this->translationGlobalOnboardPrev ;
		const double lengthOnboard = sqrt( translDeltaGlobalOnboard( 0 ) * translDeltaGlobalOnboard( 0 )
		                                 + translDeltaGlobalOnboard( 1 ) * translDeltaGlobalOnboard( 1 )
		                                 + translDeltaGlobalOnboard( 2 ) * translDeltaGlobalOnboard( 2 ) ),
		             lengthViso    = sqrt( motionDeltaViso( 0, 3 ) * motionDeltaViso( 0, 3 )
		                                 + motionDeltaViso( 1, 3 ) * motionDeltaViso( 1, 3 )
		                                 + motionDeltaViso( 2, 3 ) * motionDeltaViso( 2, 3 ) ) ;
		this->scaleFactor = lengthOnboard / lengthViso ;
		if( this->scaleFactor > 0.9
		 && this->scaleFactor < 1.1 ) {
			this->scaleFactor = 1.0 ;
		}
		
		// apply scale correction to translation, accumulate motion
		motionDeltaViso( 0, 3 ) *= this->scaleFactor ;
		motionDeltaViso( 1, 3 ) *= this->scaleFactor ;
		motionDeltaViso( 2, 3 ) *= this->scaleFactor ;
		this->pose = this->pose * motionDeltaViso.inv() ;
		
		// print motion in local coordinates
		// developer note: "translDeltaGlobalOnboard" is the transformation from the previous to the current coordinate 
		//                 system, while "motionDeltaViso" maps a 3D point from previous to current. Therefore, its signs 
		//                 need to be inverted below.
/*		const cv::Vec3d translDeltaLocalOnboard = OdometryDrone::rotateTranslation( translDeltaGlobalOnboard, rotationGlobal( 1 ) ) ;
		printf( "%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f\n"
		        ,   lengthOnboard
		        ,   lengthViso
		        ,   translDeltaLocalOnboard( 0 )
		        , - motionDeltaViso( 0, 3 )
		        ,   translDeltaLocalOnboard( 1 )
		        , - motionDeltaViso( 1, 3 )
		        ,   translDeltaLocalOnboard( 2 )
		        , - motionDeltaViso( 2, 3 )
		      ) ; //*/
	}
	
	// set new previous values
	if( !replace ) {
		this->rotationGlobalOnboardPrev    = rotationGlobal    ;
		this->translationGlobalOnboardPrev = translationGlobal ;
	}
	
	// directly return outcome to shorten calling code 
	return this->visoSuccessPrev ;
	
} // method "Sparse3D::process()"

// get control commands
bool Sparse3D::getCommands(       DroneCommands& commands,
                            const cv::Vec3d      translationGlobal ) const {
	
	// autonomous flight control
	// user note: "Corkscrew" flight and will override vertical and sideways flight control, so don't even try to set it.
	#if 0
		commands.movement.linear.x = this->trackerForwardSpeed ;
	#else
		commands.movement.linear.x = 0.0 ;
	#endif
	#if 1
		commands.controllerAngZ.reset() ; // TODO disables integral control on purpose, but also derivative control as collateral damage
		commands.controlAngZ( this->trackerAngleDelta ) ;
		printf( "%8.4f %8.4f\n", (double)this->trackerAngleDelta, commands.movement.angular.z ) ;
	#else
		commands.movement.angular.z = 0.0 ;
	#endif
	
	// "corkscrew" flight: Only forward motion and yaw rotation are based on 3D points (see above). Vertical and 
	//                     sideways motion are phase-shifted oscillations to ensure sufficient motion for sparse 3D 
	//                     reconstruction. Combining all 4 DOFs results in a "corkscrew"-shaped flight trajectory.
	// developer note: The drone's ultrasonic height sensor enables closed-loop height control, but sideways motion can 
	//                 only use open-loop control.
	const double angle = 2.0 * CV_PI * this->corkscrewFrequency * this->corkscrewTimer.toc() / 1000.0 ;
	commands.movement.linear.y = this->corkscrewAmplitudeY * (float)sin( angle ) ;
	const float altitude = - translationGlobal( 1 ) ;
	commands.controlLinZ( 1.0f - altitude
	                    + this->corkscrewAmplitudeZ * (float)sin( angle - this->corkscrewPhase / 180.0 * CV_PI ) ) ;
	
	// always return "true", because the oscillations are always set
	return true ;
}
	
// intermediately-computed 3D points for each matched feature
void Sparse3D::getPoints3D( cv::Mat&             points3D,
                            std::vector< bool >& inliers  ) const {
	
	// return empty results if last odometry failed
	if( !this->visoSuccessPrev ) {
		points3D.release() ;
		inliers.clear() ;
	} else {
		
		// convert from "libviso2" to "OpenCV" format
		const size_t numPoints = this->visoPtr->points3D.n ;
		points3D.create( 3, numPoints, CV_64FC1 ) ;
		for( int coord = 0 ; coord < 3 ; ++coord ) {
			memcpy( points3D.ptr( coord ),
			        this->visoPtr->points3D.val[ coord ],
			        numPoints * sizeof( double )         ) ;
		}
		
		// apply additional scale correction based on on-board odometry
		points3D *= this->scaleFactor ;
		
		// convert inlier indices to flags
		inliers.assign( numPoints, false ) ;
		const std::vector< int > inlierIndices = this->visoPtr->getInlierIndices() ;
		for( const auto& inlierIndex : inlierIndices ) {
			inliers[ inlierIndex ] = true ;
		}
		
	} // if "last visual odometry successful?"
	
} // method "Sparse3D::getPoints3D()"

// relative motion between current and previous successful image
void Sparse3D::getMotion( cv::Matx33d& rotationRelative,
                          cv::Vec3d&   translationRelative ) const {
	
	// return not-a-numbers if last odometry failed
	if( !this->visoSuccessPrev ) {
		rotationRelative    = cv::Matx33d::all( NAN ) ;
		translationRelative = cv::Vec3d::all(   NAN ) ;
	} else {
		cv::Matx44d motion ;
		convertMatrix2Matx( this->visoPtr->getMotion(), motion ) ;
		rotationRelative         = motion.get_minor< 3, 3 >( 0, 0 ) ;
		translationRelative( 0 ) = motion( 0, 3 ) * this->scaleFactor ;
		translationRelative( 1 ) = motion( 1, 3 ) * this->scaleFactor ;
		translationRelative( 2 ) = motion( 2, 3 ) * this->scaleFactor ;
	}
}

// accumulated pose since last call to "resetPose()"
void Sparse3D::getPose( cv::Matx33d& rotationAbsolute,
                        cv::Vec3d&   translationAbsolute ) const {
	
	// return not-a-numbers if last odometry failed
	if( !this->visoSuccessPrev ) {
		rotationAbsolute    = cv::Matx33d::all( NAN ) ;
		translationAbsolute = cv::Vec3d::all(   NAN ) ;
	} else {
		rotationAbsolute         = this->pose.get_minor< 3, 3 >( 0, 0 ) ;
		translationAbsolute( 0 ) = this->pose( 0, 3 ) ;
		translationAbsolute( 1 ) = this->pose( 1, 3 ) ;
		translationAbsolute( 2 ) = this->pose( 2, 3 ) ;
	}
}

// reset accumulated pose
void Sparse3D::resetPose() { this->pose = cv::Matx44d::eye() ; }

// draw matches and projected 3D points onto image
void Sparse3D::visualize( cv::Mat& visualization ) const {
	
	// get most recent matches and inlier flags, no inliers if last odometry failed
	const std::vector< Matcher::p_match > matches2D = this->visoPtr->getMatches() ;
	std::vector< bool > matches2DInliers( matches2D.size(), false ) ;
	if( this->visoSuccessPrev ) {
		const std::vector< int > inlierIndices = this->visoPtr->getInlierIndices() ;
		for( const auto& inlierIndex : inlierIndices ) {
			matches2DInliers[ inlierIndex ] = true ;
		}
	}
	
	// draw matches: inliers in blue, outliers in purple
	for( size_t index = 0 ; index < matches2D.size() ; ++index ) {
		const auto& match = matches2D[ index ] ;
		cv::line( visualization,
		          cv::Point( match.u1c, match.v1c ),
		          cv::Point( match.u1p, match.v1p ),
		          cv::Scalar( 255, 0, matches2DInliers[ index ] ? 0 : 255 ), 1, 4 ) ;
	}
	
	// get most recent 3D points and inlier flags, both empty if last odometry failed
	cv::Mat points3D ;
	std::vector< bool > points3DInliers ;
	this->getPoints3D( points3D, points3DInliers ) ;
	if( !points3D.empty() ) {
		
		// fast access to coordinates
		const auto* __restrict const x = points3D.ptr< double >( 0 ) ;
		const auto* __restrict const y = points3D.ptr< double >( 1 ) ;
		const auto* __restrict const z = points3D.ptr< double >( 2 ) ;
		
		// get the 10% and 90% distances to initialize the distance-to-color converter
		std::vector< double > distances ;
		for( size_t index = 0 ; index < (size_t)points3D.cols ; ++index ) {
			if( points3DInliers[ index ] ) {
				distances.push_back( z[ index ] ) ;
			}
		}
		std::sort( distances.begin(), distances.end() ) ;
		DistanceToColor distanceToColor( distances[ 0.10 * distances.size() ],
		                                 distances[ 0.90 * distances.size() ] ) ;
		// go through all 3D points
		constexpr size_t gridRows = 3,
							  gridCols = 5 ;
		std::vector< double > distanceBins[ gridRows ][ gridCols ] ;
		for( size_t index = 0 ; index < (size_t)points3D.cols ; ++index ) {
			
			// only process points in front of camera
			if( z[ index ] > 0.0 ) {
				
				// project point onto image plane
				const int u = x[ index ] / z[ index ] * this->visoPtr->param.calib.f + this->visoPtr->param.calib.cu,
				          v = y[ index ] / z[ index ] * this->visoPtr->param.calib.f + this->visoPtr->param.calib.cv ;
				
				// only process points within the image
				if( u >= 0 && u < this->sizeImage.width
				 && v >= 0 && v < this->sizeImage.height ) {
					std::cout<<x[index]<<" "
							<<y[index]<<" "
							<<z[index]<<" "
							<<std::endl;
					
					// draw point in a distance-dependent color as a full (inliers) oder hollow (outliers) blob
					const cv::Vec3b color = distanceToColor( z[ index ] ) ;
					                                 visualization.at< cv::Vec3b >( v - 2, u - 1 ) = color ;
					                                 visualization.at< cv::Vec3b >( v - 2, u     ) = color ;
					                                 visualization.at< cv::Vec3b >( v - 2, u + 1 ) = color ;
					                                 visualization.at< cv::Vec3b >( v - 1, u - 2 ) = color ;
					                                 visualization.at< cv::Vec3b >( v - 1, u - 1 ) = color ;
					if( points3DInliers[ index ] ) { visualization.at< cv::Vec3b >( v - 1, u     ) = color ; }
					                                 visualization.at< cv::Vec3b >( v - 1, u + 1 ) = color ;
					                                 visualization.at< cv::Vec3b >( v - 1, u + 2 ) = color ;
					                                 visualization.at< cv::Vec3b >( v,     u - 2 ) = color ;
					if( points3DInliers[ index ] ) { visualization.at< cv::Vec3b >( v,     u - 1 ) = color ; }
					if( points3DInliers[ index ] ) { visualization.at< cv::Vec3b >( v,     u     ) = color ; }
					if( points3DInliers[ index ] ) { visualization.at< cv::Vec3b >( v,     u + 1 ) = color ; }
					                                 visualization.at< cv::Vec3b >( v,     u + 2 ) = color ;
					                                 visualization.at< cv::Vec3b >( v + 1, u - 2 ) = color ;
					                                 visualization.at< cv::Vec3b >( v + 1, u - 1 ) = color ;
					if( points3DInliers[ index ] ) { visualization.at< cv::Vec3b >( v + 1, u     ) = color ; }
					                                 visualization.at< cv::Vec3b >( v + 1, u + 1 ) = color ;
					                                 visualization.at< cv::Vec3b >( v + 1, u + 2 ) = color ;
					                                 visualization.at< cv::Vec3b >( v + 2, u - 1 ) = color ;
					                                 visualization.at< cv::Vec3b >( v + 2, u     ) = color ;
					                                 visualization.at< cv::Vec3b >( v + 2, u + 1 ) = color ;
					
					// sort point's z-distances into a grid on the image, add inlier twice to increase its influence
					const size_t indexRow = (size_t)( (double)gridRows * (double)v / (double)this->sizeImage.height ),
									 indexCol = (size_t)( (double)gridCols * (double)u / (double)this->sizeImage.width  ) ;
					                                 distanceBins[ indexRow ][ indexCol ].push_back( z[ index ] ) ;
					if( points3DInliers[ index ] ) { distanceBins[ indexRow ][ indexCol ].push_back( z[ index ] ) ; }
					
				} // if "within image?"
				
			} // if "in front of camera?"
			
		} // for "all 3D points"
		
		// select the medians of the binned distances, also keep their sum, minimum and maximum
		cv::Matx< double, gridRows, gridCols > medians ;
		double medianSum = 0.0,
		       medianMin =   INFINITY,
		       medianMax = - INFINITY ;
		for( size_t indexRow = 0 ; indexRow < gridRows ; ++indexRow ) {
		for( size_t indexCol = 0 ; indexCol < gridCols ; ++indexCol ) {
			
			// corresponding bin and its squared median
			std::vector< double >& distanceBin = distanceBins[ indexRow ][ indexCol ] ;
			std::sort( distanceBin.begin(), distanceBin.end() ) ;
			double medianCurr = ( distanceBin.empty() ? NAN : distanceBin[ distanceBin.size() / 2 ] ) ;
			medianCurr *= medianCurr ;
			medians( indexRow, indexCol ) = medianCurr ;
			
			// other statistics
			if( !isnan( medianCurr ) ) {
				medianSum += medianCurr ;
				if( medianMin > medianCurr ) { medianMin = medianCurr ; }
				if( medianMax < medianCurr ) { medianMax = medianCurr ; }
			}
			
		} } // for "all bins"
		
//		HAWAII_PRINT( medians ) ;
		
		// use the medians as weights for finding a target direction in the image
		double uTarget = 0.0,
				 vTarget = 0.0 ;
		for( size_t indexRow = 0 ; indexRow < medians.rows ; ++indexRow ) {
		for( size_t indexCol = 0 ; indexCol < medians.cols ; ++indexCol ) {
			
			const double medianCurr = medians( indexRow, indexCol ) ;
			if( !isnan( medianCurr ) ) {
				uTarget += indexCol * medianCurr ;
				vTarget += indexRow * medianCurr ;
			}
		} } // for "all bins"
		uTarget /= medianSum ; uTarget += 0.5 ; uTarget *= (double)this->sizeImage.width  / (double)gridCols ;
		vTarget /= medianSum ; vTarget += 0.5 ; vTarget *= (double)this->sizeImage.height / (double)gridRows ;
		
		// convert its horizontal position to an angle
		const double angleDelta = atan2( this->visoPtr->param.calib.cu - uTarget, this->visoPtr->param.calib.f ) ;
		this->trackerAngleDelta( isnan( angleDelta ) ? 0.0 : angleDelta ) ;
		
//		HAWAII_PRINT( 180.0 / CV_PI *              angleDelta ) ;
//		HAWAII_PRINT( 180.0 / CV_PI * this->trackerAngleDelta ) ;
		
		// constantly fly forward
		this->trackerForwardSpeed( 0.1 ) ;
		
		// draw a circle at the target direction in the image, with its color according to the target speed
		cv::circle( visualization,
		            cv::Point( uTarget, vTarget ),
		            20,
		            cv::Scalar(   0,
		                        255 * ( 1.0 - this->trackerForwardSpeed ),
		                        255 * ( 0.0 + this->trackerForwardSpeed ) ),
		            4                                                       ) ;
		
	} // if "3D points available (i.e. last visual odometry successful)?"
	
	// otherwise, fade down the control inputs
	else {
		this->trackerAngleDelta(   0.0 ) ;
		this->trackerForwardSpeed( 0.0 ) ;
	}
	
} // method "Sparse3D::visualize()"
