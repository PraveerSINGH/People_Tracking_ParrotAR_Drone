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


// off-line dense 3D
// =================

#include "hawaii/common/helpers.h"
#include "hawaii/common/error.h"
#include "matcher.h"
#include "viso_mono.h"
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// helpers to be moved to "dense3D.cpp"
namespace {
	
	// align images captured while hovering in place, use differences as cues for moving objects
	void detectMotion( const cv::Mat  imageA,
	                   const cv::Mat  imageB,
	                         cv::Mat& motionMaskA ) {
		// check input
		HAWAII_ERROR_CONDITIONAL( imageA.empty()
		                       || imageB.empty(),
		                          "Input images must not be empty." ) ;
		HAWAII_ERROR_CONDITIONAL( imageA.type() != CV_8UC1
		                       || imageB.type() != CV_8UC1,
		                          "Input image type must be \"CV_8UC1\"." ) ;
		HAWAII_ERROR_CONDITIONAL( imageA.size() != imageB.size(),
		                          "input image sizes must be equal." ) ;
		
		// find "libviso2" matches
		Matcher::parameters params ;
		params.match_binsize =  64 ;
		params.match_radius  = 100 ;
		params.refinement    =   2 ;
		Matcher matcher( params ) ;
		int32_t dimsA[] = { imageA.cols, imageA.rows, imageA.step } ;
		int32_t dimsB[] = { imageB.cols, imageB.rows, imageB.step } ;
		matcher.pushBack( imageA.data, dimsA, false ) ;
		matcher.pushBack( imageB.data, dimsB, false ) ;
		matcher.matchFeatures( 0 ) ;
		std::vector< Matcher::p_match > matches = matcher.getMatches() ;
		
		// visualize the matches
/*		cv::Mat visuAlign ; cv::cvtColor( imageA, visuAlign, CV_GRAY2BGR ) ;
		for( const auto& match : matches ) {
			cv::line( visuAlign,
			          cv::Point2f( match.u1p, match.v1p ),
			          cv::Point2f( match.u1c, match.v1c ),
			          cv::Scalar( 0, 127, 255 ), 2        ) ;
		}
		HAWAII_IMSHOW( visuAlign ) ; //*/
		
		// convert the matches to "OpenCV" points
		std::vector< cv::Point2f > pointsA,
		                           pointsB ;
		for( const auto& match : matches ) {
			pointsA.push_back( cv::Point2f( match.u1p, match.v1p ) ) ;
			pointsB.push_back( cv::Point2f( match.u1c, match.v1c ) ) ;
		}
		
		// find and apply homography via "OpenCV"
		cv::Mat homography = cv::findHomography( pointsA,
		                                         pointsB,
															  cv::RANSAC, 3.0 ) ;
		cv::Mat stabilizedB ;
		cv::warpPerspective( imageB, stabilizedB,
		                     homography,
		                     imageB.size(),
		                     cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
		                     cv::BORDER_REPLICATE                    ) ;
		
		// simply use the absolute difference as a mask for moving objects
		cv::absdiff( imageA, stabilizedB, motionMaskA ) ;
		
	} // function "detectMotion()"
	
	// rectify images captured before and after the height change
	void rectify( const cv::Matx33d camera,
	              const cv::Vec3d   rotationDrone,
	              const cv::Vec3d   translationDrone,
	              const cv::Mat     imageBefore,
	              const cv::Mat     imageAfter,
	                    cv::Mat&    rectifiedBefore,
	                    cv::Mat&    rectifiedAfter   ) {
		
		// instantiate visual odometry with camera and ground plane parameters
		VisualOdometryMono::parameters params ;
		params.calib.f = 0.5 * ( camera( 0, 0 )
		                       + camera( 1, 1 ) ) ;
		params.calib.cu = camera( 0, 2 ) ;
		params.calib.cv = camera( 1, 2 ) ;
		params.height = - translationDrone( 1 ) ; // sign different from default "computer vision coordinates"
      params.pitch = rotationDrone( 0 ) ;
      params.roll  = rotationDrone( 2 ) ;
		VisualOdometryMono viso( params ) ;
		
		// estimate relative position between the images
		int32_t dimsBefore[] = { imageBefore.cols, imageBefore.rows, imageBefore.step } ;
		int32_t dimsAfter[]  = { imageAfter.cols,  imageAfter.rows,  imageAfter.step  } ;
		HAWAII_PRINT( viso.process( imageBefore.data, dimsBefore, false ) ) ;
		HAWAII_PRINT( viso.process( imageAfter.data,  dimsAfter,  false ) ) ;
		const Matrix motionViso = viso.getMotion() ;
		
		// visualize the matches
		cv::Mat visuRectify ; cv::cvtColor( imageAfter, visuRectify, CV_GRAY2BGR ) ;
		for( const auto& match : viso.getMatches() ) {
			cv::line( visuRectify,
			          cv::Point2f( match.u1p, match.v1p ),
			          cv::Point2f( match.u1c, match.v1c ),
			          cv::Scalar( 0, 127, 255 ), 2        ) ;
		}
		HAWAII_IMSHOW( visuRectify ) ;
		
		// convert to "OpenCV" format
		cv::Matx33d rotationViso    ;
		cv::Vec3d   translationViso ;
		rotationViso(0,0)  = motionViso.val[0][0] ;
		rotationViso(0,1)  = motionViso.val[0][1] ;
		rotationViso(0,2)  = motionViso.val[0][2] ;
		rotationViso(1,0)  = motionViso.val[1][0] ;
		rotationViso(1,1)  = motionViso.val[1][1] ;
		rotationViso(1,2)  = motionViso.val[1][2] ;
		rotationViso(2,0)  = motionViso.val[2][0] ;
		rotationViso(2,1)  = motionViso.val[2][1] ;
		rotationViso(2,2)  = motionViso.val[2][2] ;
		translationViso(0) = motionViso.val[0][3] ;
		translationViso(1) = motionViso.val[1][3] ;
		translationViso(2) = motionViso.val[2][3] ;
		HAWAII_PRINT( rotationViso    ) ;
		HAWAII_PRINT( translationViso ) ;
		
		// rectify pair of frames
		static const cv::Vec< double, 5 > distortion( 0.0, 0.0, 0.0, 0.0, 0.0 ) ;
		cv::Mat homographyAfter,
				  homographyBefore ;
		cv::Mat projAfter, projBefore, reproj ;
		cv::stereoRectify( camera, distortion, camera, distortion, imageAfter.size(),
		                   rotationViso, translationViso,
		                   homographyAfter, homographyBefore,
		                   projAfter, projBefore, reproj ) ;
		HAWAII_PRINT( homographyBefore ) ;
		HAWAII_PRINT( homographyAfter  ) ;
		
		homographyAfter  = projAfter.colRange(  0, 3 ) * homographyAfter  * (cv::Mat)camera.inv() ;
		homographyBefore = projBefore.colRange( 0, 3 ) * homographyBefore * (cv::Mat)camera.inv() ;
		cv::warpPerspective( imageBefore,  rectifiedAfter,  homographyAfter,  imageAfter.size(),  cv::INTER_LINEAR, cv::BORDER_REPLICATE, 0 ) ;
		cv::warpPerspective( imageAfter, rectifiedBefore, homographyBefore, imageBefore.size(), cv::INTER_LINEAR, cv::BORDER_REPLICATE, 0 ) ;
		HAWAII_IMSHOW( rectifiedBefore ) ;
		HAWAII_IMSHOW( rectifiedAfter  ) ;
		
		// "traditional" way, but why create maps for one-time use?
/*		cv::Mat rectifiedBeforeMap,
				  rectifiedAfterMap ;
		cv::Mat mapBeforeX, mapBeforeY, mapAfterX, mapAfterY ;
		cv::initUndistortRectifyMap( camera, distortion, homographyBefore, projBefore, imageBefore.size(), CV_32FC1, mapBeforeX, mapBeforeY ) ;
		cv::initUndistortRectifyMap( camera, distortion, homographyAfter,  projAfter,  imageAfter.size(),  CV_32FC1, mapAfterX,  mapAfterY  ) ;
		cv::remap( imageBefore, rectifiedBeforeMap, mapBeforeX, mapBeforeY, cv::INTER_LINEAR ) ;
		cv::remap( imageAfter,  rectifiedAfterMap,  mapAfterX,  mapAfterY,  cv::INTER_LINEAR ) ;
		HAWAII_IMSHOW( rectifiedBeforeMap ) ;
		HAWAII_IMSHOW( rectifiedAfterMap  ) ; //*/
	}
	
} // anonymous namespace

// entry point
void dense3DOffline(std::string filename = "data/dense3D_0.15m_0.yml") {
	
	// inputs
	cv::Matx< double, 3, 3 > camera ;
	std::vector< cv::Mat   > imagesBefore(       2 ), imagesAfter(       2 ) ;
	std::vector< cv::Vec3d > rotationsBefore(    2 ), rotationsAfter(    2 ) ;
	std::vector< cv::Vec3d > translationsBefore( 2 ), translationsAfter( 2 ) ;
	
	// load data
	cv::FileStorage fs( filename.c_str(),               cv::FileStorage::READ ) ;
//	cv::FileStorage fs( "data/dense3D/corr1_0.15m_0.25s.yml",             cv::FileStorage::READ ) ; // too much sideways
//	cv::FileStorage fs( "data/dense3D/corr2_0.15m_0.25s.yml",             cv::FileStorage::READ ) ;
	//cv::FileStorage fs( "data/dense3D/lab1_0.1m_0.25s.yml",               cv::FileStorage::READ ) ;
//	cv::FileStorage fs( "data/dense3D/lab2_0.1m_0.25s.yml",               cv::FileStorage::READ ) ; // way too much forward
//	cv::FileStorage fs( "data/dense3D/outside1_0.15m_0.25s.yml",          cv::FileStorage::READ ) ; // wrong matches on right
//	cv::FileStorage fs( "data/dense3D/outside2_0.15m_0.25s.yml",          cv::FileStorage::READ ) ;
//	cv::FileStorage fs( "data/dense3D/outsidePersonWind_0.15m_0.25s.yml", cv::FileStorage::READ ) ;
	cv::Mat dummy ;
	fs[ "camera"              ] >> dummy ; camera                  = dummy ;
	fs[ "imagesBefore0"       ] >>         imagesBefore[       0 ]         ;
	fs[ "imagesBefore1"       ] >>         imagesBefore[       1 ]         ;
	fs[ "imagesAfter0"        ] >>         imagesAfter[        0 ]         ;
	fs[ "imagesAfter1"        ] >>         imagesAfter[        1 ]         ;
	fs[ "rotationsBefore0"    ] >> dummy ; rotationsBefore[    0 ] = dummy ;
	fs[ "rotationsBefore1"    ] >> dummy ; rotationsBefore[    1 ] = dummy ;
	fs[ "rotationsAfter0"     ] >> dummy ; rotationsAfter[     0 ] = dummy ;
	fs[ "rotationsAfter1"     ] >> dummy ; rotationsAfter[     1 ] = dummy ;
	fs[ "translationsBefore0" ] >> dummy ; translationsBefore[ 0 ] = dummy ;
	fs[ "translationsBefore1" ] >> dummy ; translationsBefore[ 1 ] = dummy ;
	fs[ "translationsAfter0"  ] >> dummy ; translationsAfter[  0 ] = dummy ;
	fs[ "translationsAfter1"  ] >> dummy ; translationsAfter[  1 ] = dummy ;
	
	// print/show data
//	HAWAII_PRINT(  camera                  ) ;
//	HAWAII_IMSHOW( imagesBefore[       0 ] ) ;
	HAWAII_IMSHOW( imagesBefore[       1 ] ) ;
	HAWAII_IMSHOW( imagesAfter[        0 ] ) ;
//	HAWAII_IMSHOW( imagesAfter[        1 ] ) ;
//	HAWAII_PRINT(  rotationsBefore[    0 ] ) ;
//	HAWAII_PRINT(  rotationsBefore[    1 ] ) ;
//	HAWAII_PRINT(  rotationsAfter[     0 ] ) ;
//	HAWAII_PRINT(  rotationsAfter[     1 ] ) ;
//	HAWAII_PRINT(  translationsBefore[ 0 ] ) ;
	HAWAII_PRINT(  translationsBefore[ 1 ] ) ;
	HAWAII_PRINT(  translationsAfter[  0 ] ) ;
//	HAWAII_PRINT(  translationsAfter[  1 ] ) ;
	
	// detect motion w.r.t 2nd image before and 1st image after height change
/*	cv::Mat motionBefore1, motionAfter0 ;
	detectMotion( imagesBefore[ 1 ], imagesBefore[ 0 ], motionBefore1 ) ;
	detectMotion( imagesAfter[  0 ], imagesAfter[  1 ], motionAfter0  ) ;
	HAWAII_IMSHOW( motionBefore1 ) ;
	HAWAII_IMSHOW( motionAfter0  ) ; //*/
	
	// rectify 2nd image before and 1st image after height change
	cv::Mat rectifiedBefore1, rectifiedAfter0 ;
	rectify( camera,
	         rotationsAfter[ 0 ], translationsAfter[ 0 ],
	         imagesBefore[ 1 ], imagesAfter[ 0 ],
	         rectifiedBefore1, rectifiedAfter0 ) ; //*/
	
	// wait for keystroke, return "0" as in success
	cv::waitKey() ;
}
