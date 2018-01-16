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


// monocular sparse 3D reconstruction (via visual odometry)
// ========================================================


#include "odometryDrone.h"
#include "hawaii/GPU/autoMat.h"
#include "hawaii/common/helpers.h"
#include "hawaii/common/error.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <SDL/SDL_keysym.h>
#include <SDL/SDL_joystick.h>
#include "viso_mono.h"
#include "appFollowPersonFP.h"

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <opencv/cv.h>
#include <ctype.h>
#include <math.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <dirent.h> 
#include <string.h>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <locale.h>
#include <iostream>
#include <ctype.h>

 // --------------------------------Need to be used in future for drift computation-------------------------------

#define IMG_PER_360 6                        //number of images to take during scan (360° rotation)

#define DEFAULT_ROTATION_SPEED    0.3        //rotation speed during scan (360° rotation)
#define TARGET_HEIGHT               1        //height during scan (360° rotation)
#define AMPL     5
#define MAX_DRIFT  0.15

//--------------------------------drift computation ends----------------------------------------


using namespace std;
using namespace cv;
// helpers only used here
namespace {
      //Tracking COde

// To calculate the likelihood 
float calc_likelihood (IplImage* img, int x, int y)
	{
		float b, g, r;
		float dist = 0.0, sigma = 50.0;
	
		b = img->imageData[img->widthStep * y + x * 3];       //B
		g = img->imageData[img->widthStep * y + x * 3 + 1];   //G
		r = img->imageData[img->widthStep * y + x * 3 + 2];   //R
		dist = sqrt (b * b + g * g + (255.0 - r) * (255.0 - r));
		//if(dist<255.0)
		//printf("rgb %f %f %f dist %f\n",r,g,b,dist);
		return 1.0 / (sqrt (2.0 * CV_PI) * sigma) * expf (-dist * dist / (2.0 * sigma * sigma));
	}


// standard hog training configuration
	Size windowsz = Size(64,128);
	Size blockSize = Size(16,16);
	Size cellSize = Size(8,8);
	double wratio=(double)windowsz.height/(double)windowsz.width;
	const char* Trainpath = "./dataset/train";
	const char* Testpath = "./dataset/test";

// hog training functions
	void onMouse( int event, int x, int y, int, void* );
//adaptive hog variables
	Mat image;
	bool selectObject = false;
	Point origin;
	Rect selection;
	int skipAddSamples=4;
	int skipOldSamples=10;

// pf vars
	int n_stat = 4;
	int n_particle = 5000;
	int neff_num_particles=n_particle;
	CvConDensation *cond;
	CvMat *lowerBound = 0;
	CvMat *upperBound = 0;

	int resample_count=0;
	int minParticles=100;
//double Neff_norm;
	float Neff=0.0;

// main vars
	int frameNumber=0;
	Size frameSize;
	double t; // detection time
	int counter;
	int counterqueue1 = 0;
	int counterqueue = 0;
// Used in averaging the Hog detection rectangle	
	class queue
	{
	public :
	    int *a_queue,f_queue,r_queue,size_queue;
	    queue()
	    {
		f_queue=0;
		r_queue=-1;
		//cout<<"\nEnter size of the Queue\n";
		//cin>>size;
	       // a = new int [size];
		 a_queue = new int [30];
	    }
	    int isempty();
	    int isfull();
	    int push(int);
	    int pop();
	    //int display();
	    int wt_mean();
	};
	    int queue :: isempty()
	    {
		if(r_queue==-1)
		    return 1;
		return 0;
	    }
	    int queue :: isfull()
	    {
		if(r_queue==(30-1))
		    return 1;
		return 0;
	    }
	    int queue :: push(int alpha_queue)
	    {
		if(isfull())
		{
		    cout<<"\n Queue Overflow\n";
		    return 0;
		}
		else
		{
		    //cout<<"\nEnter an Element\n";
		     //cin>>pt.x            
		   a_queue[++r_queue] = alpha_queue;
		   // cout<<"Inserted successfully\n";
		}
	    }
	    int queue :: pop()
	    {
		if(isempty())
		{
		    cout<<"\nQueue Underflow\n";
		    return 0;
		}
		for(int i_queue=0;i_queue<=r_queue;i_queue++)
		    a_queue[i_queue]=a_queue[i_queue+1];
		--r_queue;
		//cout<<"Element deleted successfully\n";
	    }
	    int queue :: wt_mean()
	    {	
		int sum_queue = 0;
		int sumn_queue = 0;
		if(isempty())
		{
		    cout<<"\nQueue is empty -- No element to display\n";
		    return 0;
		}
		else
		{
		    for(int j_q=0;j_q<=r_queue;j_q++)
		        //cout<<""<<a[i]<<" ";
			{
			sum_queue = sum_queue + (j_q+1)*a_queue[j_q];
			sumn_queue = sumn_queue + j_q + 1;
			}
		}
		int mean_queue = sum_queue / sumn_queue;
		return mean_queue;  
	    }queue Tx,Ty,Tx1,Ty1,Bx1,By1;

	
//function for the detection and tracking 
bool detectPerson(cv::Mat imagefr,cv::Mat visualizationP1, double& offsetPix,double& heightdiff, const double focalLengthFrontU, const double principalPointFrontU, const double principalPointFrontV,bool& foundDot,bool& foundheightROI)
	{	
		int i1,j1;
		// random walk motion model parameters (px, deg)
		cv::Point Top ;
		cv::Point Bottom ;
		int delta_xy=5;  //5
		int delta_h=10;  //10	
		cv::namedWindow("main");
		cv::namedWindow("without_particles");	
		// load front camera image for processing
		cv::Mat temp1 = imagefr; 
		frameSize=temp1.size();
	
		frameNumber++;	
		// init pf-------
		double w = frameSize.width, h = frameSize.height;
		cond=0;
		cv::Point prevDetection;
	
		int xx, yy;
		// (4)Condensation To create a structure.
		cond = cvCreateConDensation (n_stat, 0, n_particle);
	
		// (5)To specify the maximum possible minimum state vector for each dimension.
		lowerBound = cvCreateMat (4, 1, CV_32FC1);
		upperBound = cvCreateMat (4, 1, CV_32FC1);
	
		cvmSet (lowerBound, 0, 0, 0.0);
		cvmSet (lowerBound, 1, 0, 0.0);
		cvmSet (lowerBound, 2, 0, -10.0);
		cvmSet (lowerBound, 3, 0, -10.0);
		cvmSet (upperBound, 0, 0, w);
		cvmSet (upperBound, 1, 0, h);
		cvmSet (upperBound, 2, 0, 10.0);
		cvmSet (upperBound, 3, 0, 10.0);
	
		// (6)Condensation Initialize a structure
		cvConDensInitSampleSet (cond, lowerBound, upperBound);
	
		// (7)ConDensation To specify the dynamics of the state vector in the algorithm
		cond->DynamMatr[0] = 1.0;
		cond->DynamMatr[1] = 0.0;
		cond->DynamMatr[2] = 1.0;
		cond->DynamMatr[3] = 0.0;
		cond->DynamMatr[4] = 0.0;
		cond->DynamMatr[5] = 1.0;
		cond->DynamMatr[6] = 0.0;
		cond->DynamMatr[7] = 1.0;
		cond->DynamMatr[8] = 0.0;
		cond->DynamMatr[9] = 0.0;
		cond->DynamMatr[10] = 1.0;
		cond->DynamMatr[11] = 0.0;
		cond->DynamMatr[12] = 0.0;
		cond->DynamMatr[13] = 0.0;
		cond->DynamMatr[14] = 0.0;
		cond->DynamMatr[15] = 1.0;
	
		// (8)Parameters to reconfigure the noise.
		cvRandInit (&(cond->RandS[0]), -25, 25, (int) cvGetTickCount (),CV_RAND_UNI);
		cvRandInit (&(cond->RandS[1]), -25, 25, (int) cvGetTickCount (),CV_RAND_UNI);
		cvRandInit (&(cond->RandS[2]), -5, 5, (int) cvGetTickCount (),CV_RAND_UNI);
		cvRandInit (&(cond->RandS[3]), -5, 5, (int) cvGetTickCount (),CV_RAND_UNI);		
		cv::VideoWriter vidout("out.mov", CV_FOURCC('D', 'I', 'V', 'X'),  15, frameSize);
		cv::VideoWriter vidout2("outMap.mov", CV_FOURCC('D', 'I', 'V', 'X'),  15, frameSize);
	
		//hog
		cout << "hog window size: "<< windowsz.width << " " << windowsz.height<< endl;
		cout << "hog window ratio: " <<wratio<<endl;
	
		cv::HOGDescriptor hog(windowsz, Size(16,16), Size(8,8), Size(8,8),9,1,-1,0,0.2,true);
	
	    vector<float> model;

			hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
			cout << "Load default detector"<<endl;



	    // skip frames at start
	    cv::Mat frame;
		setMouseCallback( "main", onMouse, 0 );    
		int posCount=0;
		int negCount=0;
		int windowPosCount=0;
		int windowNegCount=0;
		bool training=false;
		char name[30];
		cv::Point newtl;
		cv::Point newbr;
	
		// hog search roi (initialized as whole image)
		cv::Rect searchRoi(0,0,frameSize.width,frameSize.height);

		// state vars for adaptive hog
		bool firstTime=true;
	
		bool pause=false;
		bool startTraining=false;
		bool automaticTraining=false;
		int minPositives=5;
		int minNegatives=10;
		int maxPositives=40;
		int maxNegatives=80;
		bool detect=false;
		bool automaticAddSamples=false;
		long loopCount=0;
	
		cv::Mat positives(Size(windowsz.width/2,windowsz.height/2),CV_8UC3);
		cv::Mat old_positives(Size(windowsz.width/2,windowsz.height/2),CV_8UC3);
	
		cv::Point currDetection;
		cv::Mat threshold;
		bool doDetection=true;
	
		// main code 
				
			frame = imagefr;
                        cout<<frame.size();
			if(!frame.data)
			    cout<<"No image";
			cv::Mat img2=frame.clone();
			image=frame.clone();
			cv::Mat hogSearchROI=frame(searchRoi).clone(); // roi for hog getection
			frameNumber++;
			cv::inRange(img2,cv::Scalar(50,50,50),cv::Scalar(50,50,50),threshold);
			 firstTime=false;
					
			// temp image for drawing
			cv::Mat temp(img2.size(),CV_8UC3);
			temp.setTo(Scalar(0,0,0));	
		
		
			// measurement (hog detection)		
			cv::vector<Rect> found, found_filtered;
			found.clear();found_filtered.clear();
			t = (double)getTickCount();
			hog.detectMultiScale(hogSearchROI, found, 0, Size(8,8), Size(32,32), 1.05, 2);// Hog multiscale detection , results can substanially change with change of last two parameters.
			t = (double)getTickCount() - t;
			t=t*1000./cv::getTickFrequency();

			bool foundAtLeastOne=false;
			// filter hog detections for display
			size_t i, j;
			for( i = 0; i < found.size(); i++ )
			{
				cv::Rect r = found[i];
				for( j = 0; j < found.size(); j++ )
					if( j != i && (r & found[j]) == r)
						break;
				if( j == found.size() )
					found_filtered.push_back(r);
			}
			for( i = 0; i < found_filtered.size(); i++ )
			{
				cv::Rect r = found_filtered[i];
				//----da roi a immagine----
				r.x+=searchRoi.x;
				r.y+=searchRoi.y;
				currDetection.x= r.x+r.width/2;
				currDetection.y=r.y+r.height/2;
				//circle(temp, currDetection, 10, Scalar(0,255,255),4);
				//-------------------------
		                
				//cv::rectangle(temp,r.tl(),r.br(),Scalar(0,0,255),2);
				// Averaging of the detected box over past 20 frames
				if (counterqueue1 < 20){
		                	cv::Point ptT1 = r.tl();
					cv::Point ptB1 = r.br();
		                	Tx1.push(ptT1.x);
					Ty1.push(ptT1.y);
					Bx1.push(ptB1.x);
					By1.push(ptB1.y);
					cv::rectangle(temp,r.tl(),r.br(),Scalar(0,0,255),2);//hog detection
					cv::rectangle(temp1,r.tl(),r.br(),Scalar(0,0,255),2);
					counterqueue1++;
					Top = r.tl();
					Bottom = r.br();
		                    }
				else if (counterqueue1 >= 20){
					cv::Point ptT1 = r.tl();
					cv::Point ptB1 = r.br();
					Tx1.pop();
					Ty1.pop();
					Bx1.pop();
					By1.pop();
		                	Tx1.push(ptT1.x);
					Ty1.push(ptT1.y);
					Bx1.push(ptB1.x);
					By1.push(ptB1.y);
		                        newtl.x =Tx1.wt_mean();
		                        newtl.y =Ty1.wt_mean();
		                        newbr.x =Bx1.wt_mean();
		                        newbr.y =By1.wt_mean();
		                	//cout<<"Now Averaging "<<endl;
					rectangle(temp,newtl,newbr,Scalar(0,0,255),2);// Hog detection	
					cv::rectangle(temp1,r.tl(),r.br(),Scalar(0,0,255),2);
					Top = newtl;
					Bottom = newbr;	                 
				}
				
				double height_ROI = (Bottom.y - Top.y);	
				double frame_height = 180; // height of the resized front camera image
				heightdiff =(frame_height-height_ROI);// height_ROI; // difference between the frame height of image and the height of ROI
				if (heightdiff > 22 && heightdiff < 170) // thresholding for the change in rectangle size (depends with person's height as well)
			{
				foundheightROI = true;
			}		                
				cout << "detection: " << r.x << " "<< r.y<<endl; 
				cout << "Search roi: "<<searchRoi.x << " "<<searchRoi.y << " "<<searchRoi.width << " " << searchRoi.height<<endl;
				foundAtLeastOne=true;
				

				IplImage *result=cvCreateImage(cvSize(frameSize.width,frameSize.height),IPL_DEPTH_8U,3);
				cvZero(result);
				cvCircle(result,cvPoint(r.x+r.width/2,r.y+r.height/2),20, CV_RGB(100,0,0), -1,8,0);                               
				cvSmooth(result,result, CV_GAUSSIAN, 27);
				cv::Mat res(result);//imshow("result",res);
				// update phase
				float total=0.0;
					for (i = 0; i < n_particle; i++) {
						xx = (int) (cond->flSamples[i][0]);
						yy = (int) (cond->flSamples[i][1]);
						if (xx < 0 || xx >= w || yy < 0 || yy >= h) {
							cond->flConfidence[i] = 0.0;
						}
						else {				
							cond->flConfidence[i] = calc_likelihood (result, xx, yy);
							total+=cond->flConfidence[i];
							if(cond->flConfidence[i]>0.0001)
								printf("conf %f\n",cond->flConfidence[i]);
							cv::circle (temp, cvPoint (xx, yy), 2, CV_RGB (cond->flConfidence[i]*200, cond->flConfidence[i]*2000000, 255), -1,8,0);
						}
					}
			
				//normalize weights
				float sumWeightsSquare=0.0;
				for (i = 0; i < n_particle; i++)
				{
					cond->flConfidence[i]/=total;
					sumWeightsSquare+=cond->flConfidence[i]*cond->flConfidence[i];
				}
			
				//neff
				Neff=1.0/sumWeightsSquare;
				Neff /= (float)n_particle;
			
				//ROI is the yellow region of interest
				searchRoi.width=r.width*2;
				searchRoi.height=r.height*2;
				searchRoi.x=r.x-r.width/2;
				searchRoi.y=r.y-r.height/2;
				if(searchRoi.x<0) searchRoi.x=0;
				if(searchRoi.y<0) searchRoi.y=0;
				if(searchRoi.x+searchRoi.width>frameSize.width) searchRoi.width=frameSize.width-searchRoi.x;
				if(searchRoi.y+searchRoi.height>frameSize.height) searchRoi.height=frameSize.height-searchRoi.y;				
				}
	
			if(!foundAtLeastOne)
			{
				searchRoi.width=frameSize.width;
				searchRoi.height=frameSize.height;
				searchRoi.x=0;
				searchRoi.y=0;
			}
			else {
			
			}
		
			// resample
			cvConDensUpdateByTime (cond);
		
			//get best hyp
			//Tracking circle
			cv::Point estimatedPosition((int)cond->State[0], (int)cond->State[1]);
			cout << "Estimated position: "<< estimatedPosition.x << " "<< estimatedPosition.y <<endl;
		
			char s[50];

                        double dotXcordinate = estimatedPosition.x;
			cv::rectangle(temp,searchRoi.tl(),searchRoi.br(),Scalar(0,255,255),1);			
			cv::rectangle(temp1,searchRoi.tl(),searchRoi.br(),Scalar(0,255,255),1);
					
			
			cv::scaleAdd(temp,0.95,img2,img2);
			offsetPix = (dotXcordinate/0.5) - principalPointFrontU; //difference in the x coordinate of the dot and the principal point 
			
		
			cv::circle(img2,estimatedPosition,10,Scalar(0,255,255),2);
			cv::circle(temp1,estimatedPosition,10,Scalar(0,255,255),2);
			string Result;          // string which will contain the result
			ostringstream convert;   // stream used for the conversion
			convert << counter;      // insert the textual representation of 'counter' in the characters in the stream
			Result = convert.str(); // set 'Result' to the contents of the stream
			imwrite( "/home/drone/repos/image_withoutP/"+Result+".jpg",temp1  );
			imwrite( "/home/drone/repos/image_particles/"+Result+".jpg", img2);
			counter ++;
			/*(if (counterqueue < 6){
		                	cv::Point ptT = estimatedPosition;
		                	Tx.push(ptT.x);
					Ty.push(ptT.y);
					counterqueue++;
					cv::circle(img2,estimatedPosition,10,Scalar(0,255,255),2);
		                    }
				else if (counterqueue >= 6){
					cv::Point ptT = estimatedPosition;
					Tx.pop();
					Ty.pop();
		                	Tx.push(ptT.x);
					Ty.push(ptT.y);
					cv::Point newcen;
		                        newcen.x =Tx.wt_mean();
		                        newcen.y =Ty.wt_mean();
		                	cout<<"Average tracker "<<endl;
					cv::circle(img2,newcen,10,Scalar(0,255,255),2);		                 
				}*/

			if (!(std:: isnan(dotXcordinate))) //if the tracker dot is visible in the window
			{
				foundDot = true; 					
			}

			cv::imshow("main",img2);
			cv::imshow("without_particles",temp1);
		        //cv::imshow("new_threshold",threshold);

			vidout << img2;	
		
			cout << "Loop "<<loopCount++<<"..."<<endl;
			prevDetection=currDetection;
			char c;		
			if(pause)
				c = (char)waitKey(0);
			else
				c = (char)waitKey(200); //25 fps?
		
			if( c == ' ')
				pause=!pause;
			if(c == 't')
				startTraining=true;
			if(c=='h')
				detect=!detect;
			if(c=='a')
				automaticTraining=!automaticTraining;
			if(c=='s')
				automaticAddSamples=!automaticAddSamples;
			if(c=='r')
			{
				//pf_init_map(pf, m_map);
				searchRoi.width=frameSize.width;
				searchRoi.height=frameSize.height;
				searchRoi.x=0;
				searchRoi.y=0;
			
			}
			if(firstTime==true)
				firstTime=false;
		//}

		return true;
	}
	//------------hog-training---------------------------------------

	void onMouse( int event, int x, int y, int, void* )
	{
	    if( selectObject )
	    {
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		//selection.height = std::abs(y - origin.y);
			selection.height=floor(selection.width*wratio);
		selection &= Rect(0, 0, image.cols, image.rows);
	    }
	
	    switch( event )
	    {
			case CV_EVENT_LBUTTONDOWN:
				origin = Point(x,y);
				selection = Rect(x,y,0,0);
				selectObject = true;
				break;
			case CV_EVENT_LBUTTONUP:
				selectObject = false;
				break;
	    }
	}

//------------------ code tracking ended----------//
	
} // anonymous namespace

// c'tor with properties of undistorted camera images
appFollowPersonFP::appFollowPersonFP( const double focalLength,
                    const double principalPointU,
                    const double principalPointV ) :

	//-----------------------variable for drift computation-------------------------//
       start_defined(false),
       //-----------------------variable for drift computation-------------------------//

	// autonomous flight set points
	trackerAngleDelta(0.0),
	trackerForwardSpeed(0.0){

// instantiate wrapped visual odometry with required parameters
	VisualOdometryMono::parameters params ;
	//params.inlier_threshold =   0.00010 ; //   0.00001
	//params.motion_threshold = 150.0     ; // 100.0 TODO test
	//params.match.match_binsize   =  32 ; //  50 TODO test
	//params.match.match_radius    = 200 ; // 200
	//params.match.half_resolution =   1 ; //   1
	//params.match.refinement      =   2 ; //   1
	//params.bucket.max_features  =  2 ; //  2 TODO test
	//params.bucket.bucket_width  = 32 ; // 50 TODO test
	//params.bucket.bucket_height = 32 ; // 50 TODO test
	params.calib.f = focalLength ;
	params.calib.cu = principalPointU ;
	params.calib.cv = principalPointV ;
   //params.bucket.max_features = 4 ;
   //params.bucket.bucket_width  = 64 ;
   //params.bucket.bucket_height = 64 ;
	this->visoPtr.reset( new VisualOdometryMono( params ) ) ;

}

// filter out person detection and tracking results and determine flight commands
bool appFollowPersonFP::processImageFront( const cv::Mat imagefr,
                                  const cv::Vec3d rotationGlobal,
                                  const cv::Vec3d translationGlobal) {
	
	bool foundDot = false;
	bool foundheightROI = false; 
	cv::Mat visualizationP1 ;
	double offsetPix = 0.0;
	double heightdiff = 0.0;
       	const bool detect = detectPerson( imagefr, visualizationP1, offsetPix, heightdiff, this->visoPtr->param.calib.f, this->visoPtr->param.calib.cu, this->visoPtr->param.calib.cv,foundDot,foundheightROI) ;	

	if( foundDot ) 
	{
		// turn if dot is left or right
		if (abs(offsetPix) > 20 )//number is the set threshold for filtering
			{
				cout<<"offset="<<offsetPix<<endl;
				const double angleDelta = -0.8*atan2( offsetPix , this->visoPtr->param.calib.f ) ;
			
				this->trackerAngleDelta(std::isnan( angleDelta ) ? 0.0 : angleDelta ) ; 
			}
		else{
			this->trackerAngleDelta(   0.0 ) ;
			cout<<"Offset less "<<offsetPix<<endl; }
		// if the height of the ROI is not equal to the height of the image and outside the threshold constantly fly forward
		
		
		if (foundheightROI){ 
			cout<<"Person Far ::: Foundheight Roi "<<heightdiff<<endl;
			this->trackerForwardSpeed( 0.1 ) ;
			cout<<"given speed to move"<<endl; // uncomment this to move
			}
		else { 
			cout<<"Person Close::"<<heightdiff<<endl;
			this->trackerForwardSpeed( 0.0 ) ; //uncomment this to move 
			}	
	}

	// otherwise, fade down the control inputs
	else {
		this->trackerAngleDelta(   0.0 ) ; //uncomment this to move
		this->trackerForwardSpeed( 0.0 ) ; //uncomment this to move
	}

	return true ;//giving control commands to the drone for flight

} // method "appFollowPersonFP::processImageFront"

// get control commands

bool appFollowPersonFP::getCommands(       DroneCommands& commands, const cv::Vec3d rotationGlobal,
                            const cv::Vec3d      translationGlobal ) const {

		
    //------------------------variable needed for the drift computation--------------------------//
    double yaw,thresh_yaw,x_drift,y_drift,z_drift,drift,rectif;
    double yaw_step=CV_PI*2/IMG_PER_360;

    //initiallization for taking images
     //if( !this->start_defined ) {
            //this->img_taken=0;
            //this->start_yaw=-rotation(1);    //converts for easier use with 'command' functions (mathematical sense positive wrt commands.movement.angular.z ; see controller.h for details)
            //this->start_translation=translation;
            //this->start_defined=true;
        //}

        
	/*
        if( this->img_taken < IMG_PER_360 ){ //while we still need images...
            //Rotation speed control
            if(drift<MAX_DRIFT) {
                rectif=std::sqrt(1- (drift/MAX_DRIFT)*(drift/MAX_DRIFT) ); //rectification coefficient: large drifts -> smaller speed
                commands.movement.angular.z=DEFAULT_ROTATION_SPEED*rectif;
            } else
                commands.movement.angular.z=0; //above MAX_DRIFT, stop rotation
            //thresholds for when to take images
            yaw=-rotation(1)<this->start_yaw+yaw_step*(this->img_taken-.5) ? -rotation(1)+2*CV_PI : -rotation(1); //ensure that yaw > previousThreshold for consistent threshold conditions
            thresh_yaw=this->start_yaw+yaw_step*(this->img_taken+1); //note to self: img_taken+1 leaves some time for the drone to reach target height before taking images
            if( yaw > thresh_yaw ) {
                //store image + actual yaw + height?

                this->img_taken++;
                printf("img now! taken: %d\n",this->img_taken);
                //process image -> door candidates

            }
        } else //on 'this->img_taken < IMG_PER_360' condition
            //this->commands.hover();
            commands.movement.angular.z=0;
        //stay at fixed position and altitude
        commands.controlLinX(x_drift);
        commands.controlLinY(y_drift);
        commands.controlLinZ( TARGET_HEIGHT - height ) ;
    } else //on 'this->engaged' condition
        //reset 'start_defined' to 'false' for future attempts
        this->start_defined=false;

    cv::waitKey(1);   
   */
    //------------------------variable for drift computation--------------------------//
	
		// autonomous flight control
		//commands.movement.angular.z = 0.0 ;	
		commands.movement.linear.x = this->trackerForwardSpeed; //setting the forward speed
		commands.controllerAngZ.reset() ; // TODO disables integral control on purpose, but also derivative control as collateral damage
		commands.controlAngZ(this->trackerAngleDelta) ; //setting the yaw

		//compute x/y drifts, used for rotation speed control
	        //cv::Vec3d error=OdometryDrone::rotateTranslation(this->start_translation-translation,rotation(1));
	        x_drift= -translationGlobal(2);    //reminder: drone and odo axis
	        y_drift= -translationGlobal(0);    //    are completely different...
		z_drift=  -translationGlobal( 1 ); // height of the drone is kept constant
		
	        drift=std::sqrt(x_drift*x_drift+y_drift*y_drift);
	        printf("x drift: %lf, y drift: %lf, z drift: %lf, drift: %lf\n",x_drift,y_drift,z_drift,drift);
		

		printf( "angleZ = %5.2f, ctrl = %5.2f\n\n", (double)this->trackerAngleDelta * 180.0 / CV_PI, commands.movement.angular.z ) ;
		printf( "Forward_speed = %5.2f\n\n", (double)this->trackerForwardSpeed ) ;

	const float altitude = - translationGlobal( 1 ) ;
	
	cout<< altitude<<endl;
	
	return true ;
}



