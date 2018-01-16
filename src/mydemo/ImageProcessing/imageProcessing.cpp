#include "ImageProcessing/imageProcessing.h"

// General Libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <exception>

//Common Datas
//#include "Common/images.h"
#include "Common/imageInterpretation.h"
#include "Common/settings.h"
//#include "Common/command.h"
//#include "Common/record.h"
//#include "Common/state.h"

//VP_sdk
//#include <VP_Os/vp_os_print.h>
//#include <VP_Os/vp_os_delay.h>

//ardrone_tool
//#include <ardrone_tool/ardrone_tool.h>
//#include <ardrone_tool/Com/config_com.h>

//openCV
#include <opencv/cv.h>
#include <opencv/highgui.h>



//Common Variable
    //images
//imageNumbered processedFrontCamera;
//imageNumbered processedBottomCamera;
    //images processed
pthread_mutex_t processedInformationUpdate_lock;
pthread_cond_t processAchieved;
bool imageBottomProcessed;
bool imageFrontProcessed;
CvPoint bottomTarget;
CvPoint frontTarget;
float bottomThetaLine = 0;
float bottomRoll = 0;
float bottomPitch = 0;
float bottomThetaLine2 = 0;
float bottomRhoLine = 320;
symbolType symbol = NOSYMBOL;
int symbolValue;
int lostBottomFrames;
int lostFrontFrames;
CvPoint *bottomIntersection;
int theNumber;
    //settings
//int colorLine;
//int colorSignals;
//int colorReperes;
//float intensityLineForBottomCamera;
//float intensityLineForFrontCamera;
//int thresholdLowBottomCamera;
//int thresholdHighBottomCamera;
//int thresholdFrontCamera;
//int limitNbPixelsReperes;

//Global variables
int colorRed = 2;
int colorBlue = 0;
int colorGreen = 1;
CvSize cameraSize = cvSize(640, 360);



////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> luminance : calculate approximative luminance of a pixel                                               //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int luminance(uchar *pixel){
  return (3*pixel[colorRed] + 6*pixel[colorGreen] + pixel[colorBlue]) ;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> colorDectection : keep the pixel of the required color and make black the others                       //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void colorDetection (IplImage *image, IplImage *imageBinary, int color, float intensity){

  uchar *pixel;
  uchar *pixelBinary;
  int i,j;

  for( i=0; i < image->width; i++ )
    for( j=0; j < image->height; j++ ){
      pixel = (uchar *) image->imageData + j * image->widthStep + i * image->nChannels;
      pixelBinary = (uchar *) imageBinary->imageData + j * imageBinary->widthStep + i * imageBinary->nChannels;
      if((pixel[color] < intensity*pixel[(color+1)%3]) || (pixel[color] < intensity*pixel[(color+2)%3]) || (luminance(pixel) < 100) || (luminance(pixel) > 1800))
         *pixelBinary = 0;
      else{
        *pixelBinary = 255;
         pixel[0] = 0;
         pixel[1] = 0;
         pixel[2] = 255;
      }
    }

  cvSmooth( imageBinary,imageBinary, CV_MEDIAN,3,0,0,0 );
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> biColorDectection : keep the pixel of the colors of signal, line and reperes and make black the others //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int biColorDetectionBottom (IplImage *image, IplImage *imageBinary1, IplImage *imageBinary2){

  uchar *pixel;
  uchar *pixelBinary1, *pixelBinary2;
  int i,j, nbRepPixel = 0;
  float lum;
  float intLine, intRep, intSig;

  for( j=0; j < image->height; j++ ){
    for( i=0; i < image->width; i++ ){
      pixel = (uchar *) image->imageData + j * image->widthStep + i * image->nChannels;
      pixelBinary1 = (uchar *) imageBinary1->imageData + j * imageBinary1->widthStep + i * imageBinary1->nChannels;
      pixelBinary2 = (uchar *) imageBinary2->imageData + j * imageBinary2->widthStep + i * imageBinary2->nChannels;

      lum = luminance(pixel);
      if (lum > luminanceUpperBound){//white is excluded
         pixel[0] = 0;
         pixel[1] = 0;
         pixel[2] = 0;
         *pixelBinary1 = 0;
         *pixelBinary2 = 0;
      }
      else if (lum > luminanceHighBound){//high lumiance, easy limit
             *pixelBinary1 = 0;
             *pixelBinary2 =0;
             if((pixel[colorReperes] > intensityReperes*pixel[colorSignals]) && (pixel[colorReperes] > intensityReperes*pixel[colorLine])){
               *pixelBinary2 = 255;
               nbRepPixel ++;
               pixel[colorReperes] = 255;
               pixel[colorLine] = 0;
               pixel[colorSignals] = 0;
             }
             else if ((pixel[colorSignals]>(intensitySignals-1)*pixel[colorReperes]+intensitySignals*pixel[colorLine]) && (pixel[colorReperes]>(intensitySignals-1)*pixel[colorSignals]+intensitySignals*pixel[colorLine])){
               pixel[colorSignals] = 255;
               pixel[colorLine] = 0;
               pixel[colorReperes] = 255;
             }
             else{
               pixel[0] = 0;
               pixel[1] = 0;
               pixel[2] = 0;
             }
      }
      else if (lum > luminanceLowerBound){//medium luminance, 
             lum = (float)(lum-luminanceLowerBound)/(float)(luminanceUpperBound-luminanceLowerBound);
             intLine = intensityHighLineForBottomCamera + sqrt(sqrt(lum))*(intensityLineForBottomCamera-intensityHighLineForBottomCamera);
             intRep = intensityHighReperes*(1-lum) + lum*intensityReperes;
             intSig = intensityHighSignals*(1-lum) + lum*intensitySignals;
             *pixelBinary1 = 0;
             *pixelBinary2 =0;
             if((pixel[colorLine] > intLine*pixel[colorReperes]) && (pixel[colorLine] > intLine*pixel[colorSignals])){
               *pixelBinary1 = 255;
               pixel[colorLine] = 255;
               pixel[colorSignals] = 0;
               pixel[colorReperes] = 0;
             }
             else if((pixel[colorReperes] > intRep*pixel[colorSignals]) && (pixel[colorReperes] > intRep*pixel[colorLine])){
               *pixelBinary2 = 255;
               nbRepPixel ++;
               pixel[colorReperes] = 255;
               pixel[colorLine] = 0;
               pixel[colorSignals] = 0;
             }
             else if ((pixel[colorSignals]>(intSig-1)*pixel[colorReperes]+intensitySignals*pixel[colorLine]) && (pixel[colorReperes]>(intSig-1)*pixel[colorSignals]+intensitySignals*pixel[colorLine])){
               pixel[colorSignals] = 255;
               pixel[colorLine] = 0;
               pixel[colorReperes] = 255;
             }
             else{
               pixel[0] = 0;
               pixel[1] = 0;
               pixel[2] = 0;
             }
      }
      else{//black is excluded
         pixel[0] = 0;
         pixel[1] = 0;
         pixel[2] = 0;
         *pixelBinary1 = 0;
         *pixelBinary2 = 0;
      }
    }
  }

return(nbRepPixel);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> splitImage : keep the part of the image on the right of the line detected                              //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void splitImage (IplImage *image, IplImage *imageBinary, CvPoint *point, float theta){

  uchar *pixelBin, *pixel;
  int i,j;
  int limit;
  float coefficient;

  if (theta == CV_PI/2)
    return;
  else if  (theta == CV_PI)
    coefficient = 0;
  else
    coefficient = 1.0/tan(theta - CV_PI/2);


  for( j=point->y; j < image->height; j++ ){
    limit = floor(point->x+(j-point->y)*coefficient)-5;
    limit = std::min(image->width, limit);
    for( i=0; i < limit ; i++ ){
      pixel = (uchar *) image->imageData + j * image->widthStep + i * image->nChannels;
      pixelBin = (uchar *) imageBinary->imageData + j * imageBinary->widthStep + i * imageBinary->nChannels;
      *pixelBin = 0;
      pixel[0] = 0;
      pixel[1] = 0;
      pixel[2] = 0;
    }
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> createConnectedComponentsTable :                                                                       //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ConnectedComponentsTable createConnectedComponentsTable(int length){

  ConnectedComponentsTable cct;
  cct.size = 0;
  cct.sizeMax = length;
  cct.table = (ConnectedComponent*)malloc(length*sizeof(ConnectedComponent));
  return (cct);

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> copyConnectedComponent :                                                                               //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void copyConnectedComponent(ConnectedComponent *dst, ConnectedComponent c){

  dst->linkTo = c.linkTo;
  dst->identity = c.identity;
  dst->numberPixel = c.numberPixel;
  dst->sumPixel.x = c.sumPixel.x;
  dst->sumPixel.y = c.sumPixel.y;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> addNewConnectedComponent :                                                                             //
//                                add Nex connectedComponent to the maion table                               //
//                                and reallocate dynamcally of the size table if necessary                    //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void addNewConnectedComponent(ConnectedComponentsTable *cct, int x, int y){ //JDC 

  ConnectedComponent cc;
  cc.sumPixel.x= x;
  cc.sumPixel.y = y;
  cc.identity = cct->size;
  cc.numberPixel = 1;
  cc.linkTo = -1;
  int i;

  if (cct->size == cct->sizeMax){
    ConnectedComponent *newTable = (ConnectedComponent*)malloc((2*cct->size +1)*sizeof(ConnectedComponent));

    for (i = 0; i < cct->size; i++)
      copyConnectedComponent(&newTable[i], cct->table[i]);

    free(cct->table);
    cct->table = newTable;
  }

  cct->table[cct->size] = cc;
  cct->size += 1;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> linkConnectedComponents : link two connected component                                                 //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void linkConnectedComponents(int id1, int id2, ConnectedComponentsTable *cct){

  if (id1 == id2 || id1 >= cct->size || id2 >= cct->size)
    return;

//Should be more efficient but with parasites detected makes the algorithme ineffective
/*int cl1, cl2;
  if (cct->table[id1].linkTo > 0)
    cl1 = cct->table[id1].linkTo;
  else
    cl1 = id1;
  if (cct->table[id2].linkTo > 0)
    cl2 = cct->table[id2].linkTo;
  else
    cl2 = id2;
  if (cl1 < cl2)
    cct->table[id2].linkTo = cl1;
  else
    cct->table[id1].linkTo = cl2;
*/


  if (id1 < id2)
    cct->table[id2].linkTo = id1;
  else
    cct->table[id1].linkTo = id2;

}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> createConnectedComponent : obvious                                                                     //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ConnectedComponentsTable connectedComponentAlgorithm(IplImage *image){

  IplImage *imageCC = cvCreateImage( cameraSize, IPL_DEPTH_16U, 1 );
  uchar *pixel, *pixelCC;
  int idCC,i,j;

  //create the main table
  ConnectedComponentsTable cct = createConnectedComponentsTable(1000); //JDC pb allocation memory : avoid any trouble but should be reduced if possible

  //for each pixel :
  for( j=0; j < image->height; j++ ){
    for( i=0; i < image->width; i++ ){

      //associate him temporarily the highest id
      idCC = cct.size;
      pixel = (uchar *) image->imageData + j * image->widthStep + i * image->nChannels;
      pixelCC = (uchar *) imageCC->imageData + j * imageCC->widthStep + i * imageCC->nChannels;
//      *pixelCC = 1200; //higher than 1000 (warning not important)
      *pixelCC = 232; // Are you kidding? The warning from the line above definitely IS important! btw: The code above also writes 232.

      //look for the lowest id in the neighbourhood and associate the pixel with
      if (*pixel == 255){
        if (j > 0){
          pixelCC = (uchar *) imageCC->imageData + (j-1) * imageCC->widthStep + i * imageCC->nChannels;
          linkConnectedComponents(*pixelCC, idCC, &cct);
          idCC = std::min((int)*pixelCC, idCC);
          if ((i+1) < image->width) {
            pixelCC = (uchar *) imageCC->imageData + (j-1) * imageCC->widthStep + (i+1) * imageCC->nChannels;
            linkConnectedComponents(*pixelCC, idCC, &cct);
            idCC = std::min((int)*pixelCC, idCC);
          }
        }
        if (i > 0){
          pixelCC = (uchar *) imageCC->imageData + j * imageCC->widthStep + (i-1) * imageCC->nChannels;
          linkConnectedComponents(*pixelCC, idCC, &cct);
          idCC = std::min((int)*pixelCC, idCC);
          if (j > 0) {
            pixelCC = (uchar *) imageCC->imageData + (j-1) * imageCC->widthStep + (i-1) * imageCC->nChannels;
            linkConnectedComponents(*pixelCC, idCC, &cct);
            idCC = std::min((int)*pixelCC, idCC);
          }
        }

        //associte the pixel to the connected component with the id it is associated with
        pixelCC = (uchar *) imageCC->imageData + j * imageCC->widthStep + i * imageCC->nChannels;
        *pixelCC = idCC;
        if (idCC < cct.size){
          cct.table[idCC].sumPixel.x += i;
          cct.table[idCC].sumPixel.y += j;
          cct.table[idCC].numberPixel += 1;
        }
        else {
          addNewConnectedComponent(&cct, i, j);
        }
      }
    }
  }
  cvReleaseImage(&imageCC);
  return(cct);
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> produitVectoriel                                                                                       //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


float produitVectoriel(CvPoint a, CvPoint b, CvPoint c, CvPoint d){

  float den = (float)((b.x-a.x)*(d.y-c.y)-(b.y-a.y)*(d.x-c.x));
  float len1 = sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
  float len2 = sqrt((d.x-c.x)*(d.x-c.x) + (d.y-c.y)*(d.y-c.y));
  return (den/(len1 * len2));

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> produitScalaire                                                                                        //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float produitScalaire(CvPoint a, CvPoint b, CvPoint c, CvPoint d){

  return ((b.x-a.x)*(d.x-c.x)+(b.y-a.y)*(d.y-c.y));

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> produitScalaireNormalized                                                                              //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float produitScalaireNormalized(CvPoint a, CvPoint b, CvPoint c, CvPoint d){

  float den = (float)((b.x-a.x)*(d.x-c.x)+(b.y-a.y)*(d.y-c.y));
  float len1 = sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
  float len2 = sqrt((d.x-c.x)*(d.x-c.x) + (d.y-c.y)*(d.y-c.y));
  return (den/(len1 * len2));

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> checkRectangle                                                                                           //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool checkRectangle(CvPoint* centers){

//  RECTANGLE Looks like:
//   0  1
//       
//   2  3

  int i, ind;
  float value, value1, value2, value3;
  CvPoint exchange;

//find the point 0
  ind = 0;
  for (i = 1; i <4; i ++){
    if ((centers[i].x + centers[i].y)<(centers[ind].x + centers[ind].y))
      ind = i;
  }

  exchange = centers[0];
  centers[0] = centers[ind];
  centers[ind] = exchange;

//Check that the rectangle must have parallel segments
  value1 = fabs(produitVectoriel(centers[0],centers[1],centers[2],centers[3]));
  value2 = fabs(produitVectoriel(centers[0],centers[2],centers[1],centers[3]));
  value3 = fabs(produitVectoriel(centers[0],centers[3],centers[1],centers[2]));
  value = std::max(value2, value3);
  value = std::max(value1, value);

  if (value1 != value && value1 > toleranceAngleRectangle){
    return (false);
  }
  if (value2 != value && value2 > toleranceAngleRectangle){
    return (false);
  }
  if (value3 != value && value3 > toleranceAngleRectangle){
    return (false);
  }

//Deduce the point 3
  if (value == value1){
    exchange = centers[1];
    centers[1] = centers[3];
    centers[3] = exchange;
  }
  else if (value == value2){
    exchange = centers[2];
    centers[2] = centers[3];
    centers[3] = exchange;
  }

//Check that the rectangle must have law angles
  if (fabs(produitScalaireNormalized(centers[1],centers[0],centers[2],centers[0])) > toleranceAngleRectangle)
    return(false);

  value1 = produitScalaire(centers[1],centers[0],centers[1],centers[0]);
  value2 = produitScalaire(centers[2],centers[0],centers[2],centers[0]);

//According to the length of the segments. Deduce the points 2 and 3
  if (value1 > value2){
    if (value1 > toleranceLengthRectangle*value2)
      return false;
    exchange = centers[2];
    centers[2] = centers[1];
    centers[1] = exchange;
  }
  else if (value2 > toleranceLengthRectangle*value1)
    return false;

  return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> checkPoint :                                                                                           //
//                   Check the existansce of a point of a precise color around the coordinate of the center   //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool checkPoint(IplImage *image, CvPoint center, int color){

  int i,j,x,y,nb;
  uchar *pixel;
  nb = 0;

  for (i=0;i<21;i++){
    for (j=0;j<21;j++){

      if (nb > limitNumberPixelForSignal)
        return true;

      x = center.x + i - 10;
      y = center.y + j - 10;

      if ( x > 0  &&  y > 0  &&  x < image->width && y < image->height){
        pixel = (uchar *) image->imageData + y * image->widthStep + x * image->nChannels;
        if (pixel[color] != 0)//Already checked
          nb++;
      }
    }
  }

  return false;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> checkSymbol                                                                                           //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void checkSymbol(IplImage *imageBinary, IplImage *image, int *typeSymbol, int *valueSymbol, float* posU, float* posV, float *diag){
  
  //default return values
  *posU = *posV = *diag = NAN;
  
  //apply connected components algorithm
  ConnectedComponentsTable cct = connectedComponentAlgorithm(imageBinary);
  int u, n, i, verification, nb = 0;
  chain *p, *q, *r, *s;
  p = NULL;
  CvPoint top4[4];
  CvPoint out4[4];
  CvPoint Diag1, Diag2;
  CvPoint center;
  bool found4;

  //rank the connected components according their size
  for (i = cct.size -1; i > -1; i--){
    u = cct.table[i].linkTo;
    n = cct.table[i].numberPixel;
    if (u != -1){
      cct.table[u].sumPixel.x += cct.table[i].sumPixel.x;
      cct.table[u].sumPixel.y += cct.table[i].sumPixel.y;
      cct.table[u].numberPixel += n;
    }
    else{
      cct.table[i].sumPixel.x = (int)floor((float)cct.table[i].sumPixel.x/(float)n);
      cct.table[i].sumPixel.y = (int)floor((float)cct.table[i].sumPixel.y/(float)n);
      q = p;
      s = NULL;
      r = (chain*)malloc(sizeof(chain));
      nb ++;
      while (q != NULL && n < q->value){
        s = q;
        q = q->next;
      }
      if (s!=NULL)
        s->next = r;
      else
        p = r;
      r->next = q;
      r->value = n;
      r->id = i;
    }
  }

  q = p;

  if (nb<4){
    free(cct.table);
    while (q != NULL){
      p = q;
      q = p->next;
      free(p);
    }
    return;
  }


// register the four main connected components
  top4[0]=cct.table[q->id].sumPixel;
  constexpr int valueMin = 15; // 30
  if (q->value > valueMin){
    cvCircle(image, cct.table[q->id].sumPixel, 10, cvScalar(0,255,255,1), 15, 8, 0);
    q = q->next;
    if (q-> value > valueMin){
      top4[1]=cct.table[q->id].sumPixel;
      cvCircle(image, cct.table[q->id].sumPixel, 10, cvScalar(0,255,255,1), 15, 8, 0);
      q = q->next;
      if (q-> value > valueMin){
        top4[2]=cct.table[q->id].sumPixel;
        cvCircle(image, cct.table[q->id].sumPixel, 10, cvScalar(0,255,255,1), 15, 8, 0);
        q = q->next;
        if (q-> value > valueMin){
          top4[3]=cct.table[q->id].sumPixel;
          cvCircle(image, cct.table[q->id].sumPixel, 10, cvScalar(0,255,255,1), 15, 8, 0);
          found4 = true;
        }
      }
    }
  }

//if we have a rectangle  whose tops are the four main connected components:
  if (found4 && checkRectangle(top4)){
      
    *posU = ( top4[0].x + top4[1].x + top4[2].x + top4[3].x ) / 4.0f;
    *posV = ( top4[0].y + top4[1].y + top4[2].y + top4[3].y ) / 4.0f;
    *diag = ( sqrt( ( top4[0].x - top4[3].x ) * ( top4[0].x - top4[3].x )
                  + ( top4[0].y - top4[3].y ) * ( top4[0].y - top4[3].y ) )
            + sqrt( ( top4[1].x - top4[2].x ) * ( top4[1].x - top4[2].x )
                  + ( top4[1].y - top4[2].y ) * ( top4[1].y - top4[2].y ) ) ) / 2.0f;
    
//Draw symbol borders
    Diag1.x = (top4[3].x-top4[0].x)/5;
    Diag1.y = (top4[3].y-top4[0].y)/5;
    Diag2.x = (top4[2].x-top4[1].x)/5;
    Diag2.y = (top4[2].y-top4[1].y)/5;
    out4[0].x = top4[0].x - Diag1.x;
    out4[0].y = top4[0].y - Diag1.y;
    out4[1].x = top4[1].x - Diag2.x;
    out4[1].y = top4[1].y - Diag2.y;
    out4[2].x = top4[2].x + Diag2.x;
    out4[2].y = top4[2].y + Diag2.y;
    out4[3].x = top4[3].x + Diag1.x;
    out4[3].y = top4[3].y + Diag1.y;
    cvLine(image, out4[0], out4[1], cvScalar(0,255,255,1), 3, 8, 0);
    cvLine(image, out4[1], out4[3], cvScalar(0,255,255,1), 3, 8, 0);
    cvLine(image, out4[3], out4[2], cvScalar(0,255,255,1), 3, 8, 0);
    cvLine(image, out4[2], out4[0], cvScalar(0,255,255,1), 3, 8, 0);

    cvCircle(image, top4[0], 3, cvScalar(0,0,0,1), 5, 8, 0);
    cvCircle(image, top4[1], 3, cvScalar(255,255,255,1), 5, 8, 0);

    *typeSymbol = 0;
    verification = 0;

//Check the control and values points
    for (i = 1; i<4; i++){
      center.x = (4-i)*top4[0].x/4.0 + i*top4[1].x/4.0;
      center.y = (4-i)*top4[0].y/4.0 + i*top4[1].y/4.0;
      *typeSymbol = 2*(*typeSymbol);
      if (checkPoint(image, center, colorSignals)){
        *typeSymbol += 1;
        verification += 1;
        cvCircle(image, center, 3, cvScalar(255,255,255,1), 5, 8, 0);
      }
    }
    for (i = 1; i<3; i++){
      center.x = (4-i)*top4[2].x/4.0 + i*top4[3].x/4.0;
      center.y = (4-i)*top4[2].y/4.0 + i*top4[3].y/4.0;
      *typeSymbol = 2*(*typeSymbol);
      if (checkPoint(image, center, colorSignals)){
        *typeSymbol += 1;
        verification += 1;
        cvCircle(image, center, 3, cvScalar(255,255,255,1), 5, 8, 0);
      }
    }

    center.x = top4[2].x/4.0 + 3*top4[3].x/4.0;
    center.y = top4[2].y/4.0 + 3*top4[3].y/4.0;
    if ((int)(checkPoint(image, center, colorSignals)) == verification%2){
      *typeSymbol = 256;
      *valueSymbol = 256;

      free(cct.table);
      q =p;
      while (q != NULL){
        p=q;
        q = p->next;
        free(p);
      }

      return;
    }

    *valueSymbol = 0;
    verification = 0;

    for (i = 1; i<5; i++){
      center.x = (5-i)*top4[0].x/5.0 + i*top4[2].x/5.0;
      center.y = (5-i)*top4[0].y/5.0 + i*top4[2].y/5.0;
      *valueSymbol = 2*(*valueSymbol);
      if (checkPoint(image, center, colorSignals)){
        *valueSymbol += 1;
        verification += 1;
        cvCircle(image, center, 3, cvScalar(255,255,0,1), 5, 8, 0);
      }
    }
    for (i = 1; i<4; i++){
      center.x = (5-i)*top4[1].x/5.0 + i*top4[3].x/5.0;
      center.y = (5-i)*top4[1].y/5.0 + i*top4[3].y/5.0;
      *valueSymbol = 2*(*valueSymbol);
      if (checkPoint(image, center, colorSignals)){
        *valueSymbol += 1;
        verification += 1;
        cvCircle(image, center, 3, cvScalar(255,0,255,1), 5, 8, 0);
      }
    }

    center.x = top4[1].x/5.0 + 4*top4[3].x/5.0;
    center.y = top4[1].y/5.0 + 4*top4[3].y/5.0;
    if ((int)(checkPoint(image, center, colorSignals)) == verification%2){
      *typeSymbol = 256;
      *valueSymbol = 256;
    }


  }

  free(cct.table);
  q =p;
  while (q != NULL){
    p=q;
    q = p->next;
    free(p);
  }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> changeCvPoint : obvious                                                                                //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void changeCvPoint(CvPoint *point, float x, float y){
  point->x = (int)x;
  point->y = (int)y;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> intersectionLines : return the intersection point of two lines resolving                               //
//                            a1y + b1x + c1 = 0                                                              //
//                            a2y + b2x + c2 = 0                                                              //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool intersectionLines(float *line1, float *line2, CvPoint *destination){

  float a1 = cos(line1[1]-CV_PI/2);
  float b1 = -sin(line1[1]-CV_PI/2);
  float c1 = -line1[0];
  float a2 = cos(line2[1]-CV_PI/2);
  float b2 = -sin(line2[1]-CV_PI/2);
  float c2 = -line2[0];

  if (a1*b2 - a2*b1 == 0){
    changeCvPoint(destination, -1,-1);
    return false;
  }

  else if (a1 == 0)
    changeCvPoint(destination, -c1/b1, (b2*c1/b1 - c2)/a2);

  else if (a2 == 0)
    changeCvPoint(destination, -c2/b2, (b1*c2/b2 - c1)/a1);

  else{
    float x = (c1*a2 - c2*a1)/(b2*a1 - b1*a2);
    changeCvPoint(destination, x,(-b2*x - c2)/a2);
  }


  if ( destination->x < 0 || destination->x > cameraSize.width || destination->y < 0 || destination->y > cameraSize.height) {
    changeCvPoint(destination, -1, -1);
    return false;
  }

  else
    return true;

}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> getIntersectionFrame : find the intersection point with the frame of the picture                       //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void getIntersectionFrame(float *line, CvPoint *destination){

  if (line[1] == CV_PI/2)
    changeCvPoint(destination, cameraSize.width, line[0]);
  else if (line[1] > CV_PI/2)
    if (line[0]/sin(line[1]-CV_PI/2) < 0)
      changeCvPoint(destination, -line[0]/sin(line[1]-CV_PI/2), 0);
    else
      changeCvPoint(destination, 0, line[0]/cos(line[1]-CV_PI/2));
  else
    if (cameraSize.width + line[0]/sin(line[1]-CV_PI/2) > 0)
      changeCvPoint(destination, -line[0]/sin(line[1]-CV_PI/2), 0);
    else
      changeCvPoint(destination, cameraSize.width, tan(line[1]-CV_PI/2)*cameraSize.width + line[0]/cos(line[1]-CV_PI/2));
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> distanceStraight : distance of the line with the vertical one                                          //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float distanceStraight(float arg){
  float theta = fabs(arg);
  return (std::min(theta,fabsf(CV_PI - theta)));
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> determineDestinationBottom : find the target of the Bottom line                                        //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
void determineDestinationBottom(IplImage *image, IplImage *imageBinary, CvSeq * lines, CvMemStorage * storage, float *line1, float *line2, CvPoint *destination, CvPoint *intersection, float *rho, float* theta, float* theta2){

  vp_os_mutex_lock(&stateUpdate_lock);
    int alt = altitude;
  vp_os_mutex_unlock(&stateUpdate_lock);

//determine de threshold for hough transform according to the altitude
  int threshold = thresholdHighBottomCamera +(alt-500)*(thresholdLowBottomCamera-thresholdHighBottomCamera)/1000;

//apply the hough transform
  lines = cvHoughLines2( imageBinary, storage, CV_HOUGH_STANDARD, 1, CV_PI/180, std::max( threshold,thresholdLowBottomCamera), 0, 0 );

  int i ;
  bool biLine = false;

  if (lines->total > 0){

    line1 = (float*)cvGetSeqElem(lines,0);

    line2[0] = 0;
    line2[1] = CV_PI/2;

//The second line must have a limit angle difference with the main line;
    for (i = 1; i < lines->total; i++){

      line2 = (float*)cvGetSeqElem(lines,i);

      if (distanceStraight(line2[1]-line1[1]) > limitDifferenceAngleBetweenTwoLines){
        biLine = true;
        break;
      }

      line2[0] = 0;
      line2[1] = CV_PI/2;

    }

//if there are two lines : the main line is the one that is the more straight, find the intersection between the lines, find the intersection between the main line and the frame
    if (biLine) {
      intersectionLines(line1, line2, intersection);
      if (intersection->x && intersection->x != -1){
        cvCircle(image, *intersection, 10, cvScalar(150,150,150,1), 15, 8, 0);
      }
      if (distanceStraight(line1[1]) > distanceStraight(line2[1])) {
        getIntersectionFrame(line2, destination);
        *rho = line2[0];
        *theta = line2[1];
        *theta2 = line1[1];
      }
//find the intersection between the main line and the frame
      else{
        getIntersectionFrame(line1, destination);
        *rho = line1[0];
        *theta = line1[1];
        *theta2 = line2[1];
      }
    }
//find the intersection between the main line and the frame
    else{
      getIntersectionFrame(line1, destination);
      *rho = line1[0];
      *theta = line1[1];
      changeCvPoint(intersection, -1, -1);
    }
  }

  else {
    changeCvPoint(destination, -1, -1);
    changeCvPoint(intersection, -1, -1);
}

}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> symbolChoice : convert point detection in symbol detection                                             //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void symbolChoice(int type, int value){

  symbolValue = value;

  switch(type){

    case 1:
      symbol = SYMBOLLAND;
    break;

    case 4:
      symbol = SYMBOLSPEED;
    break;

    case 8:
      symbol = SYMBOLALTITUDE;
    break;

    case 29:
      symbol = SYMBOLTURNRIGHT;
    break;

    case 30:
      symbol = SYMBOLTURNLEFT;
    break;

    default : 
      symbol = NOSYMBOL;
    break;

  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> frontProcessing : thread                                               //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
void frontProcessing(){

  if( !ardrone_tool_exit() ){

    PRINT("\n   frontImageProcessing thread initialisation\n\n");

    while( !ardrone_tool_exit() ){

      vp_os_delay(10);

    }


    PRINT( "frontProcessing Thread Ending\n" );
  }

  return NULL;
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ==> symbolChoice : convert point detection in symbol detection                                             //
//                                                                                                            //
//                                                                                                            //
//                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
void bottomProcessing(){

  if( !ardrone_tool_exit() )
  {

    PRINT("\n   bottomImageProcessing thread initialisation\n\n");

    //some useful variables
    CvSeq* lines = 0;
    CvMemStorage* storage = cvCreateMemStorage(0);
    float* line1 = (float *)malloc(sizeof(float)*2);
    float* line2 = (float *)malloc(sizeof(float)*2);
    CvPoint *destination = (CvPoint *)malloc(sizeof(CvPoint));
    CvPoint *intersection = (CvPoint *)malloc(sizeof(CvPoint));
    IplImage *imageToProcess1 = cvCreateImage( cameraSize, IPL_DEPTH_8U, 1);
    IplImage *imageToProcess2 = cvCreateImage( cameraSize, IPL_DEPTH_8U, 3 );
    processedBottomCamera.image = cvCreateImage( cameraSize, IPL_DEPTH_8U, 3 );
    IplImage *tempProcessedBottomCamera = cvCreateImage( cameraSize, IPL_DEPTH_8U, 3 );
    char *str;
    str = malloc(30*sizeof(char));
    char nbb[10];
    int copyNumber;
    float copyRoll, copyPitch;
    bool copyRecord;
    float nextTheta = 0, nextTheta2 = 0, nextRho = 0;
    int type=0, value=0;
    int NbPixelsReperes;

    vp_os_mutex_lock(&processedInformationUpdate_lock);
      bottomIntersection = malloc(sizeof(CvPoint));
    vp_os_mutex_unlock(&processedInformationUpdate_lock);

    vp_os_mutex_lock(&recordUpdate_lock);
      copyRecord = record;
      if(!processedBottomCamera.path)
        processedBottomCamera.path = malloc(sizeof(directory)+12*sizeof(char));
    vp_os_mutex_unlock(&recordUpdate_lock);


    while( !ardrone_tool_exit() ){
      //obtain the image that is going to be processed and copy the 
      type = 0;
      value = 0;
      pthread_mutex_lock(&imagesUpdate_lock);
        if (!imageBottomRecieved || !imageBottomCamera.image){
          pthread_cond_wait(&reception, &imagesUpdate_lock);
        }
        imageBottomRecieved = false; 

        cvCopy(imageBottomCamera.image, tempProcessedBottomCamera, NULL);
        processedBottomCamera.number = imageBottomCamera.number;
        processedBottomCamera.roll = imageBottomCamera.roll;
        copyNumber = imageBottomCamera.number;
        copyRoll = imageBottomCamera.roll;
        copyPitch = imageBottomCamera.pitch;
        if (copyRecord){
          strcpy(processedBottomCamera.path,imageBottomCamera.path);
        }
      pthread_mutex_unlock(&imagesUpdate_lock);

      //binarize the piture
      NbPixelsReperes = biColorDetectionBottom(tempProcessedBottomCamera, imageToProcess1, imageToProcess2);

     //extract the line and the destination to follow
      determineDestinationBottom(tempProcessedBottomCamera, imageToProcess1, lines, storage, line1, line2, destination, intersection, &nextRho, &nextTheta, &nextTheta2);

      // if a line is detected
      if (destination->x && destination->x != -1){
        //erase the part of the image on the left;
        splitImage(tempProcessedBottomCamera, imageToProcess2, destination, nextTheta);
        //if enough pixel of the color of the repere were detected
        if (NbPixelsReperes > limitNbPixelsReperes)
          //look for a symbol
          checkSymbol(imageToProcess2, tempProcessedBottomCamera, &type, &value);
      }


      pthread_mutex_lock(&processedInformationUpdate_lock);
        vp_os_mutex_lock(&commandUpdate_lock);

          //update extracted information

          changeCvPoint(bottomIntersection, intersection->x, intersection->y);

          if (destination->x && destination->x != -1){
            //draw the target point
            cvCircle(tempProcessedBottomCamera, *destination, 10, cvScalar(0,255,0,1), 15, 8, 0);

            bottomRhoLine = nextRho;
            bottomThetaLine = nextTheta;
            bottomRoll = copyRoll;
            bottomPitch = copyPitch;
            bottomThetaLine2 = nextTheta2;
            theNumber = copyNumber;
            symbolChoice(type, value);
            lostBottomFrames = 0;
          }
          else if (isFlying && currentTypeControls == AUTO)
            lostBottomFrames++;
          imageBottomProcessed = true;

          pthread_cond_signal(&processAchieved);//signal the image has been processed 

        vp_os_mutex_unlock(&commandUpdate_lock);
      pthread_mutex_unlock(&processedInformationUpdate_lock);

      pthread_mutex_lock(&imagesUpdate_lock);
        cvCopy(tempProcessedBottomCamera, processedBottomCamera.image, NULL);
        if (copyRecord){
          str = strcpy(str, processedBottomCamera.path);
          strcat(str, "pro.jpg");
          cvSaveImage(str, processedBottomCamera.image, 0);
          vp_os_mutex_lock(&recordUpdate_lock);

            strcpy(records[copyNumber%20].lines[2].line," processed and saved \n");
            sprintf(nbb, "%lu", (clock()-startTime)/1000);
            strcat(records[copyNumber%20].lines[2].line,"   from beginning time = ");
            strcat(records[copyNumber%20].lines[2].line,nbb);
            strcat(records[copyNumber%20].lines[2].line,"ms\n");
            records[copyNumber%20].lines[2].written = true;

          vp_os_mutex_unlock(&recordUpdate_lock);
        }
        imageBottomProcessedUndisplayed = true;
      pthread_mutex_unlock(&imagesUpdate_lock);
    }

    free(line1);
    free(line2);
    free(destination);
    free(intersection);
    free(str);
    free(bottomIntersection);
    if (copyRecord)
      free(processedBottomCamera.path);
  }
}
*/
/*
DEFINE_THREAD_ROUTINE(frontImageProcessing, data){
  frontProcessing();
  return (THREAD_RET)0;
}

DEFINE_THREAD_ROUTINE(bottomImageProcessing, data){
  bottomProcessing();
  return (THREAD_RET)0;
}
*/

// find a landmark's type (some of the blue/black dots) and value (other blue/black dots), its center's pixel position 
// and the mean of its diagonals' lengths in pixels
void detectLandmark(const cv::Mat image,
                          int&    type,
                          int&    value,
                          float&  u,
                          float&  v,
                          float&  diag){
	
	// set default return values to be overridden later
	type  = 0;
	value = 0;
	u    = NAN;
	v    = NAN;
	diag = NAN;
	
	// check input
	if (image.empty()){           throw std::invalid_argument("detectLandmark(): input image must not be empty."          ); }
	if (image.type() != CV_8UC3){ throw std::invalid_argument("detectLandmark(): input image must be of type \"CV_8UC3\"."); }
	
	// horizontally stretch the input image to a multiple of 4 columns, else the wrapped code can't handle it
	// developer note: Even is this wasn't necessary, the input image would still have to be copied because the wrapped 
	//                 code also draws on it.
	cv::Mat tempProcessedBottomCameraMat(image.rows, ((image.cols+3)/4)*4, image.type());
	cv::resize(image, tempProcessedBottomCameraMat, tempProcessedBottomCameraMat.size());
	IplImage tempProcessedBottomCamera = tempProcessedBottomCameraMat;
	
	// allocate temporary buffers
	IplImage *imageToProcess1 = cvCreateImage(tempProcessedBottomCameraMat.size(), IPL_DEPTH_8U, 1);
	IplImage *imageToProcess2 = cvCreateImage(tempProcessedBottomCameraMat.size(), IPL_DEPTH_8U, 3);
	
	// perform color filtering, continue only if enough pixels with the color of the landmarks' corners are found
	if (biColorDetectionBottom(&tempProcessedBottomCamera, imageToProcess1, imageToProcess2) > limitNbPixelsReperes){
		
		// detect and print the landmarks properties
		checkSymbol(imageToProcess2, &tempProcessedBottomCamera, &type, &value, &u, &v, &diag);
//		printf( "u=%6.2f, v=%6.2f, diag=%6.2f, type=%d, value=%d\n", u, v, diag, type, value ) ;
	}
	
	// visualize the intermediate and final results
	cvShowImage("tempProcessedBottomCamera", &tempProcessedBottomCamera);
//	cvShowImage("imageToProcess1", imageToProcess1);
//	cvShowImage("imageToProcess2", imageToProcess2);
	
	// delete temporary buffers
	cvReleaseImage(&imageToProcess1);
	cvReleaseImage(&imageToProcess2);
}
