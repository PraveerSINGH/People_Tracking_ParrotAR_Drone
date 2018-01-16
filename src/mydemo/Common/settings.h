#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <VP_Api/vp_api_thread_helper.h>

#ifndef CommonSettings
#define CommonSettings


//static vp_os_mutex_t  settingsUpdate_lock = PTHREAD_MUTEX_INITIALIZER; //could be useful later

extern int colorLine;
extern int colorSignals;
extern int colorReperes;
extern int thresholdLowBottomCamera;
extern int thresholdHighBottomCamera;
extern int thresholdFrontCamera;
extern float intensityLineForFrontCamera;
extern float intensityLineForBottomCamera;
extern float intensitySignals;
extern float intensityReperes;
extern float intensityHighLineForBottomCamera;
extern float intensityHighSignals;
extern float intensityHighReperes;
extern int luminanceUpperBound;
extern int luminanceHighBound;
extern int luminanceLowerBound;
extern float toleranceAngleRectangle;
extern float toleranceLengthRectangle;
extern int limitNumberPixelForSignal;
extern float limitDifferenceAngleBetweenTwoLines;
extern int limitNbPixelsReperes;


#endif
