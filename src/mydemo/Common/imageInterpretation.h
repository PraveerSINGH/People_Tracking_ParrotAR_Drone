#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <VP_Api/vp_api_thread_helper.h>
#include <stdbool.h>
//#include <Common/record.h>


#ifndef CommonImageInterpretation
#define CommonImageInterpretation


extern pthread_mutex_t processedInformationUpdate_lock;
extern pthread_cond_t processAchieved;


typedef enum{
  NOSYMBOL,
  SYMBOLTURNRIGHT,
  SYMBOLTURNLEFT,
  SYMBOLRAISE,
  SYMBOLLAND,
  SYMBOLSPEED,
  SYMBOLALTITUDE
} symbolType;


extern bool imageBottomProcessed;
extern bool imageFrontProcessed;
extern float bottomThetaLine;
extern float bottomThetaLine2;
extern float bottomRhoLine;
extern float bottomRoll;
extern float bottomPitch;
extern CvPoint *bottomIntersection;
extern int lostBottomFrames;
extern int lostFrontFrames;
extern symbolType symbol;
extern int symbolValue;
extern int theNumber;

#endif
