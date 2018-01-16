//#include <config.h>
//#include <VP_Api/vp_api_thread_helper.h>
//#include <VP_Os/linux/vp_os_signal_dep.h>

//openCV
#include <opencv/cv.h>
#include <opencv/highgui.h>

void bottomProcessing();
/*
PROTO_THREAD_ROUTINE(frontImageProcessing, data);
PROTO_THREAD_ROUTINE(bottomImageProcessing, data);
*/
typedef struct ConnectedComponent{
  CvPoint sumPixel;
  int numberPixel;
  int identity;
  int linkTo;
}ConnectedComponent;

typedef struct ConnectedComponentsTable{
  int size;
  int sizeMax;
  ConnectedComponent* table;
}ConnectedComponentsTable;

typedef struct ch{
  int id;
  int value;
  struct ch *next;
}chain;

// find a landmark's type (some of the blue/black dots) and value (other blue/black dots), its center's pixel position 
// and the mean of its diagonals' lengths in pixels
// user note: if no symbol found, "type == value == 0" and "u == v == diag == NAN"
void detectLandmark(const cv::Mat image,
                          int&    type,
                          int&    value,
                          float&  u,
                          float&  v,
                          float&  diag);
