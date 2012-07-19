#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/video/tracking.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/gpu/gpu.hpp>

//*************** isinf and isnanis not working en windows because is C99. You mus declare as follows
#ifndef isnan
        # define isnan(x) \
            (sizeof (x) == sizeof (long double) ? isnan_ld (x) \
             : sizeof (x) == sizeof (double) ? isnan_d (x) \
             : isnan_f (x))
        static inline int isnan_f  (float       x) { return x != x; }
        static inline int isnan_d  (double      x) { return x != x; }
        static inline int isnan_ld (long double x) { return x != x; }
        #endif

        #ifndef isinf
        # define isinf(x) \
            (sizeof (x) == sizeof (long double) ? isinf_ld (x) \
             : sizeof (x) == sizeof (double) ? isinf_d (x) \
             : isinf_f (x))
        static inline int isinf_f  (float       x)
        { return !isnan (x) && isnan (x - x); }
        static inline int isinf_d  (double      x)
        { return !isnan (x) && isnan (x - x); }
        static inline int isinf_ld (long double x)
        { return !isnan (x) && isnan (x - x); }
        #endif


struct MOSAIC
{
    IplImage* imgDoble;
    IplImage* imgDobleLast;
    int FirstImagePosx;
    int FirstImagePosy;
    int finalsizex;
    int finalsizey;
    int lastsizex;
    int lastsizey;
    int levelx;
    int levely;
    int lastlevelx;
    int lastlevely;
};

//Function to DrawtheMosaic based on the current total Homography. Mosaic Image mantains a fixed size.
int drawMosaic3(MOSAIC *mosaic,IplImage* img0,CvMat* Hall,CvMat* Hold,int  channels, int q,int nofound, int interruptresize);

// draw the mosaic based on the homographies from a file. Calls drawMosaic3
int drawMosaicFromHomographies(MOSAIC *mosaic, int channels, int start_imgnum, int max_imgnum, int last_computed, bool use_prev);

//Data normalization
//Data and Data Norm are Mat of n*2 columns in the form Row(i,:)=(x,y)
void DataNormalization(CvMat* Data,CvMat* DataNorm, CvMat* T);

//Apply a homography to a set of point in sourcePoint and returns DestPoints
int ApplyHomograpy(CvMat* H1,CvMat* SourcePoint, CvMat* DestPoints);

// print an opencv matrix of the form CvMat* to standard out
void print_cv_matrix(CvMat* mat_to_print);
