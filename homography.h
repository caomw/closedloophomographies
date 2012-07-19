//#include <cv.h>
//#include <cxcore.h>
//#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/video/video.hpp>
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


// Calculate Homography error
int Reprojection_error(CvMat* Homography, CvMat* CorrespondingImagePoints_1, CvMat* CorrespondingImagePoints_2, double *Acumulated_Direct_error, double *Acumulated_Inv_error);

	
// Rotates a Image according to a defined angle
IplImage* rotatedImage(IplImage* OriginalImg, IplImage* DestinationImg, double rotangle, double rotscale);


// Remove the distotirion on the image, and the interlaced effect and make a ROI to remove black regions
//IplImage* correctImage(IplImage* imgOriginal, IplImage* imgCorrected,int undistort, int uninterlaced,int ROISIZE, CvMat *intrinsic=NULL, CvMat *distortion=NULL, IplImage* mapx = NULL,IplImage* mapy = NULL);
IplImage* correctImage(IplImage* imgOriginal, IplImage* imgCorrected,int undistort, int uninterlaced,int ROISIZE, CvMat *intrinsic, CvMat *distortion, IplImage* mapx,IplImage* mapy );

//Function to DrawtheMosaic based on the current total Homography. Mosaic Image increase size

int drawMosaic(IplImage* imgDoble,IplImage* imgDobleLast,IplImage* img0,CvMat* Hall,CvMat* Hold, int FirstImagePosx, int FirstImagePosy,int finalsizex,int finalsizey,int lastsizex,int lastsizey,int  channels, int q,int nofound, int interruptresize);

//Function to DrawtheMosaic based on the current total Homography. Mosaic Image mantains a fixed size.
int drawMosaic3(MOSAIC *mosaic,IplImage* img0,CvMat* Hall,CvMat* Hold,int  channels, int q,int nofound, int interruptresize);

// Function similar to drawMosaic3 but priorizing the first image.
int drawMosaic3Inv(MOSAIC *mosaic,IplImage* img0,CvMat* Hall,CvMat* Hold,int  channels, int q,int nofound, int interruptresize);

// draw the mosaic based on the homographies from a file. Calls drawMosaic3
int drawMosaicFromHomographies(MOSAIC *mosaic, int channels, int start_imgnum, int max_imgnum, int last_computed, bool use_prev);

//Levenber-marquard ML homography optimization
bool MLHomographyEstimation(CvMat* seedHomography, CvMat* InliersImg1,CvMat* InliersImg2, CvMat* OptH);


//Data normalization
//Data and Data Norm are Mat of n*2 columns in the form Row(i,:)=(x,y)
void DataNormalization(CvMat* Data,CvMat* DataNorm, CvMat* T);

//Homography calculation 
//PointsImg are  Mat of n*2 columns in the form Row(i,:)=(x,y)
int CalHomographyNorm(CvMat* PointsImg1,CvMat* PointsImg2, CvMat* Homography);

// SOLVE A LINER SYSTEMS FOR A EUCLIDEAN MODEL
    CvMat* lsq_euclidean( CvPoint2D32f* pts, CvPoint2D32f* mpts, int n, CvMat* H);

// SOLVE A LINEAR SYSTEM FOR AN AFFINE MODEL
  CvMat* lsq_affine( CvPoint2D32f* pts, CvPoint2D32f* mpts, int n, CvMat* H);


// RANSAC FOR AFFINE AND EUCLIDEAN MODEL
bool RANSACadjust(CvPoint2D32f* cornersA, CvPoint2D32f* cornersB,CvMat* MATwarping, int totalMatched, int model);

//Homography calculation 
//PointsImg are  Mat of n*2 columns in the form Row(i,:)=(x,y)
int CalHomography(CvMat* PointsImg1,CvMat* PointsImg2, CvMat* Homography, int KindProjection);


//Obtain the Homogrphy of a set of Points using a RANSAC using OPENCV2.1 functions.
bool CalHomographyRANSAC_opencv2(CvMat* PointsImg1, CvMat* PointsImg2, CvMat* Homography, CvMat* InliersMask);


//Obtain the Homogrphy of a set of Points using a RANSAC for discard outlierts
//PointsImg are  Mat of n*2 columns in the form Row(i,:)=(x,y)
bool CalHomographyRANSAC(CvMat* PointsImg1, CvMat* PointsImg2, CvMat* Homography, CvPoint* FramePoints);


//
//Apply a homography to a set of point in sourcePoint and returns DestPoints
int ApplyHomograpy(CvMat* H1,CvMat* SourcePoint, CvMat* DestPoints);

// print an opencv matrix of the form CvMat* to standard out
void print_cv_matrix(CvMat* mat_to_print);

//calculation of optical flow
void flow();
