#include <opencv2/opencv.hpp>
#include <string.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include "homography_custom.h"

using namespace cv;

const char* test_name;
const char* test_set_name;

int main(int argc, const char* argv[])
{
    if (argc != 5) {
        printf("need more arguments\n");
        exit(1);
    }   

    int imgWidth = 600;
    int imgHeight = 450; //328; //450;

    int firstimgnum = atoi(argv[1]); // first argument is the first image
    int lastimgnum = atoi(argv[2]); // second argument is the second image 
    test_name = argv[3];
    test_set_name = argv[4];
    
    MOSAIC *mosaic_recomp;
    mosaic_recomp = new(MOSAIC);
    
    mosaic_recomp->finalsizex=6;
    mosaic_recomp->finalsizey=6;
    mosaic_recomp->levelx=2;
    mosaic_recomp->levely=2;
    mosaic_recomp->imgDoble = cvCreateImage( cvSize(mosaic_recomp->finalsizex*imgWidth,mosaic_recomp->finalsizey*imgHeight), 8, 3);
    mosaic_recomp->imgDobleLast = cvCreateImage( cvSize(mosaic_recomp->finalsizex*imgWidth,mosaic_recomp->finalsizey*imgHeight), 8, 3);

    CvMat* H_recomp = cvCreateMat(3, 3, CV_32FC1);
    CvMat* H_cum = cvCreateMat(3, 3, CV_32FC1);
    CvMat* H_all = cvCreateMat(3, 3, CV_32FC1);
    cvSetIdentity(H_all);
    CvMat* H_recomp_old = cvCreateMat(3, 3, CV_32FC1);
    CvMat* H_old = cvCreateMat(3, 3, CV_32FC1);
    cvSetIdentity(H_old);
    IplImage* imgColorRecomp = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 3);//cvLoadImage(filename); 

    char filename[200];

    cvNamedWindow("recomputed_mosaic", 0);

    //redraw homographies using this new computed homographies
    for (int i = firstimgnum; i <= lastimgnum; i++)
    {
        sprintf(filename, "%s/homografia_opt%04d.xml", test_name, i);
        H_recomp = (CvMat*)cvLoad(filename);
        cvGEMM(H_all, H_recomp, 1, NULL, 0, H_all, 0);

        sprintf(filename, "%s/cumhom%04d.xml", test_name, i);
        H_cum = (CvMat*)cvLoad(filename);

        printf("H_allnew\n");
        print_cv_matrix(H_all);
        printf("H_allold\n");
        print_cv_matrix(H_cum); 

        //load the image
        sprintf(filename, "%s/mosaico%04d.tif", test_set_name, i);
        imgColorRecomp = cvLoadImage(filename);

        drawMosaic3(mosaic_recomp, imgColorRecomp, H_all, H_old, 3, 0, 0, 0);
        cvShowImage("recomputed_mosaic", mosaic_recomp->imgDoble);

        sprintf(filename, "%s/cummosaic%04d.tif", test_name, i);
        cvSaveImage(filename, mosaic_recomp->imgDoble);

        cvCopy(H_all, H_old, NULL);    
        
        printf("done with image %d\n", i); 
        waitKey(-1);
    }

    cvReleaseMat(&H_recomp);
    cvReleaseMat(&H_all);
    cvReleaseMat(&H_old);
    cvReleaseMat(&H_cum);
    cvReleaseMat(&H_recomp_old);
    cvReleaseImage(&imgColorRecomp);
    return 0;
}
