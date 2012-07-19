#include "homography.h"
#include <opencv2/opencv.hpp>
#include <string.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <cstdio>

using namespace cv;

const char* test_name;

void computeNewMosaic(MOSAIC *mosaic, int start_imgnum, int max_imgnum, const char* test_name)
{
    int interruptresize = 0;
    cvSetZero(mosaic->imgDoble);
    cvSetZero(mosaic->imgDobleLast);
    mosaic->finalsizex = 3;
    mosaic->finalsizey = 3;
    mosaic->levelx = 1;
    mosaic->levely = 1;
    char filename[200];
    CvMat* new_Hall = cvCreateMat(3, 3, CV_32FC1);
    CvMat* old_Hall = cvCreateMat(3, 3, CV_32FC1);
    cvSetIdentity(new_Hall);
    cvSetIdentity(old_Hall);
    IplImage* img;    

    cvNamedWindow("recomputed mosaic", 0);

    for (int i = start_imgnum; i <= max_imgnum; i++)
    {
        printf("i = %d \n", i);
        sprintf(filename, "test_set_3/mosaico%04d.tif", i);
        img = cvLoadImage(filename);
    
        drawMosaic3(mosaic, img, new_Hall, old_Hall, 3, 0, 0, interruptresize);

//        sprintf(filename, "%s/cummos%04d.tif", test_name, i);
//        cvSaveImage(filename, mosaic->imgDoble);

        
        cvCopy(new_Hall, old_Hall, NULL);
        sprintf(filename, "%s/cumhom%04d.xml", test_name, i);
        new_Hall = (CvMat*)cvLoad(filename);
//        sprintf(filename, "test_set_3/homografia%04d.xml", i);
//        CvMat* H = (CvMat*)cvLoad(filename);
//        cvGEMM(new_Hall, H, 1, NULL, NULL, new_Hall);
        
        cvShowImage("recomputed mosaic", mosaic->imgDoble);    
    
        waitKey(-1);
    }
}


int main(int argc, const char* argv[])
{
    int imgWidth = 600;
    int imgHeight = 328;
    
    MOSAIC *mosaic;
    mosaic = new(MOSAIC);
    
    mosaic->finalsizex=3;
    mosaic->finalsizey=3;
    mosaic->levelx=1;
    mosaic->levely=1;
    mosaic->imgDoble = cvCreateImage( cvSize(mosaic->finalsizex*imgWidth,mosaic->finalsizey*imgHeight), 8, 3);
    mosaic->imgDobleLast = cvCreateImage( cvSize(mosaic->finalsizex*imgWidth,mosaic->finalsizey*imgHeight), 8, 3);

    if (argc != 4){
        printf("need more arguments\n");
        exit(1);
    }
    
    int firstimgnum = atoi(argv[1]);
    int lastimgnum = atoi(argv[2]);
    const char* test_name = argv[3];

    computeNewMosaic(mosaic, firstimgnum, lastimgnum, test_name);
}
