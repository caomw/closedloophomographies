#include <opencv2/opencv.hpp>
#include <string.h>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <iostream>
        
using namespace cv;

const char* test_set_name;

int main(int argc, const char* argv[])
{
    if (argc != 2) {
        printf("need more arguments\n");
        exit(1);
    }   

    test_set_name = argv[1];
    IplImage* imgCaptured = NULL;
    IplImage* imgToSave = cvCreateImage(cvSize(600, 450), IPL_DEPTH_8U, 3);
    char filename[200];
    int imgnum = 1;
    CvCapture *capture = cvCaptureFromFile("test_set_15/MVI_0741.AVI");
    double totalvideoframes= cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_COUNT );
    int frameI=(int)(totalvideoframes*((double)(frameI)/100));
    int frameF=totalvideoframes-1;
    cvSetCaptureProperty(capture,CV_CAP_PROP_POS_FRAMES,(double)(frameI));
    imgCaptured=cvQueryFrame(capture);
    cvNamedWindow("frames"); 

    for (int q = frameI; q < frameF; q++)
    {
        cvShowImage("frames", imgCaptured);
        waitKey(-1);
        
        if (q%10==0)
        {
            cvResize(imgCaptured, imgToSave);
            sprintf(filename, "%s/mosaico%04d.tif", test_set_name, imgnum);
            cvSaveImage(filename, imgToSave);
            imgnum++;
        }

        imgCaptured = cvQueryFrame(capture);
    }
    return 0;
}
