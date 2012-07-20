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
    int winSize = 55;

    MOSAIC *mosaic;
    mosaic = new(MOSAIC);
   
    mosaic->finalsizex=6;
    mosaic->finalsizey=6;
    mosaic->levelx=2;
    mosaic->levely=2;
    mosaic->imgDoble = cvCreateImage( cvSize(mosaic->finalsizex*imgWidth,mosaic->finalsizey*imgHeight), 8, 3);
    mosaic->imgDobleLast = cvCreateImage( cvSize(mosaic->finalsizex*imgWidth,mosaic->finalsizey*imgHeight), 8, 3);
    cvSetZero(mosaic->imgDoble);
    cvSetZero(mosaic->imgDobleLast);

    char filename[200];
    int firstimgnum = atoi(argv[1]); // first argument is the first image
    int lastimgnum = atoi(argv[2]); // second argument is the second image 
    test_name = argv[3];
    sprintf(filename, "rm %s/*", test_name);
    system(filename);
    sprintf(filename, "mkdir %s", test_name);
    system(filename);
    test_set_name = argv[4];

    CvMat* H_all = cvCreateMat(3, 3, CV_32FC1);
    CvMat* H_old = cvCreateMat(3, 3, CV_32FC1);
    cvSetIdentity(H_all);
    cvSetIdentity(H_old);
    IplImage* imgColor = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 3); 
    IplImage* imgColorPrev = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 3); 
    IplImage* imgBW = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
    IplImage* imgBWPrev = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
    IplImage* imgC = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
     
    // spare variables for calculating homographies
    IplImage* eig_image = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
    IplImage* tmp_image = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
    IplImage* pyr1 = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
    IplImage* pyr2 = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
    const int MAX_CORNERS = 500;
    int corner_count = MAX_CORNERS;
    CvPoint2D32f* cornersA = new CvPoint2D32f[MAX_CORNERS];
    CvPoint2D32f* cornersB = new CvPoint2D32f[MAX_CORNERS];
    char features_found[MAX_CORNERS];
    float feature_errors[MAX_CORNERS];    
//    CvMat mask = imread("mask_none_white.tif", 0);
    int countfound = 0;
    int matched_features = 0;
    int point_num_limit = 300;
    CvMat* PointImg1;
    CvMat* PointImg2;

    cvNamedWindow("mosaic", 0);

    //load the first image
    sprintf(filename, "%s/mosaico%04d.tif", test_set_name, firstimgnum);
    imgColorPrev = cvLoadImage(filename);
    cvCvtColor(imgColorPrev, imgBWPrev, CV_BGR2GRAY); 
    cvCopy(imgBWPrev, imgC, NULL);

    drawMosaic3(mosaic, imgColorPrev, H_all, H_old, 3, 0, 0, 0);
    cvShowImage("mosaic", mosaic->imgDoble); 
    cvNamedWindow("points");

    waitKey(-1);
 
    for (int cur_img = firstimgnum + 1; cur_img <= lastimgnum; cur_img++)
    {
        //load the next image
        sprintf(filename, "%s/mosaico%04d.tif", test_set_name, cur_img);
        imgColor = cvLoadImage(filename);
        cvCvtColor(imgColor, imgBW, CV_BGR2GRAY);

        // calculate the homography
        cvGoodFeaturesToTrack(imgBWPrev, eig_image, tmp_image, cornersA, &corner_count, 0.01, 5.0, NULL, 3, 1, 0.04);
        cvFindCornerSubPix(imgBWPrev, cornersA, corner_count, cvSize(10, 10), cvSize(-1, -1), 
                cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03));

        cvCalcOpticalFlowPyrLK(
            imgBWPrev, 
            imgBW,
            pyr1,
            pyr2,
            cornersA,
            cornersB,
            corner_count,
            cvSize(winSize, winSize),
            3, 
            features_found,
            feature_errors,
            cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03),
            0 
            );
                    
        countfound = 0;
                    
        //count how many matched features you get from the optical flow
        for (int i = 0; i < corner_count; i++)
        {
            if (features_found[i] == 0 || feature_errors[i] > point_num_limit) {continue; }
            countfound++;
        }
        if (countfound <= 0) {countfound = 1;}
        PointImg1 = cvCreateMat(countfound, 2, CV_32F);
        PointImg2 = cvCreateMat(countfound, 2, CV_32F);
        countfound = 0;
        matched_features = 0;
        for (int i = 0; i < corner_count; i++)
        {
            if (features_found[i] == 0 || feature_errors[i] > point_num_limit) {continue; }
            CvPoint p0 = cvPoint(cvRound(cornersA[i].x), cvRound(cornersA[i].y));
            CvPoint p1 = cvPoint(cvRound(cornersB[i].x), cvRound(cornersB[i].y));
            cvmSet(PointImg1, countfound, 0, p0.x);
            cvmSet(PointImg1, countfound, 1, p0.y);
            cvmSet(PointImg2, countfound, 0, p1.x);
            cvmSet(PointImg2, countfound, 1, p1.y);
            countfound++;

            cvLine( imgC, p0, p1, CV_RGB(255,0,0),2 );
            cvCircle(imgC,p1,3,CV_RGB(0,255,0),1,8);
            
            cvShowImage("points", imgC);

            matched_features++;
        }
        
        cvShowImage("curimg", imgBW);
                   
        CvMat* H = cvCreateMat(3, 3, CV_32FC1);
             
        // get a singular homography if less than 5 points, but use
        // 10 to guarantee that you can at least find 10 of them in
        // the next image    
        if (matched_features >= 10) {
            cvFindHomography(PointImg2, PointImg1, H, CV_RANSAC, 1.0, NULL);
        }
        else{
            cvSetIdentity(H);
        }
    
        double detH = cvDet(H);

        printf("calculated homography determinant = %f\n", detH);
        print_cv_matrix(H);

        //save the homography
        sprintf(filename, "%s/homografia%04d.xml", test_set_name, cur_img);
        cvSave(filename, H);    

        if (fabs(1.0 - detH) < 0.4){
            cvGEMM(H_all, H, 1, NULL, NULL, H_all, 0);
        }
  
        //save the cumulative homography
        sprintf(filename, "%s/cumhom%04d.xml", test_name, cur_img);
        cvSave(filename, H_all);
 
        //set current image to the previous image
        cvCopy(imgColor, imgColorPrev);
        cvCopy(imgBW, imgBWPrev);
        cvCopy(imgBW, imgC);

        drawMosaic3(mosaic, imgColor, H_all, H_old, 3, 0, 0, 0);
        cvShowImage("mosaic", mosaic->imgDoble); 

//        sprintf(filename, "%s/regmos%04d.tif", test_name, cur_img);
//        cvSaveImage(filename, mosaic->imgDoble);

        cvCopy(H_all, H_old, NULL);
        
        printf("hall\n");
        print_cv_matrix(H_all);
       
        printf("end of image %d \n", cur_img);
        waitKey(-1);
    }
  
    printf("all done\n"); 
    cvReleaseMat(&H_all);
    cvReleaseMat(&H_old);
    cvReleaseImage(&imgColor);
    cvReleaseImage(&imgColorPrev);
    cvReleaseImage(&imgBW);
    cvReleaseImage(&imgBWPrev);

    waitKey(-1);
} 
