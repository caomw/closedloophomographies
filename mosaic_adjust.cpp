#include "homography_custom.h"
#include <opencv2/opencv.hpp>
#include <string.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <cstdio>

using namespace cv;

const char* test_name;

void computeNewMosaic(MOSAIC *mosaic, int start_imgnum, int max_imgnum)
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

    for (int i = start_imgnum; i <= max_imgnum; i++)
    {
        printf("i = %d \n", i);
        sprintf(filename, "test_set_3/mosaico%04d.tif", i);
        img = cvLoadImage(filename);
    
        drawMosaic3(mosaic, img, new_Hall, old_Hall, 3, 0, 0, interruptresize);

        sprintf(filename, "%s/cummos%04d.tif", test_name, i);
        cvSaveImage(filename, mosaic->imgDoble);

        cvCopy(new_Hall, old_Hall, NULL);
        sprintf(filename, "%s/cumhom%04d.xml", test_name, i);
        new_Hall = (CvMat*)cvLoad(filename);
        
    }
}


int main(int argc, const char* argv[])
{
    if (argc != 4) {
        printf("need more arguments\n");
        exit(1);
    }   
    
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

    MOSAIC *mosaic_recomp;
    mosaic_recomp = new(MOSAIC);

    mosaic_recomp->finalsizex=3;
    mosaic_recomp->finalsizey=3;
    mosaic_recomp->levelx=1;
    mosaic_recomp->levely=1;
    mosaic_recomp->imgDoble = cvCreateImage( cvSize(mosaic_recomp->finalsizex*imgWidth,mosaic_recomp->finalsizey*imgHeight), 8, 3);
    mosaic_recomp->imgDobleLast = cvCreateImage( cvSize(mosaic_recomp->finalsizex*imgWidth,mosaic_recomp->finalsizey*imgHeight), 8, 3);

    int detecttype = 2;

    char filename[200];
    int firstimgnum = atoi(argv[1]); // first argument is the first image
    int lastimgnum = atoi(argv[2]); // second argument is the second image 
    test_name = argv[3];
    sprintf(filename, "rm %s/*", test_name);
    system(filename);
    sprintf(filename, "mkdir %s", test_name);
    system(filename);

    CvMat* H_all = cvCreateMat(3, 3, CV_32FC1);
    CvMat* H_old = cvCreateMat(3, 3, CV_32FC1);
    cvSetIdentity(H_all);
    cvSetIdentity(H_old);
    IplImage* imgColor; 
    IplImage* imgBW = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
    
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
    CvMat mask = imread("mask_none_white.tif", 0);
    int countfound = 0;
    int matched_features = 0;
    int point_num_limit = 300;
    CvMat* PointImg1;
    CvMat* PointImg2;

    cvNamedWindow("mosaic regular", 0);
//    cvNamedWindow("closed loop mosaic", 0);

    bool closed_loop = false; 

    int img_buffer = 20;
    for (int cur_img = firstimgnum; cur_img <= lastimgnum; cur_img++)
    {
        //load the homography
        sprintf(filename, "test_set_3/homografia%04d.xml", cur_img);
        CvMat* H = (CvMat*)cvLoad(filename);
        printf("%s\n", filename);
        //print_cv_matrix(H);
        cvGEMM(H_all, H, 1, NULL, NULL, H_all, 0);
   
        //save the cumulative homography
        sprintf(filename, "%s/cumhom%04d.xml", test_name, cur_img);
        cvSave(filename, H_all);
 
        //load the image
        sprintf(filename, "test_set_3/mosaico%04d.tif", cur_img);
        imgColor = cvLoadImage(filename);
        cvCvtColor(imgColor, imgBW, CV_BGR2GRAY);

        cvNamedWindow("cur_img");
        cvShowImage("cur_img", imgBW); 

        //look for closed loop  
        switch(detecttype){
            case 1: // when a similar-enough old previous homography has been found
                break;
            case 2: // recompute homographies for all previous images
                closed_loop = false;
 
                //check all previous images for possibility of a closed loop
                for (int olderimgnum = firstimgnum; olderimgnum < cur_img - img_buffer; olderimgnum++)
                {
                    IplImage* overlp_img = cvCreateImage( cvSize(imgWidth,imgHeight), IPL_DEPTH_8U, 3);
                    IplImage* overlap_img = cvCreateImage( cvSize(imgWidth,imgHeight), IPL_DEPTH_8U, 1);
                    sprintf(filename, "test_set_3/mosaico%04d.tif", olderimgnum);
                    overlp_img = cvLoadImage(filename);
                    //turn the image to gray
                    cvCvtColor(overlp_img, overlap_img, CV_BGR2GRAY);

                    cvNamedWindow("overlap");
                    cvShowImage("overlap", overlap_img);

                    cvGoodFeaturesToTrack(overlap_img, eig_image, tmp_image, cornersA, &corner_count, 0.01, 5.0, &mask, 3, 1, 0.04);
                    cvFindCornerSubPix(overlap_img, cornersA, corner_count, cvSize(10, 10), cvSize(-1, -1), 
                            cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03));

                    cvCalcOpticalFlowPyrLK(
                        overlap_img, 
                        imgBW,
                        pyr1,
                        pyr2,
                        cornersA,
                        cornersB,
                        corner_count,
                        cvSize(10, 10),
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
                        matched_features++;
                    }
                    
                    CvMat* H_new = cvCreateMat(3, 3, CV_32FC1);
                
                    // get a singular homography if less than 5 points, but use
                    // 10 to guarantee that you can at least find 10 of them in
                    // the next image    
                    if (matched_features >= 10) {
                        cvFindHomography(PointImg1, PointImg2, H_new, CV_RANSAC, 1.0, NULL);

                        sprintf(filename, "%s/cumhom%04d.xml", test_name, olderimgnum);
                        CvMat* H_olderimgnum = (CvMat*)cvLoad(filename);
 
                        double detHnew = cvDet(H_new);
                        printf("detHnew = %f \n", detHnew);                       
 
                        if (fabs(1 - detHnew) < 0.05){
                            //you have a cycle
                            printf("detH = %f, cur_img = %d, olderimgnum = %d\n", detHnew, cur_img, olderimgnum); 
                            
                            CvMat* H_j_prime = cvCreateMat(3, 3, CV_32FC1);
 
                            //update the homographies in the cycle
                            for (int cycle_img = olderimgnum + 1; cycle_img <= cur_img; cycle_img++)
                            {
                                printf("cycle_img = %d\n", cycle_img);

                                // the old-calculated cumulative homography for that image
                                sprintf(filename, "%s/cumhom%04d.xml", test_name, cycle_img);
                                CvMat* H_j = (CvMat*)cvLoad(filename);
                                cvSetIdentity(H_j_prime);
                                
                                // calculate H_j_prime
                                for (int mult_index = cur_img; mult_index > cycle_img; mult_index--)
                                {
                                    // printf("mult_index = %d", mult_index);
                                    sprintf(filename, "test_set_3/homografia%04d.xml", mult_index);
                                    CvMat* temp_H = (CvMat*)cvLoad(filename);
                                    cvGEMM(temp_H, H_j_prime, 1, NULL, NULL, H_j_prime);
                                }
                                
                                // printf("\n");
                                cvInvert(H_j_prime, H_j_prime, CV_LU);
                                cvGEMM(H_new, H_j_prime, 1, NULL, NULL, H_j_prime);
                                cvGEMM(H_olderimgnum, H_j_prime, 1, NULL, NULL, H_j_prime);
                                
                                double detHj = cvDet(H_j);
                                double detH_j_prime = cvDet(H_j_prime);
                                
                                printf("det H_j = %f, det H_j_prime = %f \n", detHj, detH_j_prime); 
                                
                                double detHjdiff = fabs(1 - detHj);
                                double detHjprimediff = fabs(1 - detH_j_prime);
                                
                                // choose which to keep for the cumulative
                                // homography based on which is closer to 1
                                if (detHjdiff > detHjprimediff)
                                {
                                    printf("saving sometihng new!\n");
                                    closed_loop = true;
                                    // the new one is better, so save it
                                    sprintf(filename, "%s/cumhom%04d.xml", test_name, cycle_img);
                                    cvSave(filename, H_j_prime);
                                    
                                    printf("saved new matrix H_j_prime:\n");
                                    print_cv_matrix(H_j_prime);
                                    printf("instead of the old one: \n");
                                    print_cv_matrix(H_j);
                                }
                                // otherwise don't do anything
                            } 
                        } 
                    }
                }
                break;
            default:
                printf("pick a valid option!\n");
                break;
        }
    
        // detect closed loop
        if (closed_loop)
        {
            // do something
            printf("closed loop found \n");

            //computeNewMosaic(mosaic_recomp, firstimgnum, cur_img); 
            
            //cvShowImage("closed loop mosaic", mosaic_recomp->imgDoble);
            //break;
//            waitKey(-1);
            continue;
        } //else 
        //{ 
        //    drawMosaic3(mosaic, imgColor, H_all, H_old, 3, 0, 0, 0);
        //}
        
        drawMosaic3(mosaic, imgColor, H_all, H_old, 3, 0, 0, 0);

        cvShowImage("mosaic regular", mosaic->imgDoble); 
        
//        sprintf(filename, "%s/regmos%04d.tif", test_name, cur_img);
//        cvSaveImage(filename, mosaic->imgDoble);

        cvCopy(H_all, H_old, NULL);
        
        printf("end of image %d \n", cur_img);
//        waitKey(-1);
    }
  
    printf("all done\n"); 
    cvReleaseMat(&H_all);
    cvReleaseMat(&H_old);
    cvReleaseImage(&imgColor);
    cvReleaseImage(&imgBW);

//    computeNewMosaic(mosaic_recomp, firstimgnum, lastimgnum);
//    cvShowImage("closed loop mosaic", mosaic_recomp->imgDoble);
//    sprintf(filename, "%s/mosaic_recomp%d.tif", test_name, lastimgnum);
//    cvSaveImage(filename, mosaic_recomp->imgDoble);

    waitKey(-1);
} 
