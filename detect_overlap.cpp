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

    int detecttype = 1;

    test_set_name = argv[4];

    MOSAIC *mosaic;
    mosaic = new(MOSAIC);
    
    mosaic->finalsizex=6;
    mosaic->finalsizey=6;
    mosaic->levelx=2;
    mosaic->levely=2;
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

    char filename[200];
    CvPoint rtvecs[300];
    int firstimgnum = atoi(argv[1]); // first argument is the first image
    int lastimgnum = atoi(argv[2]); // second argument is the second image 
    test_name = argv[3];
    sprintf(filename, "rm %s/*", test_name);
    system(filename);
    sprintf(filename, "mkdir %s", test_name);
    system(filename);

    CvMat* H_all = cvCreateMat(3, 3, CV_32FC1);
    CvMat* H_old = cvCreateMat(3, 3, CV_32FC1);
    CvMat* H_recomp = cvCreateMat(3, 3, CV_32FC1);
    CvMat* H_recomp_old = cvCreateMat(3, 3, CV_32FC1);
    cvSetIdentity(H_all);
    cvSetIdentity(H_old);
    cvSetIdentity(H_recomp);
    cvSetIdentity(H_recomp_old);
//    sprintf(filename, "%s/mosaico%04d.tif", test_set_name, firstimgnum);
    IplImage* imgColor = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 3);//cvLoadImage(filename); 
    IplImage* imgColorRecomp = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 3);//cvLoadImage(filename); 
    IplImage* imgBW = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
    IplImage* imgBWRecomp = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
    IplImage* imgfirstBW = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
    cvNamedWindow("mosaic", 0); 
    cvNamedWindow("recomputed_mosaic", 0);
 
    bool closed_loop = false; 

    CvMat* KR = cvCreateMat(2, 2, CV_32FC1);
    CvMat* KRcum = cvCreateMat(2, 2, CV_32FC1);
    CvMat* KRinv = cvCreateMat(2, 2, CV_32FC1);
    CvMat* JKRtranspose = cvCreateMat(2, 2, CV_32FC1);
    CvMat* JKRtransposeJ = cvCreateMat(2, 2, CV_32FC1);
    CvMat* tcur = cvCreateMat(2, 1, CV_32FC1);
    cvSetZero(tcur);
    CvMat* tcum = cvCreateMat(2, 1, CV_32FC1);
    cvSetZero(tcum);
    CvMat* Rt = cvCreateMat(2, 1, CV_32FC1);
    CvMat* KRt = cvCreateMat(2, 1, CV_32FC1);
    CvMat* KRcurtcum = cvCreateMat(2, 1, CV_32FC1);
    CvMat* nKRt = cvCreateMat(2, 1, CV_32FC1);
    CvMat* nKRtcur = cvCreateMat(2, 1, CV_32FC1);
    CvMat* a13a23 = cvCreateMat(2, 1, CV_32FC1);
    CvMat* translation = cvCreateMat(2, 1, CV_32FC1);
    CvMat* K = cvCreateMat(2, 2, CV_32FC1);
    CvMat* R = cvCreateMat(2, 2, CV_32FC1);
    CvMat* J = cvCreateMat(2, 2, CV_32FC1);
    CvMat* Hoverlap = cvCreateMat(3, 3, CV_32FC1);
    
    double centerX = (mosaic->imgDoble->width)/2 - 3*imgWidth/4;
    double centerY = (mosaic->imgDoble->height)/2 - 3*imgHeight/4;
 
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

    //save the identity as the optimal homography 1 in case the optimization doesn't compute it
    sprintf(filename, "%s/homografia_opt0001.xml", test_name); //in case the first one is not optimized
    cvSave(filename, H_all);

    for (int cur_img = firstimgnum; cur_img <= lastimgnum; cur_img++)
    {   
        closed_loop = false;

        //load the homography
        sprintf(filename, "%s/homografia%04d.xml", test_set_name, cur_img);
        CvMat* H = (CvMat*)cvLoad(filename);
        printf("%s\n", filename);
        //print_cv_matrix(H);
        cvGEMM(H_all, H, 1, NULL, 0, H_all, 0);
   
        sprintf(filename, "%s/homografia_new%04d.xml", test_name, cur_img);
        cvSave(filename, H_all);  

        //save the cumulative homography
        sprintf(filename, "%s/cumhom%04d.xml", test_name, cur_img);
        cvSave(filename, H_all);

        //load the image
        sprintf(filename, "%s/mosaico%04d.tif", test_set_name, cur_img);
        imgColor = cvLoadImage(filename);
        cvCvtColor(imgColor, imgBW, CV_BGR2GRAY);
        if (cur_img == firstimgnum)
        {
            cvCopy(imgBW, imgfirstBW);
            //save time by computing the features once
            cvGoodFeaturesToTrack(imgfirstBW, eig_image, tmp_image, cornersA, &corner_count, 0.01, 5.0, NULL, 3, 1, 0.04);
            cvFindCornerSubPix(imgfirstBW, cornersA, corner_count, cvSize(10, 10), cvSize(-1, -1), 
                cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03));

        }

        cvmSet(J, 0, 1, 1);
        cvmSet(J, 1, 0, 1);
        double r11 = 0;
        double r12 = 0;
        double r21 = 0;
        double r22 = 0;
        double k11 = 0;
        double k12 = 0;
        double k22 = 0;
        double a = 0;
        double b = 0;
        double c = 0;
        double d = 0;
        double sqcsqpdsq = 0;
        double bdpac = 0;
        double bcmad = 0;
        double tx = 0;
        double ty = 0;
        double transerror = 0;   
        double dimensionerror = 1.5; 

        //look for closed loop  
        switch(detecttype){
            case 1: 
                // make submatrices 
                //cvGetSubRect(H_all, KR, cvRect(0, 0, 2, 2));
                cvGetSubRect(H, KR, cvRect(0, 0, 2, 2));
                cvInvert(KR, KRinv, CV_SVD);
               // cvGetSubRect(H_all, nKRtcur, cvRect(2, 0, 1, 2));
                cvGetSubRect(H, nKRt, cvRect(2, 0, 1, 2));
                cvGEMM(KRinv, nKRt, -1.0, NULL, 0, tcur, 0);
//                printf("nKRt = \n");
//                print_cv_matrix(nKRt);
    
//                printf("tcur = \n");
//                print_cv_matrix(tcur);

                cvGetSubRect(H_all, KRcum, cvRect(0, 0, 2, 2));
                //cvGEMM(J, KR, 1, NULL, 0, JKRtranspose, CV_GEMM_B_T);
                //cvGEMM(JKRtranspose, J, 1, NULL, 0, JKRtransposeJ, 0);

                a = cvmGet(KRcum, 0, 0);
                b = cvmGet(KRcum, 0, 1);
                c = cvmGet(KRcum, 1, 0);
                d = cvmGet(KRcum, 1, 1);
//                // DEBUGGING
//                a = 1.0; b = 2.0; c = 3.0; d = 4.0;
                sqcsqpdsq = sqrt(c*c + d*d);
                bdpac = b*d + a*c;
                bcmad = b*c - a*d;
                k22 = sqcsqpdsq;
                k12 = (bdpac)/k22;
                k11 = (bcmad)/k22;
                r22 = d/k22;
                r21 = c/k22;
                r12 = ((sqcsqpdsq*sqcsqpdsq)*b - (bdpac)*d)/(bcmad*sqcsqpdsq);
                r11 = ((sqcsqpdsq*sqcsqpdsq)*a - (bdpac)*c)/(bcmad*sqcsqpdsq);
//                printf("r11 = %f, r12 = %f, r21 = %f, r22 = %f, k11 = %f, k12 = %f, k22 = %f\n", r11, r12, r21, r22, k11, k12, k22);
                cvmSet(R, 0, 0, r11);
                cvmSet(R, 0, 1, r12);
                cvmSet(R, 1, 0, r21);
                cvmSet(R, 1, 1, r22);
                cvmSet(K, 0, 0, k11);
                cvmSet(K, 0, 1, k12);
                cvmSet(K, 1, 0, 0);
                cvmSet(K, 1, 1, k22);
                
//                printf("K = \n");
//                print_cv_matrix(K);
//                printf("R = \n");
//                print_cv_matrix(R);
 
//                cvGEMM(R, tcur, 1.0, NULL, 0, Rt, 0);
//                printf("Rtcur = \n");
//                print_cv_matrix(Rt);
               
                cvAdd(tcur, tcum, tcum);
                cvGEMM(R, tcum, 1.0, NULL, 0, Rt, 0);
                //cvGEMM(R, tcur, 1.0, NULL, 0, Rt, 0);
                cvGEMM(K, Rt, 1.0, NULL, 0, KRt, 0);
//                printf("tcum = \n");
//                print_cv_matrix(tcum);
 
//                printf("Rtcum = \n");
//                print_cv_matrix(Rt);

                cvGEMM(KRcum, tcum, 1.0, NULL, 0, KRcurtcum, 0);
//                printf("KRcurtcum = \n");
//                print_cv_matrix(KRcurtcum);

                rtvecs[cur_img - firstimgnum] = cvPoint(cvRound(cvmGet(KRcurtcum, 0, 0)), cvRound(cvmGet(KRcurtcum, 1, 0)));
                
//                cvInvert(KR, KRinv, CV_SVD);
//                cvGEMM(KRinv, t, -1.0, NULL, 0, translation);
//                printf("t: \n");
//                print_cv_matrix(translation);
                tx = rtvecs[cur_img - firstimgnum].x;
                ty = rtvecs[cur_img - firstimgnum].y;
                transerror = tx*tx/(dimensionerror * imgWidth/2 * dimensionerror * imgWidth/2) + ty*ty/(dimensionerror * imgHeight/2 * dimensionerror * imgHeight/2);
                printf("transerror = %f\n", transerror);

                if (transerror < 1 && cur_img != firstimgnum){
                    closed_loop = true;
                }
                break;
            case 2: // the 1,3 and 2,3 entries of the homography as the translation
                cvGetSubRect(H_all, a13a23, cvRect(2, 0, 1, 2));
                printf("translation\n");
                print_cv_matrix(a13a23);
                break;
            case 3: //iterative translations of points
                break;
            default:
                printf("pick a valid option!\n");
                break;
        }
    
        // detect closed loop
        if (closed_loop)
        {
            // do something
            printf("closed loop found with image number %d\n", cur_img);
            
            //compute homography with first image and the new image
            cvCalcOpticalFlowPyrLK(
                imgfirstBW, 
                imgBW,
                pyr1,
                pyr2,
                cornersA,
                cornersB,
                corner_count,
                cvSize(30, 30),
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
            
            // get a singular homography if less than 5 points, but use
            // 10 to guarantee that you can at least find 10 of them in
            // the next image    
            if (matched_features >= 10) {
                cvFindHomography(PointImg2, PointImg1, Hoverlap, CV_RANSAC, 1.0, NULL);
            }
            else{
                cvSetIdentity(Hoverlap);
            }
        
            double detHoverlap = cvDet(Hoverlap);
            printf("detHoverlap = %f\n", detHoverlap);

            if (fabs(detHoverlap - 1.0) < 0.20)
            {
                printf("calculated homography determinant = %f\n", detHoverlap);
                print_cv_matrix(Hoverlap);
        
                printf("multiplied cumulative homography: \n");
                print_cv_matrix(H_all);

                //save the homography
                sprintf(filename, "%s/homografia_new%04d.xml", test_name, cur_img);
                cvSave(filename, Hoverlap);  
 
                cvSetZero(mosaic_recomp->imgDoble);
                cvSetZero(mosaic_recomp->imgDobleLast);
          
                if (false){ 
                    //redraw homographies using this new computed homographies
                    for (int i = firstimgnum; i <= cur_img; i++)
                    {
                        sprintf(filename, "%s/homografia_new%04d.xml", test_name, i);
                        H_recomp = (CvMat*)cvLoad(filename);
                        
                        //load the image
                        sprintf(filename, "%s/mosaico%04d.tif", test_set_name, i);
                        imgColorRecomp = cvLoadImage(filename);
                        cvCvtColor(imgColorRecomp, imgBWRecomp, CV_BGR2GRAY);

                        drawMosaic3(mosaic_recomp, imgColorRecomp, H_recomp, H_recomp_old, 3, 0, 0, 0);
                        cvShowImage("recomputed_mosaic", mosaic_recomp->imgDoble);

                        cvCopy(H_recomp, H_recomp_old, NULL);     
                        waitKey(-1);
                    }
                } 
            } 
        } 
        
        drawMosaic3(mosaic, imgColor, H_all, H_old, 3, 0, 0, 0);
        //draw translation centers 
        //cvCircle(mosaic->imgDoble, cvPoint(0, 0), 5, CV_RGB(0, 0, 255), 2);
        
        
        cvCircle(mosaic->imgDoble, cvPoint(centerX, centerY), 5, CV_RGB(255, 0, 0), 2);       
        for (int i = 0; i <= cur_img - firstimgnum; i++)
        {
//            printf("x = %d, y = %d\n", rtvecs[i].x, rtvecs[i].y);
            cvCircle(mosaic->imgDoble, cvPoint(centerX - cvRound(rtvecs[i].x), centerY - cvRound(rtvecs[i].y)), 5, CV_RGB(0, 255, 0), 2);
        }

        cvShowImage("mosaic", mosaic->imgDoble); 

        sprintf(filename, "%s/moswithtrans%04d.tif", test_name, cur_img);
        cvSaveImage(filename, mosaic->imgDoble);

        cvCopy(H_all, H_old, NULL);
       
        printf("end of image %d \n", cur_img);
        waitKey(-1);
    }
   
    printf("all done\n"); 
    cvReleaseMat(&H_all);
    cvReleaseMat(&H_old);
    cvReleaseMat(&H_recomp);
    cvReleaseMat(&H_recomp_old);
    cvReleaseMat(&KR);
    cvReleaseMat(&KRinv);  
    cvReleaseMat(&JKRtranspose);
    cvReleaseMat(&JKRtransposeJ);
    cvReleaseMat(&tcur);
    cvReleaseMat(&tcum);
    cvReleaseMat(&Rt);
    cvReleaseMat(&nKRt);
    cvReleaseMat(&nKRtcur);
    cvReleaseMat(&KRt);
    cvReleaseMat(&KRcurtcum);
    cvReleaseMat(&a13a23);
    cvReleaseMat(&translation);
    cvReleaseMat(&R);
    cvReleaseMat(&K);
    cvReleaseMat(&J);    
    cvReleaseMat(&Hoverlap);    
    cvReleaseImage(&imgColor);
    cvReleaseImage(&imgColorRecomp);
    cvReleaseImage(&imgBW);
    cvReleaseImage(&imgfirstBW);
    cvReleaseImage(&imgBWRecomp);

    waitKey(-1);
} 
