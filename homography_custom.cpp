#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <time.h>
#include "homography_custom.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/video/tracking.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/gpu/gpu.hpp>

#define MOSAIC_RES_METHOD1

#include <math.h>
double param_radial;
double param_lineal;


// Reprojection error from two images and their homography.

int Reprojection_error(CvMat* Homography, CvMat* CorrespondingImagePoints_1, CvMat* CorrespondingImagePoints_2, double *Acumulated_Direct_error, double *Acumulated_Inv_error)
{
    int index_i;
    int corresponding_number_points;
    corresponding_number_points=CorrespondingImagePoints_1->height;
    double H1_1,H1_2,H1_3,H1_4,H1_5,H1_6,H1_7,H1_8,H1_9;
    double InvH1_1,InvH1_2,InvH1_3,InvH1_4,InvH1_5,InvH1_6,InvH1_7,InvH1_8,InvH1_9;
    double x_1,y_1,x_2,y_2;
    double Direct_error, Inv_error;
    double Direct_error_array[corresponding_number_points];
    double Inv_error_array[corresponding_number_points];

//printf("Corresponding number points %d\n",corresponding_number_points);
    Direct_error=0;
    Inv_error=0;

    H1_1=cvmGet(Homography,0,0);
    H1_2=cvmGet(Homography,0,1);
    H1_3=cvmGet(Homography,0,2);
    H1_4=cvmGet(Homography,1,0);
    H1_5=cvmGet(Homography,1,1);
    H1_6=cvmGet(Homography,1,2);
    H1_7=cvmGet(Homography,2,0);
    H1_8=cvmGet(Homography,2,1);
    H1_9=cvmGet(Homography,2,2);

    CvMat *InvertedHomography;
    InvertedHomography = cvCreateMat( 3, 3, CV_32FC1 );

    //double determinante;
   //determinante=cvDet(Homography);
  // printf("determinante H %f\n",determinante);


    cvInvert(Homography,InvertedHomography,CV_LU);

    InvH1_1=cvmGet(InvertedHomography,0,0);
    InvH1_2=cvmGet(InvertedHomography,0,1);
    InvH1_3=cvmGet(InvertedHomography,0,2);
    InvH1_4=cvmGet(InvertedHomography,1,0);
    InvH1_5=cvmGet(InvertedHomography,1,1);
    InvH1_6=cvmGet(InvertedHomography,1,2);
    InvH1_7=cvmGet(InvertedHomography,2,0);
    InvH1_8=cvmGet(InvertedHomography,2,1);
    InvH1_9=cvmGet(InvertedHomography,2,2);

    for (index_i=0;index_i<corresponding_number_points;index_i++)
    {
        //int pba=0;
       // printf("Corresponding number points %d, index %d, size %d %d\n",corresponding_number_points,index_i,CorrespondingImagePoints_1->height,CorrespondingImagePoints_1->width);

        x_1=cvmGet(CorrespondingImagePoints_1,index_i,0);
        y_1=cvmGet(CorrespondingImagePoints_1,index_i,1);

        x_2=cvmGet(CorrespondingImagePoints_2,index_i,0);
        y_2=cvmGet(CorrespondingImagePoints_2,index_i,1);

        Direct_error_array[index_i]=pow(x_2 - (H1_3 + H1_1*x_1 + H1_2*y_1)/(H1_9 + H1_7*x_1 + H1_8*y_1),2) + pow(y_2 - (H1_6 + H1_4*x_1 + H1_5*y_1)/(H1_9 + H1_7*x_1 + H1_8*y_1),2);
        Direct_error=Direct_error+Direct_error_array[index_i];
        Inv_error_array[index_i]=pow(x_1 - (InvH1_3 + InvH1_1*x_2 + InvH1_2*y_2)/(InvH1_9 + InvH1_7*x_2 + InvH1_8*y_2),2) + pow(y_1 - (InvH1_6 + InvH1_4*x_2 + InvH1_5*y_2)/(InvH1_9 + InvH1_7*x_2 + InvH1_8*y_2),2);
        Inv_error=Inv_error+Inv_error_array[index_i];

     }
    *Acumulated_Direct_error=sqrt(Direct_error)/corresponding_number_points;

   // printf("Acumulated_Direct_error %f\n",*Acumulated_Direct_error);
    *Acumulated_Inv_error=sqrt(Inv_error)/corresponding_number_points;
    return 0;
}



// Rotates a Image according to a defined angle
IplImage* rotatedImage(IplImage* OriginalImg, IplImage* DestinationImg, double rotangle, double rotscale)
{
CvMat* warp_mat=cvCreateMat(2,3,CV_32FC1);

CvPoint2D32f center =cvPoint2D32f(OriginalImg->width/2,OriginalImg->height/2);

cv2DRotationMatrix(center, rotangle, rotscale, warp_mat);

cvWarpAffine(OriginalImg,DestinationImg, warp_mat);

cvReleaseMat(&warp_mat);


return DestinationImg;
}



// Remove the distotirion on the image, and the interlaced effect and make a ROI to remove black regions
IplImage* correctImage(IplImage* imgOriginal, IplImage* imgCorrected, int undistort, int uninterlaced,int ROISIZE, CvMat *intrinsic=NULL, CvMat *distortion=NULL, IplImage* mapx = NULL,IplImage* mapy = NULL)
{

	if(imgCorrected)
	{
		cvReleaseImage(&imgCorrected);
	}
	
	imgCorrected = cvCloneImage(imgOriginal);
	cvCopy(imgOriginal,imgCorrected,NULL);

	if(uninterlaced==1)// Remove the interlaced video making a liner interpolation
	{
		int i,j;
		for (i=0; i <imgCorrected->width; i++)
		{
			for (j=1; j <(imgCorrected->height-1); j=j+2)
			{
				
				((uchar*)(imgCorrected->imageData + (int)j*imgCorrected->widthStep))[(int)i*imgCorrected->nChannels+0]
				=(((uchar*)(imgCorrected->imageData + (int)(j-1)*imgCorrected->widthStep))[(int)i*imgCorrected->nChannels+0]+((uchar*)(imgCorrected->imageData + (int)(j+1)*imgCorrected->widthStep))[(int)i*imgCorrected->nChannels+0])/2;
				if(imgCorrected->nChannels==3)
				{
					((uchar*)(imgCorrected->imageData + (int)j*imgCorrected->widthStep))[(int)i*imgCorrected->nChannels+1]
					=(((uchar*)(imgCorrected->imageData + (int)(j-1)*imgCorrected->widthStep))[(int)i*imgCorrected->nChannels+1]+((uchar*)(imgCorrected->imageData + (int)(j+1)*imgCorrected->widthStep))[(int)i*imgCorrected->nChannels+1])/2;
					((uchar*)(imgCorrected->imageData + (int)j*imgCorrected->widthStep))[(int)i*imgCorrected->nChannels+2]
					=(((uchar*)(imgCorrected->imageData + (int)(j-1)*imgCorrected->widthStep))[(int)i*imgCorrected->nChannels+2]+((uchar*)(imgCorrected->imageData + (int)(j+1)*imgCorrected->widthStep))[(int)i*imgCorrected->nChannels+2])/2;
				}// end if imgCorrected->nChannels
			}// end for j
		} //end for i
	}



	
	

	if(undistort==1) //remove the camera distortion using the calibrated data
	{
	//printf( " Undisrot \n");
	//IplImage* temp = cvCloneImage(imgOriginal);
	IplImage* temp = cvCloneImage(imgCorrected);
	cvRemap( temp, imgCorrected, mapx, mapy );
	cvReleaseImage(&temp);
	//printf( " Undisrot done\n");	
	}


	if(ROISIZE !=0)// make a ROI on the image in order to remove the black frame in some captured images.
	{

		//printf( " ROISIZE\n");
		IplImage* img0Copy =cvCreateImage( cvSize((imgCorrected->width)-(2*ROISIZE),(imgCorrected->height)-(2*ROISIZE)), 8, imgCorrected->nChannels );
		cvSetImageROI(imgCorrected,cvRect(ROISIZE,ROISIZE,imgCorrected->width-(2*ROISIZE),imgCorrected->height-(2*ROISIZE)));

				//printf( "Before Image 0  (%d %d)  (%d,%d) \n",img0Copy->width,img0Copy->height, (img0->width)-(ROISIZE),(img0->height)-(ROISIZE));			
		cvCopy(imgCorrected,img0Copy,NULL);

		cvResetImageROI(imgCorrected);
		cvReleaseImage(&imgCorrected);
		imgCorrected=cvCloneImage(img0Copy);
		cvCopy(img0Copy,imgCorrected,NULL);
		cvReleaseImage(&img0Copy);
		//printf( " ROISIZE done\n");
	}


	
return imgCorrected;
}
										
						
/////////////////////// Version 3 DrawMosaic
/// This fuction draws the mosaic, but allways have the same size

int drawMosaic3(MOSAIC *mosaic,IplImage* img0,CvMat* Hall,CvMat* Hold,int  channels, int q,int nofound, int interruptresize)
{
    double h11 = cvmGet( Hall, 0, 0 );
    double h12 = cvmGet( Hall, 0, 1 );
    double h13 = cvmGet( Hall, 0, 2 );
    double h21 = cvmGet( Hall, 1, 0 );
    double h22 = cvmGet( Hall, 1, 1 );
    double h23 = cvmGet( Hall, 1, 2 );
    double h31 = cvmGet( Hall, 2, 0 );
    double h32 = cvmGet( Hall, 2, 1 );
    double h33 = cvmGet( Hall, 2, 2 );

//    printf("h11 = %f, h12 = %f, h13 = %f, h21 = %f, h22 = %f, h23 = %f, h31 = %f, h32 = %f, h33 = %f\n", h11, h12, h13, h21, h22, h23, h31, h32, h33);

    double x,y,z;
    CvRect recROI;
    int i,j;

    IplImage* imgDobleCopy;

    cvCopy(mosaic->imgDoble,mosaic->imgDobleLast);
    mosaic->lastsizex=mosaic->finalsizex;
    mosaic->lastsizey=mosaic->finalsizey;
    mosaic->lastlevelx=mosaic->levelx;
    mosaic->lastlevely=mosaic->levely;

    mosaic->FirstImagePosx=(int)(mosaic->imgDoble->width*mosaic->levelx/mosaic->finalsizex);
    mosaic->FirstImagePosy=(int)(mosaic->imgDoble->height*mosaic->levely/mosaic->finalsizey);

    //printf("Beginning Mosaic %d, pos (%d,%d) size (%d,%d) img0Size(%d,%d) \n",q, mosaic->FirstImagePosx, mosaic->FirstImagePosy,mosaic->finalsizex, mosaic->finalsizey, img0->width, img0->height );


    for (i=0; i <img0->width; i++)
    {
        for (j=0; j <img0->height; j++)
        {
            param_radial=((pow((i-(img0->width/2)),2)+pow((-j+(img0->height/2)),2))/(pow(img0->width/2,2)+pow(img0->height/2,2))); //Función de peso para los píxeles de la imagen antigua, en el centro es cero y el extremo 1
            //                z = 1/(h31*i+h32*j+h33);
            //                x = (mosaic->FirstImagePosx+(h11*i+h12*j+h13)*z);
            //                y = (mosaic->FirstImagePosy+(h21*i+h22*j+h23)*z);


            param_lineal=fabs((pow((i-(img0->width/1.2)),1)+pow((-j+(img0->height/1.2)),1))/(pow(img0->width/1.2,1)+pow(img0->height/1.2,1))); //Función de peso para los píxeles de la imagen antigua, en el centro es cero y el extremo 1


            z = 1/(h31*i+h32*j+h33);
            x = (int) mosaic->FirstImagePosx+((h11*i+h12*j+h13)*z)*3/mosaic->finalsizex;
            y = (int) mosaic->FirstImagePosy+((h21*i+h22*j+h23)*z)*3/mosaic->finalsizey;

//            printf("i = %d, j = %d, x = %f, y = %f, z = %f\n", i, j, x, y, z);


            if ((isinf(z)==1) || (isinf(x)==1) || (isinf(y)==1)) // used when z=inf or nan
            {
                nofound++;
                //printf(" inf x %f, y %f, z %f \n",x,y,z);
                cvCopy(Hold,Hall,NULL);
                interruptresize=1;
                break;
                //continue;
            }

            if ( (isnan(z)==1) || (isnan(x)==1) || (isnan(y)==1)) // used when z=inf or nan
            {
                //printf(" NAN x %f, y %f, z %f \n",x,y,z);
                nofound++;
                cvCopy(Hold,Hall,NULL);
                interruptresize=1;
                break;
                //continue;
            }


            while((int(x)<0) || (int(x)>mosaic->imgDoble->width) || (int(y)<0) || (int(y)>mosaic->imgDoble->height) )  //change image size X
            {
                printf("sizechange\n");



                if ((int(x)<0) || (int(x)>mosaic->imgDoble->width))
                {
                    mosaic->finalsizex++;

                    if (int(x)<0)
                    {
                        mosaic->levelx++;
                        //mosaic->levely++;
                    }
                   mosaic->finalsizey++;


                }
                if ((int(y)<0) || (int(y)>mosaic->imgDoble->height))
                {
                    mosaic->finalsizey++;
                    if (int(y)<0)
                    {
                        mosaic->levely++;
                        //mosaic->levelx++;
                    }
                    mosaic->finalsizex++;

                }


                mosaic->FirstImagePosx=(int)(mosaic->imgDoble->width*mosaic->levelx/mosaic->finalsizex);
                mosaic->FirstImagePosy=(int)(mosaic->imgDoble->height*mosaic->levely/mosaic->finalsizey);

                imgDobleCopy =cvCreateImage(cvSize((int) (mosaic->imgDoble->width*(mosaic->finalsizex-1)/mosaic->finalsizex),(int) (mosaic->imgDoble->height*(mosaic->finalsizey-1)/mosaic->finalsizey)), 8, channels );

              cvResize(mosaic->imgDoble,imgDobleCopy, CV_INTER_AREA);
              //cvResize(mosaic->imgDoble,imgDobleCopy, CV_INTER_LINEAR );

              //printf("Beginning Mosaic %d, pos (%d,%d) finalsize (%d,%d) img0Size(%d,%d) level(%d,%d) \n",q, mosaic->FirstImagePosx, mosaic->FirstImagePosy,mosaic->finalsizex, mosaic->finalsizey, img0->width, img0->height, mosaic->levelx, mosaic->levely );

              cvSetZero(mosaic->imgDoble);

              recROI.x=0+mosaic->imgDoble->width*(mosaic->levelx-1)/mosaic->finalsizex;
              recROI.y=0+mosaic->imgDoble->height*(mosaic->levely-1)/mosaic->finalsizey;
              recROI.width=imgDobleCopy->width;
              recROI.height=imgDobleCopy->height;

              if(recROI.x<0){recROI.x=0;}
                if(recROI.y<0){recROI.y=0;}
                if(recROI.width> mosaic->imgDoble->width){recROI.width=mosaic->imgDoble->width;}
                if(recROI.height> mosaic->imgDoble->height){recROI.height=mosaic->imgDoble->height;}
                if(recROI.width+recROI.x> mosaic->imgDoble->width){recROI.width=mosaic->imgDoble->width-recROI.x;}
                if(recROI.height+recROI.y> mosaic->imgDoble->height){recROI.height=mosaic->imgDoble->height-recROI.y;}
                cvSetImageROI(mosaic->imgDoble,recROI);
                cvSetImageROI(imgDobleCopy,cvRect(0,0,recROI.width,recROI.height));
              //cvSetImageROI(mosaic->imgDoble,cvRect(0+mosaic->imgDoble->width*(mosaic->levelx-1)/mosaic->finalsizex,0+mosaic->imgDoble->height*(mosaic->levely-1)/mosaic->finalsizey,imgDobleCopy->width,imgDobleCopy->height));
               cvCopy(imgDobleCopy,mosaic->imgDoble,NULL);
                cvResetImageROI(mosaic->imgDoble);
                cvResetImageROI(imgDobleCopy);

//                                cvNamedWindow( "imageD", 0 );
//                                cvShowImage( "imageD", mosaic->imgDoble); // se muestran las imagenes en dichas ventanas
//                                cvNamedWindow( "imageDC", 0 );
//                                cvShowImage( "imageDC", imgDobleCopy); // se muestran las imagenes en dichas ventanas
//                                cvWaitKey(0);

                z = 1/(h31*i+h32*j+h33);
                x = (int) mosaic->FirstImagePosx+((h11*i+h12*j+h13)*z)*3/mosaic->finalsizex;
                y = (int) mosaic->FirstImagePosy+((h21*i+h22*j+h23)*z)*3/mosaic->finalsizey;



                cvReleaseImage( &imgDobleCopy );



                if ( ((mosaic->finalsizex-mosaic->lastsizex)>=3)||((mosaic->finalsizey-mosaic->lastsizey)>=3)           ) // used when z=inf or nan
                {
                    printf("Too hihg image %d resizing Diff %d,  %d, Returnign to size (%d,%d) \n",q,(mosaic->finalsizex-mosaic->lastsizex),(mosaic->finalsizey-mosaic->lastsizey), mosaic->lastsizex, mosaic->lastsizey);
                    nofound++;
                    cvCopy(Hold,Hall,NULL);
                    mosaic->finalsizex=mosaic->lastsizex;
                    mosaic->finalsizey=mosaic->lastsizey;
                    mosaic->levelx=mosaic->lastlevelx;
                    mosaic->levely=mosaic->lastlevely;

                    cvCopy(mosaic->imgDobleLast,mosaic->imgDoble);




                    mosaic->FirstImagePosx=(int)(mosaic->imgDoble->width*mosaic->levelx/mosaic->finalsizex);
                    mosaic->FirstImagePosy=(int)(mosaic->imgDoble->height*mosaic->levely/mosaic->finalsizey);
                    z = 1/(h31*i+h32*j+h33);
                    x = (int) mosaic->FirstImagePosx+((h11*i+h12*j+h13)*z)*3/mosaic->finalsizex;
                    y = (int) mosaic->FirstImagePosy+((h21*i+h22*j+h23)*z)*3/mosaic->finalsizey;
                    interruptresize=1;
                    break;


                }


            }// end while

            if(interruptresize==1)
            {
                printf("Breaking image  j resize %d \n",q);
                //                mosaic->FirstImagePosx=(mosaic->imgDoble->width)/2-(img0->width)/2;
                //                mosaic->FirstImagePosy=(mosaic->imgDoble->height)/2-(img0->height)/2;
                break;
            }




            z = 1/(h31*i+h32*j+h33);
            x = (int) mosaic->FirstImagePosx+((h11*i+h12*j+h13)*z)*3/mosaic->finalsizex;
            y = (int) mosaic->FirstImagePosy+((h21*i+h22*j+h23)*z)*3/mosaic->finalsizey;

            if(
                    (((uchar*)(mosaic->imgDoble->imageData + (int)y*mosaic->imgDoble->widthStep))[(int)x*mosaic->imgDoble->nChannels + 0]==0)

                    &&
                    (((uchar*)(mosaic->imgDoble->imageData + (int)y*mosaic->imgDoble->widthStep))[(int)x*mosaic->imgDoble->nChannels + 1]==0)
                    &&
                    (((uchar*)(mosaic->imgDoble->imageData + (int)y*mosaic->imgDoble->widthStep))[(int)x*mosaic->imgDoble->nChannels + 2]==0)
                    )
            {


                ((uchar*)(mosaic->imgDoble->imageData + (int)y*mosaic->imgDoble->widthStep))[(int)x*mosaic->imgDoble->nChannels+0] = ((uchar*)(img0->imageData + (int)j*img0->widthStep))[(int)i*img0->nChannels + 0]*(1-param_lineal);

                if(mosaic->imgDoble->nChannels==3)
                {

                    ((uchar*)(mosaic->imgDoble->imageData + (int)y*mosaic->imgDoble->widthStep))[(int)x*mosaic->imgDoble->nChannels + 1]
                            = ((uchar*)(img0->imageData + (int)j*img0->widthStep))[(int)i*img0->nChannels + 1]*(1-param_lineal);

                    ((uchar*)(mosaic->imgDoble->imageData +(int)y*mosaic->imgDoble->widthStep))[(int)x*mosaic->imgDoble->nChannels + 2]
                            = ((uchar*)(img0->imageData + (int)j*img0->widthStep))[(int)i*img0->nChannels + 2]*(1-param_lineal);
                }// end if mosaic->imgDoble->nChannels==3)


            }
            else
            {
              //   printf("param_radial %f\n",param_radial);

                ((uchar*)(mosaic->imgDoble->imageData + (int)y*mosaic->imgDoble->widthStep))[(int)x*mosaic->imgDoble->nChannels+0]
                        =( ((uchar*)(img0->imageData + (int)j*img0->widthStep))[(int)i*img0->nChannels + 0]*(1-param_radial)+((uchar*)(mosaic->imgDoble->imageData + (int)y*mosaic->imgDoble->widthStep))[(int)x*mosaic->imgDoble->nChannels+0]*param_radial )  ;
                if(mosaic->imgDoble->nChannels==3)
                {
                    ((uchar*)(mosaic->imgDoble->imageData + (int)y*mosaic->imgDoble->widthStep))[(int)x*mosaic->imgDoble->nChannels+1]
                            =( ((uchar*)(img0->imageData + (int)j*img0->widthStep))[(int)i*img0->nChannels + 1]*(1-param_radial)+((uchar*)(mosaic->imgDoble->imageData + (int)y*mosaic->imgDoble->widthStep))[(int)x*mosaic->imgDoble->nChannels+1]*param_radial )  ;
                    ((uchar*)(mosaic->imgDoble->imageData + (int)y*mosaic->imgDoble->widthStep))[(int)x*mosaic->imgDoble->nChannels+2]
                            =( ((uchar*)(img0->imageData + (int)j*img0->widthStep))[(int)i*img0->nChannels + 2]*(1-param_radial)+((uchar*)(mosaic->imgDoble->imageData + (int)y*mosaic->imgDoble->widthStep))[(int)x*mosaic->imgDoble->nChannels+2]*param_radial )  ;
                    //   =( ((uchar*)(img0->imageData + (int)j*img0->widthStep))[(int)i*img0->nChannels + 2]*((uchar*)(mosaic->imgDoble->imageData + (int)y*mosaic->imgDoble->widthStep))[(int)x*mosaic->imgDoble->nChannels+2])/2;
                }// end if mosaic->imgDoble->nChannels==3)

            } // End Else


        } // END for (j=0; j <img0->height; j++)




        if(interruptresize==1)
        {
            //printf("Breaking image i resize %q \n",q);
            interruptresize=0;
            //break;
            return 0;
        }
    }  // end for (i=0; i <img0->width; i++)ç



//   printf("End Mosaic %d, pos (%d,%d) size (%d,%d) img0Size(%d,%d) \n",q, mosaic->FirstImagePosx, mosaic->FirstImagePosy,mosaic->finalsizex, mosaic->finalsizey, img0->width, img0->height );

    return 1;
}



//**************************************************************************************
void DataNormalization(CvMat* Data,CvMat* DataNorm, CvMat* T)
{
	//Normalizarion works fine for Homography and Affine projection. Not well for perspective.
	int i;
	double sumI = 0, sumJ = 0, meanI = 0, meanJ = 0;
	double squareDist = 0, sumDist = 0, meanDist = 0;
	double scale = 0;
	double x, y, xx, yy, ww;
	
	int numOfPositions=Data->height;	
	
	// calculate the centroid
	for(i = 0; i < numOfPositions; i++)
	{
		sumI += cvmGet(Data,i,0);
		sumJ += cvmGet(Data,i,1);;
	}
	meanI = sumI / numOfPositions;
	meanJ = sumJ / numOfPositions;
	// calculate the mean distance
	for(i = 0; i < numOfPositions; i++)
	{
		squareDist = pow(cvmGet(Data,i,0) - meanI, 2)+ pow(cvmGet(Data,i,1) - meanJ, 2);
		sumDist += pow(squareDist, 0.5);
	}
	meanDist = sumDist / numOfPositions;


	// set the similarity transform
	scale = pow(1, 0.5) / meanDist;
	double t[9] = {scale, 0, -scale * meanI,0, scale, -scale * meanJ,0, 0, 1};
	//Array2CvMat(t, T, 3, 3);
	cvmSet(T,0,0,scale); cvmSet(T,0,1,0); cvmSet(T,0,2,-scale*meanI);
	cvmSet(T,1,0,0); cvmSet(T,1,1,scale); cvmSet(T,1,2,-scale*meanJ);
	cvmSet(T,2,0,0); cvmSet(T,2,1,0); cvmSet(T,2,2,1);
	
	// data normalization
	for(i = 0; i < numOfPositions; i++)
	{
		x = cvmGet(Data,i,0);
		y = cvmGet(Data,i,1);
		xx = t[0] * x + t[1] * y + t[2];
		yy = t[3] * x + t[4] * y + t[5];
		ww = t[6] * x + t[7] * y + t[8];
		
		xx = xx / ww;
		yy = yy / ww;
		//cvmSet(DataNorm,0,i,xx);
		//cvmSet(DataNorm,1,i,yy);
		cvmSet(DataNorm,i,0,xx);
		cvmSet(DataNorm,i,1,yy);
		
	}

}
//**************************************************************************************************

int ApplyHomograpy(CvMat* H1,CvMat* SourcePoint, CvMat* DestPoints)
{
		double x, y,z;
		double h11, h12, h13, h21, h22, h23, h31, h32, h33;		
		h11 = cvmGet( H1, 0, 0 );
		h12 = cvmGet( H1, 0, 1 );
		h13 = cvmGet( H1, 0, 2 );
		h21 = cvmGet( H1, 1, 0 );

		h22 = cvmGet( H1, 1, 1 );
		h23 = cvmGet( H1, 1, 2 );
		h31 = cvmGet( H1, 2, 0 );
		h32 = cvmGet( H1, 2, 1 );
		h33 = cvmGet( H1, 2, 2 );
		CvPoint2D32f Ximg1;
		for (int i=0; i<SourcePoint->height; i++)
		{
			Ximg1=cvPoint2D32f(cvmGet(SourcePoint,i,0),cvmGet(SourcePoint,i,1));
			z = 1/(h31*Ximg1.x+h32*Ximg1.y+h33);
			x = (h11*Ximg1.x+h12*Ximg1.y+h13)*z;
			y = (h21*Ximg1.x+h22*Ximg1.y+h23)*z;
			cvmSet(DestPoints,i,0,x);
			cvmSet(DestPoints,i,1,y);
		}
			
			
	
	return 1;
}



void print_cv_matrix(CvMat* mat_to_print)
{
    for (int row = 0; row < mat_to_print->rows; ++ row)
    {
        for (int col = 0; col < mat_to_print->cols; ++ col)
        {
            std::cout<< cvmGet(mat_to_print, row, col) << " ";
        }
        std::cout << std::endl;
    }
}

int drawMosaicFromHomographies(MOSAIC *mosaic, int channels, int start_imgnum, int max_imgnum, int last_computed, bool use_prev)
{
    printf("start_imgnum = %d, last_computed = %d \n", start_imgnum, last_computed);
    int interruptresize = 0;
    int img_num = 1;
    cvSetZero(mosaic->imgDoble);
    mosaic->finalsizex = 3;
    mosaic->finalsizey = 3;
    mosaic->levelx = 1;
    mosaic->levely = 1;
    cvSetZero(mosaic->imgDobleLast);
    char filename[200];
    int mosaic_num = std::min(start_imgnum - 1, last_computed);
    if (use_prev)
    {
        char cmdstart[256] = "ls mosaicos/ | grep ";
    //    int loop_start = 1;
        //check to see whether there is a stored homography starting at start_imgnum
        sprintf(filename, "mosrec%04d.tif", mosaic_num);
        strcat(cmdstart, filename);
    //    printf("output = %d", system(cmdstart));
        IplImage* start;

        if (system(cmdstart) == 0)
        {
            sprintf(filename, "mosaicos/mosrec%04d.tif", mosaic_num);
            start = cvLoadImage(filename);
            cvResize(mosaic->imgDoble,start, CV_INTER_AREA);
            cvCopy(start, mosaic->imgDoble);
            img_num = std::min(start_imgnum, last_computed + 1); // the first image is image number 1
        }
        else
        {
            cvSetZero(mosaic->imgDoble);
            img_num = 1;
        }
    }


    //start iterating through images
    //change image in whatever way you need (so it's the same as what img0 is)
    CvMat* new_Hall = cvCreateMat(3,3,CV_32FC1);
    CvMat* old_Hall = cvCreateMat(3,3,CV_32FC1);
    cvSetIdentity(new_Hall);
    cvSetIdentity(old_Hall);

    IplImage* img;

    //load next image and next homography
    for (int i = img_num; i < max_imgnum; i++)
    {
//        printf("processing img %d \n", i);
        sprintf(filename,"homogs/cumhom%04d.xml", i);
        new_Hall = (CvMat*)cvLoad(filename);
        sprintf(filename,"imagenes/mosaico%04d.tif", i);
        img = cvLoadImage(filename);

//        if (i == max_imgnum - 1 || i == max_imgnum - 2)
//        {
//            printf("i = %d, mat = \n", i);
//            print_cv_matrix(new_Hall);
//        }

        //call drawMosaic3 for every consecutive image
        drawMosaic3(mosaic, img, new_Hall, old_Hall, channels, 0, 0, interruptresize);

        //save old homographyimg_num
        cvCopy(new_Hall, old_Hall, NULL);

//        //save computed mosaic image
        sprintf(filename, "mosaicos/mosrec%04d.tif", i);
        cvSaveImage(filename, mosaic->imgDoble);
    }

//    sprintf(filename, "mosaicos/mosrec%04d.tif", max_imgnum - 1);
//    cvSaveImage(filename, mosaic->imgDoble);
    return 1;
}
