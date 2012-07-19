//#include <cv.h>
//#include <cxcore.h>
//#include <highgui.h>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <time.h>
#include "homography.h"
//#include "LVoptimization.h"
//#include "Levenberg_Marquartd_JC.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/video/video.hpp>
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

//Function to DrawtheMosaic based on the current total Homography 

int drawMosaic(IplImage* imgDoble, IplImage* imgDobleLast,IplImage* img0,CvMat* Hall,CvMat* Hold, int FirstImagePosx, int FirstImagePosy,int finalsizex,int finalsizey, int lastsizex,int lastsizey,int  channels, int q, int nofound, int interruptresize)
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

double x,y,z;

int i,j;


//int nofound=0, interruptresize=0;

IplImage* imgDobleCopy; 

//printf("Beginning Mosaic %d \n",q);


lastsizex=(int)(imgDoble->width/img0->width);
lastsizey=(int)(imgDoble->height/img0->height);
finalsizex=(int)(imgDoble->width/img0->width);
finalsizey=(int)(imgDoble->height/img0->height);;
FirstImagePosx=(imgDoble->width)/2-(img0->width)/2;
FirstImagePosy=(imgDoble->height)/2-(img0->height)/2;



for (i=0; i <img0->width; i++)
{
	for (j=0; j <img0->height; j++)
	{
               param_radial=((pow((i-(img0->width/2)),2)+pow((-j+(img0->height/2)),2))/(pow(img0->width/2,2)+pow(img0->height/2,2))); //Función de peso para los píxeles de la imagen antigua, en el centro es cero y el extremo 1
             //  param_radial=1/((1-(pow((i-(img0->width/2)),2)))*(1-(pow((j-(img0->height/2)),2))));
//             param_radial=((pow((i-(img0->width/2)),2)+pow((-j+(img0->height/2)),2))/pow(img0->height/2,2)); //Problema al generar fornteras, sólo pondera las partes centrales y los extremos se ven muy raros.
 //            if (param_radial>1) //con esto elimino las posibles deformaciones cuando da un parámetro negativo.
  //           {
  //               param_radial=1;
  //           }


		//printf("BUCLe  J incio %d i=%d, j=%d pos (%d,%d)  \n",q,i,j,FirstImagePosx,FirstImagePosy);
		z = 1/(h31*i+h32*j+h33);
//printf("BUCLe  J while z= %f\n",z);
		x = (FirstImagePosx+(h11*i+h12*j+h13)*z);
//printf("BUCLe  J while x= %f\n",x);
		y = (FirstImagePosy+(h21*i+h22*j+h23)*z);
		//printf("BUCLe  J while y= %f\n",y);

										
		if ((isinf(z)==1) || (isinf(x)==1) || (isinf(y)==1)) // used when z=inf or nan 
		{
			nofound++;
			//printf(" inf x %f, y %f, z %f \n",x,y,z);
			cvCopy(Hold,Hall,NULL);
                        interruptresize=1;
			break;
			
		}
	
		if ( (isnan(z)==1) || (isnan(x)==1) || (isnan(y)==1)) // used when z=inf or nan 
		{
			//printf(" NAN x %f, y %f, z %f \n",x,y,z);
			nofound++;
			cvCopy(Hold,Hall,NULL);
                        interruptresize=1;
			break;
			
						
		}
		//x=abs(((int)x)%(int)imgDoble->width);
		//y=abs(((int)y)%(int)imgDoble->height);
		//x=fabs(fmod(x,imgDoble->width));
		//y=fabs(fmod(y,imgDoble->height));
		//x=(fmod(x,imgDoble->width));
		//y=(fmod(y,imgDoble->height));	

		
		//if ((int(x)<0) || (int(x)>imgDoble->width) || (int(y)<0) || (int(y)>imgDoble->height))  //change image size X

		while((int(x)<0) || (int(x)>imgDoble->width) || (int(y)<0) || (int(y)>imgDoble->height) )  //change image size X
		{
			if ((int(x)<0) || (int(x)>imgDoble->width))
			{
				finalsizex++;
			

				//printf(" increasing image q %d xsize %d, x= %f y= %f, %d=(%d,%d) \n",q,finalsizex, x, y,q,i,j);
			}
			if ((int(y)<0) || (int(y)>imgDoble->height))
			{
				finalsizey++;	
				
				//printf(" increasing image q %d ysize %d, x= %f y= %f, %d=(%d,%d) \n",q,finalsizey, x, y,q,i,j);			
			}

			cvRectangle( imgDoble, cvPoint(0,0),cvPoint(162,122), CV_RGB(0,0,0),-1,8,NULL); //fill the up left zone black (erease last small image)
					
			imgDobleCopy =cvCreateImage( cvSize(finalsizex*img0->width,finalsizey*img0->height), 8, channels );
			
//printf("before copy 1 done   widht doblecopy (%d,%d), doble (%d,%d), ROI(%d,%d)\n" ,imgDobleCopy->width,imgDobleCopy->height,imgDoble->width,imgDoble->height,(int)round((imgDobleCopy->width-imgDoble->width)/2)+imgDoble->width, (int)round((imgDobleCopy->height-imgDoble->height)/2)+imgDoble->height);			
			cvSetImageROI(imgDobleCopy,cvRect(round((int)(imgDobleCopy->width-imgDoble->width)/2),(int)round((imgDobleCopy->height-imgDoble->height)/2),imgDoble->width,imgDoble->height));			
			cvCopy(imgDoble,imgDobleCopy,NULL);
			
			cvResetImageROI(imgDobleCopy);
			
			cvReleaseImage( &imgDoble );
			imgDoble= cvCreateImage(cvGetSize(imgDobleCopy),8,channels);

			cvCopy(imgDobleCopy,imgDoble,NULL);
		
			FirstImagePosx=(imgDoble->width)/2-(img0->width)/2;
			FirstImagePosy=(imgDoble->height)/2-(img0->height)/2;

		
			
			z = 1/(h31*i+h32*j+h33);
                        //printf("Finishing Mosaic %d \n",q);
                        x = (FirstImagePosx+(h11*i+h12*j+h13)*z);
			y = (FirstImagePosy+(h21*i+h22*j+h23)*z);
			
			
			cvReleaseImage( &imgDobleCopy );
			
			if ( ((finalsizex-lastsizex)>3)||((finalsizey-lastsizey)>3)           ) // used when z=inf or nan 
			{
				//printf("Too hihg image %d resizing Diff %d,  %d, Returnign to size (%d,%d) \n",q,(finalsizex-lastsizex),(finalsizey-lastsizey), lastsizex, lastsizey);
				nofound++;
				cvCopy(Hold,Hall,NULL);
				finalsizex=lastsizex;
				finalsizey=lastsizey;
				cvReleaseImage( &imgDoble );
                                //usleep(100);
				imgDoble=cvCloneImage(imgDobleLast);
				cvCopy(imgDobleLast,imgDoble);
				//printf("returning to Doble to size (%d %d)\n",imgDoble->width,imgDoble->height);
				finalsizex=(int)(imgDoble->width/img0->width);
				finalsizey=(int)(imgDoble->height/img0->height);;
				FirstImagePosx=(imgDoble->width)/2-(img0->width)/2;
				FirstImagePosy=(imgDoble->height)/2-(img0->height)/2;
				
				
			
				z = 1/(h31*i+h32*j+h33);
				x = (FirstImagePosx+(h11*i+h12*j+h13)*z);
				y = (FirstImagePosy+(h21*i+h22*j+h23)*z);
				interruptresize=1;
				break;
				
						
			}
			

		}// end while		
		if(interruptresize==1)
		{
                        printf("Breaking image  j resize %d \n",q);
			FirstImagePosx=(imgDoble->width)/2-(img0->width)/2;
			FirstImagePosy=(imgDoble->height)/2-(img0->height)/2;

			
							
                        break;
		}
		FirstImagePosx=(imgDoble->width)/2-(img0->width)/2;
		FirstImagePosy=(imgDoble->height)/2-(img0->height)/2;

			
	
		z = 1/(h31*i+h32*j+h33);
		x = (FirstImagePosx+(h11*i+h12*j+h13)*z);
		y = (FirstImagePosy+(h21*i+h22*j+h23)*z);
		if(
		(((uchar*)(imgDoble->imageData + (int)y*imgDoble->widthStep))[(int)x*imgDoble->nChannels + 0]==0)

		&&
 		(((uchar*)(imgDoble->imageData + (int)y*imgDoble->widthStep))[(int)x*imgDoble->nChannels + 1]==0) 
		&&
		(((uchar*)(imgDoble->imageData + (int)y*imgDoble->widthStep))[(int)x*imgDoble->nChannels + 2]==0)
		)
		{
                    printf("por aqui\n");
		
			((uchar*)(imgDoble->imageData + (int)y*imgDoble->widthStep))[(int)x*imgDoble->nChannels+0] = ((uchar*)(img0->imageData + (int)j*img0->widthStep))[(int)i*img0->nChannels + 0];
		
			if(imgDoble->nChannels==3)
				{
					((uchar*)(imgDoble->imageData + (int)y*imgDoble->widthStep))[(int)x*imgDoble->nChannels + 1]
					= ((uchar*)(img0->imageData + (int)j*img0->widthStep))[(int)i*img0->nChannels + 1];
		
					((uchar*)(imgDoble->imageData +(int)y*imgDoble->widthStep))[(int)x*imgDoble->nChannels + 2]
					= ((uchar*)(img0->imageData + (int)j*img0->widthStep))[(int)i*img0->nChannels + 2];
				}// end if imgDoble->nChannels==3)


		}
		else
		{
                    printf("parametro radial %f\n",param_radial);

			((uchar*)(imgDoble->imageData + (int)y*imgDoble->widthStep))[(int)x*imgDoble->nChannels+0] 
                        =( ((uchar*)(img0->imageData + (int)j*img0->widthStep))[(int)i*img0->nChannels + 0]*(1-param_radial)+((uchar*)(imgDoble->imageData + (int)y*imgDoble->widthStep))[(int)x*imgDoble->nChannels+0]*param_radial )  ;
			if(imgDoble->nChannels==3)
				{
					((uchar*)(imgDoble->imageData + (int)y*imgDoble->widthStep))[(int)x*imgDoble->nChannels+1] 
                                        =( ((uchar*)(img0->imageData + (int)j*img0->widthStep))[(int)i*img0->nChannels + 1]*(1-param_radial)+((uchar*)(imgDoble->imageData + (int)y*imgDoble->widthStep))[(int)x*imgDoble->nChannels+1]*param_radial )  ;
					((uchar*)(imgDoble->imageData + (int)y*imgDoble->widthStep))[(int)x*imgDoble->nChannels+2] 	
                                        =( ((uchar*)(img0->imageData + (int)j*img0->widthStep))[(int)i*img0->nChannels + 2]*(1-param_radial)+((uchar*)(imgDoble->imageData + (int)y*imgDoble->widthStep))[(int)x*imgDoble->nChannels+2]*param_radial )  ;
                                     //   =( ((uchar*)(img0->imageData + (int)j*img0->widthStep))[(int)i*img0->nChannels + 2]*((uchar*)(imgDoble->imageData + (int)y*imgDoble->widthStep))[(int)x*imgDoble->nChannels+2])/2;
				}// end if imgDoble->nChannels==3)
			
		} // End Else 


	} // END for (j=0; j <img0->height; j++)
	
	


	if(interruptresize==1)
		{
			//printf("Breaking image i resize %q \n",q);
			interruptresize=0;				
			//break;
			return 0;
		}
}  // end for (i=0; i <img0->width; i++)
//printf(" Reprojecting DONE %d \n" );


//printf("Finishing Mosaic %d \n",q);


return 1;
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



            if ((isinf(z)==1) || (isinf(x)==1) || (isinf(y)==1)) // used when z=inf or nan
            {
                nofound++;
                //printf(" inf x %f, y %f, z %f \n",x,y,z);
                cvCopy(Hold,Hall,NULL);
                interruptresize=1;
                break;

            }

            if ( (isnan(z)==1) || (isnan(x)==1) || (isnan(y)==1)) // used when z=inf or nan
            {
                //printf(" NAN x %f, y %f, z %f \n",x,y,z);
                nofound++;
                cvCopy(Hold,Hall,NULL);
                interruptresize=1;
                break;
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


//******************************************************************************************************************************************
//*************              function for drawing mosaics priorizing the first image
//******************************************************************************************************************************************


int drawMosaic3Inv(MOSAIC *mosaic,IplImage* img0,CvMat* Hall,CvMat* Hold,int  channels, int q,int nofound, int interruptresize)
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

           // param_radial=1;
            //                z = 1/(h31*i+h32*j+h33);
            //                x = (mosaic->FirstImagePosx+(h11*i+h12*j+h13)*z);
            //                y = (mosaic->FirstImagePosy+(h21*i+h22*j+h23)*z);


            param_lineal=fabs((pow((i-(img0->width/1.2)),1)+pow((-j+(img0->height/1.2)),1))/(pow(img0->width/1.2,1)+pow(img0->height/1.2,1))); //Función de peso para los píxeles de la imagen antigua, en el centro es cero y el extremo 1


            z = 1/(h31*i+h32*j+h33);
            x = (int) mosaic->FirstImagePosx+((h11*i+h12*j+h13)*z)*3/mosaic->finalsizex;
            y = (int) mosaic->FirstImagePosy+((h21*i+h22*j+h23)*z)*3/mosaic->finalsizey;



            if ((isinf(z)==1) || (isinf(x)==1) || (isinf(y)==1)) // used when z=inf or nan
            {
                nofound++;
                //printf(" inf x %f, y %f, z %f \n",x,y,z);
                cvCopy(Hold,Hall,NULL);
                interruptresize=1;
                break;

            }

            if ( (isnan(z)==1) || (isnan(x)==1) || (isnan(y)==1)) // used when z=inf or nan
            {
                //printf(" NAN x %f, y %f, z %f \n",x,y,z);
                nofound++;
                cvCopy(Hold,Hall,NULL);
                interruptresize=1;
                break;
            }


            while((int(x)<0) || (int(x)>mosaic->imgDoble->width) || (int(y)<0) || (int(y)>mosaic->imgDoble->height) )  //change image size X
            {




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

            if (x>mosaic->imgDoble->width)
                x=mosaic->imgDoble->width;



            if (y>mosaic->imgDoble->height)
                y=mosaic->imgDoble->height;


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

// end drawmosiac3inv





//**************************************************************************************
bool MLHomographyEstimation(CvMat* seedHomography, CvMat* InliersImg1,CvMat* InliersImg2, CvMat* OptH)
{
//int k=0, kmax=100;
//int v=2;
//double a1,a2,a3,a4,a5,a6,a7,a8,a9;
//double min_J,max_J;
//int X1, Y1, X2, Y2, div;
/*cvMat* A;
cvMat* JtxJ;
CvMat* E;
CvMat g;
*/
//double e1=0.000000000000001, e2=0.000000000000001, tao=0.001;
/*
a1=cvmGet(seedHomography, i, 0);


CvMat* J=cvCreateMat(3,3*InliersImg1->height,CV_32F);
j=0;
for (i=0;i< InliersImg1->height;i++)
{
X1=cvmGet(InliersImg1, i, 0);
Y1=cvmGet(InliersImg1, i, 1);
div=a7*X1+a8*Y1+a9;
cvmSet(J,0,j,cvmGet(InliersImg1, 0, 0));
	

}
cvMinMaxLoc(J, min_J,max_J,NULL, NULL, NULL );

double u=tao*max_J;

*/

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


///////////////////////////////////////////////////////////////////////////////////////
// Linear Squared solution for a Euclidean projection matrix.
    CvMat* lsq_euclidean( CvPoint2D32f* pts, CvPoint2D32f* mpts, int n, CvMat* H )
    {

	CvMat  * A, * B, X, *Xfin;
	double x[3];
	int i;

	if( n < 2 )
	{
            fprintf( stderr, "Warning: too few points in lsq_homog(), %s line %d\n",
                     __FILE__, __LINE__ );
            return NULL;
	}

	/* set up matrices so we can unstack homography into X; AX = B */
	A = cvCreateMat( 2*n, 3, CV_32FC1 );
	B = cvCreateMat( 2*n, 1, CV_32FC1 );
	X = cvMat( 3, 1, CV_32FC1, x );
	Xfin = cvCreateMat( 3, 1, CV_32FC1);
	
	cvZero( A );


	for( i = 0; i < n; i++ )
	{
            cvmSet( A, i, 0, -pts[i].y );
            cvmSet( A, i, 1, 1);
            cvmSet( A, i, 2, 0);
            cvmSet( A, i+n, 0, pts[i].x );
            cvmSet( A, i+n, 1, 0);
            cvmSet( A, i+n, 2, 1);
            cvmSet( B, i, 0, mpts[i].x- pts[i].x);
            cvmSet( B, i+n, 0, mpts[i].y-pts[i].y );
	}


	cvSolve( A, B, &X, CV_SVD );

	X = cvMat( 3, 1, CV_32FC1, x );
	cvConvert( &X, Xfin );
		
	cvmSet( H, 0, 0, 1);
	cvmSet( H, 0, 1, -cvmGet(Xfin,0,0));
	cvmSet( H, 0, 2, cvmGet(Xfin,1,0));
	cvmSet( H, 1, 0, cvmGet(Xfin,0,0));
	cvmSet( H, 1, 1, 1);
	cvmSet( H, 1, 2, cvmGet(Xfin,2,0));
//	cvmSet( H, 2, 0,0.0);
//	cvmSet( H, 2, 1, 0.0);
//	cvmSet( H, 2, 2, 1.0);
	cvReleaseMat( &A );
	cvReleaseMat( &B );
	cvReleaseMat( &Xfin );

	return H;
    }


////////////////////


// Linear Squared solution for a Euclidean projection matrix.
    CvMat* lsq_affine( CvPoint2D32f* pts, CvPoint2D32f* mpts, int n, CvMat* H )
    {
	CvMat  * A, * B, X, *Xfin;
	double x[6];
	int i;

	if( n < 3 )
	{
            fprintf( stderr, "Warning: too few points in lsq_homog(), %s line %d\n",
                     __FILE__, __LINE__ );
            return NULL;
	}

	/* set up matrices so we can unstack homography into X; AX = B */
	A = cvCreateMat( 2*n, 6, CV_32FC1 );
	B = cvCreateMat( 2*n, 1, CV_32FC1 );
	X = cvMat( 6, 1, CV_32FC1, x );
	Xfin = cvCreateMat( 6, 1, CV_32FC1);
	
	cvZero( A );


	for( i = 0; i < n; i++ )
	{
            cvmSet( A, i, 0, pts[i].x );
            cvmSet( A, i, 1, pts[i].y);
            cvmSet( A, i, 2, 0);
	    cvmSet( A, i, 3, 0 );
            cvmSet( A, i, 4, 1.0);
	    cvmSet( A, i, 5, 0.0);
            cvmSet( A, i, 2, 0);
            cvmSet( A, i+n, 0, 0 );
            cvmSet( A, i+n, 1, 0);
            cvmSet( A, i+n, 2, pts[i].x);
	    cvmSet( A, i+n, 3, pts[i].y);
	    cvmSet( A, i+n, 4, 0.0);
	    cvmSet( A, i+n, 5, 1.0);	 
            cvmSet( B, i, 0, mpts[i].x);
            cvmSet( B, i+n, 0, mpts[i].y);
	}


	cvSolve( A, B, &X, CV_SVD );
	
	X = cvMat( 6, 1, CV_32FC1, x );
	cvConvert( &X, Xfin );
	
	cvmSet( H, 0, 0, cvmGet(Xfin,0,0));
	cvmSet( H, 0, 1, cvmGet(Xfin,1,0));
	cvmSet( H, 0, 2, cvmGet(Xfin,4,0));
	cvmSet( H, 1, 0, cvmGet(Xfin,2,0));
	cvmSet( H, 1, 1, cvmGet(Xfin,3,0));
	cvmSet( H, 1, 2, cvmGet(Xfin,5,0));
//	cvmSet( H, 2, 0,0.0);
//	cvmSet( H, 2, 1, 0.0);
//	cvmSet( H, 2, 2, 1.0);
	cvReleaseMat( &A );
	cvReleaseMat( &B );
	cvReleaseMat( &Xfin );
	return H;
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Ransac bundle adjustment according to the selected model. (Affine )
    bool RANSACadjust(CvPoint2D32f* cornersA, CvPoint2D32f* cornersB,CvMat* MATwarping, int totalMatched, int model)
    {
	//RANSAC VARIABLES ACCORDIGN TO ZISSERMAN PAGE 118
	double standardesv =2.0;//standar desviation in measured error pixels
	double  p=0.99999; //probability that at least one of the ramdodons samples of s is free from outliers
        int sample_count=0;
        //int concensus;  //notation zisserman Page 118
	double e=0.5;  //e=1-w Probability that any selected point is an outlier initially e=0.5 worst stimated for begin
	int N; //N selections or test (each of s points) requiered
	int s;// number of point necesarry to made the test
	double T; // Threshold T for min concesus to finish the test
	double t =sqrt(5.99*pow(standardesv,2)); //d² =5.99*standardesv² distance threshold to consider a point inlier
	int recalculateH=0; //Recalculate Projection with only the inliers points.
	//End RANSAC VARIABLES	
	int chaos, correspond_total = 0;
	CvRNG rng = cvRNG(chaos);
        double x, y, d;
	double best_d = 0xffffff;
	int best_clan = 0;
	int clan = 0;
        //int HRANSACfound=0;
	clock_t init_bucle;
	double tmedio=0;
	
	CvMat* MATwarpingR = cvCloneMat(MATwarping);

	if(model ==0) {	s=2;}
	if(model ==1) {	s=3;}
	CvPoint2D32f pointImgA[4];	
	CvPoint2D32f pointImgB[4];
	CvPoint2D32f pointImgA2[totalMatched];	
	CvPoint2D32f pointImgB2[totalMatched];

	correspond_total=totalMatched;
	if ( correspond_total < s ){return 0;}

	

	N=(int) log(1-p)/log(1-pow((1-e),s));
	T=(1-e)*correspond_total; // correspond_n =total  number of matched keypoints
	sample_count=0;
	//best_clan= (int)T;
	t= sqrt(5.99*pow(standardesv,2));

	
	init_bucle =clock();

	
	while((int)N >(int)sample_count )
	{	
            int selector[s];
            for( int i = 0; i < s; i++ )
            {
                while ( selector[i] = (cvRandInt(&rng)%(correspond_total)))
                {
                    bool recalc = false;
                    for ( int j = 0; j < i; j++ )
                        if ( selector[i] == selector[j] )
                            recalc = true;
                    if ( !recalc )
                        break;
                }

                pointImgA[i]=cvPoint2D32f(cornersA[selector[i]].x,cornersA[selector[i]].y);
                pointImgB[i]=cvPoint2D32f(cornersB[selector[i]].x,cornersB[selector[i]].y);



            } // end for( int i = 0; i < s; i++ )
		
	    if(model ==0) {lsq_euclidean( pointImgA,pointImgB, s,MATwarpingR );}
	    if(model ==1) {lsq_affine( pointImgA,pointImgB, s,MATwarpingR );}


            //cvGetAffineTransform(pointImgB,pointImgA, MATwarpingR);
            d = 0;
            clan = 0;
            int j=0;
            for ( int i = 0; i < correspond_total; i++ )
            {
                x=cvmGet(MATwarpingR,0,0)*cornersA[i].x+cvmGet(MATwarpingR,0,1)*cornersA[i].y+cvmGet(MATwarpingR,0,2)-cornersB[i].x;
                y=cvmGet(MATwarpingR,1,0)*cornersA[i].x+cvmGet(MATwarpingR,1,1)*cornersA[i].y+cvmGet(MATwarpingR,1,2)-cornersB[i].y;
                if ( x*x+y*y < t*t )
                {
                    d=d+(x*x+y*y); //standar desvitation on concensus distance.
                    clan++;
                    pointImgA2[j]=cvPoint2D32f(cornersA[i].x,cornersA[i].y);
                    pointImgB2[j]=cvPoint2D32f(cornersB[i].x,cornersB[i].y);

                    j++;
                }

            }// end for ( int i = 0; i < correspond_total; i++ )

            if ( ( clan >= round( best_clan) )&&(sqrt(d/clan)< best_d*1.1 )&&( clan >= s ))
            {
                //printf("Affine candidate found   \n");
                cvCopy(MATwarpingR,MATwarping,NULL);
                if ((clan>=s)&&(recalculateH==1))
                {

                    CvPoint2D32f pointImgA3[j];
                    CvPoint2D32f pointImgB3[j];
                    for (int i=0; i<j; i++)
                    {
                        pointImgA3[i]=cvPoint2D32f(pointImgA2[i].x,pointImgA2[i].y);
                        pointImgB3[i]=cvPoint2D32f(pointImgB2[i].x,pointImgB2[i].y);

                    }
		     	if(model ==0) {lsq_euclidean( pointImgA3,pointImgB3, j,MATwarpingR );}
	   		if(model ==1) {lsq_affine( pointImgA3,pointImgB3, j,MATwarpingR );}
              
                    //cvGetAffineTransform(pointImgB3,pointImgA3, MATwarping);



                }


                best_d= sqrt(d/clan);
                best_clan = (int)clan;

                e=1.0-(double)clan/(double)correspond_total;
                N=(int) log(1-p)/log(1-pow((1-e),s));


            }//end if ( ( clan >= best_clan )&&(sqrt(d/clan)< best_d ) )
            sample_count++;
            tmedio=(clock()-init_bucle)/(double)CLOCKS_PER_SEC;
            if (tmedio>0.5)
            {


                cvReleaseMat( &MATwarpingR );
                cvSetIdentity(MATwarping);
                return 0;
            }
	} //end while((int)N >(int)sample_count )
	
	if ( best_clan < s )
	{

            cvSetIdentity(MATwarping);
            cvReleaseMat( &MATwarpingR );
            return 0;


	}

	cvReleaseMat( &MATwarpingR );
	return 1;
    }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function that allow selet between an homography, a projective space or a Affine (Euclidean) model.
    int CalHomography(CvMat* PointsImg1,CvMat* PointsImg2, CvMat* Homography,int KindProjection)
    {
	
	CvPoint2D32f pointImgA[PointsImg1->rows];	
	CvPoint2D32f pointImgB[PointsImg1->rows];
	CvMat* Affine = cvCreateMat(2, 3, CV_32FC1);
        //double standardesv =2;//standar desviation in measured error pixels
      //  double t =sqrt(5.99*pow(standardesv,2));
        double t=1;
	
	
	switch (KindProjection)
	{
            //Notes; Affine is a case of a general Homography. Homography iw is a mapping  points  on one
            //surface to points on another surface. So if mapping in the same space (image space)
            //a Perspective projectionrelates a  3D space to 2D using a single projection center (Camera model. So it is not recomended to use.
            // A perspective transformation is a specific kind of homography //See OpenCv book page 407.
        case 1: //Homgraphy
            cvFindHomography(PointsImg1,PointsImg2,Homography,CV_RANSAC, t,NULL);
            break;
        case 2: //Affine projection
              for (int i=0; i< PointsImg1->rows; i++)
            {
                pointImgA[i]=cvPoint2D32f(cvmGet(PointsImg1,i,0),cvmGet(PointsImg1,i,1));
                pointImgB[i]=cvPoint2D32f(cvmGet(PointsImg2,i,0),cvmGet(PointsImg2,i,1));
            }

            RANSACadjust(pointImgA, pointImgB,Affine, PointsImg1->rows,1);

            //cvGetAffineTransform(pointImgA,pointImgB,Affine);
            for (int i=0; i<Affine->rows; i++)
            {
                for (int j=0; j< Affine->cols; j++)
                {
                    cvmSet(Homography,i,j,cvmGet(Affine,i,j));
                }
            }
            cvmSet(Homography,2,0,0);
            cvmSet(Homography,2,1,0);
            cvmSet(Homography,2,2,1);

            
            break;
	case 3: //Euclidean
            for (int i=0; i< PointsImg1->rows; i++)
            {
                pointImgA[i]=cvPoint2D32f(cvmGet(PointsImg1,i,0),cvmGet(PointsImg1,i,1));
                pointImgB[i]=cvPoint2D32f(cvmGet(PointsImg2,i,0),cvmGet(PointsImg2,i,1));
            }

            RANSACadjust(pointImgA, pointImgB,Affine, PointsImg1->rows, 0);

            //cvGetAffineTransform(pointImgA,pointImgB,Affine);
            for (int i=0; i<Affine->rows; i++)
            {
                for (int j=0; j< Affine->cols; j++)
                {
                    cvmSet(Homography,i,j,cvmGet(Affine,i,j));
                }
            }
            cvmSet(Homography,2,0,0);
            cvmSet(Homography,2,1,0);
            cvmSet(Homography,2,2,1);

            break;
        }
	cvReleaseMat(&Affine);
        return 1;
    }


//**************************************************************************************************
int CalHomographyNorm(CvMat* PointsImg1,CvMat* PointsImg2, CvMat* Homography)
{
	CvMat* T1;
	CvMat* T2;
	CvMat* Hnorm;
	CvMat* PointsImg1Norm;
	CvMat* PointsImg2Norm;
	CvMat* invT2; CvMat* temp;
	T1 = cvCreateMat(3, 3, CV_32FC1);
	T2 = cvCreateMat(3, 3, CV_32FC1);
	Hnorm = cvCreateMat(3, 3, CV_32FC1);
	invT2 = cvCreateMat(3, 3, CV_32FC1);
	temp = cvCreateMat(3, 3, CV_32FC1);


	//PointsImg1Norm = cvCreateMat(PointsImg1->width, PointsImg1->height, CV_32FC1);
	//PointsImg2Norm = cvCreateMat(PointsImg2->width, PointsImg2->height, CV_32FC1);
	PointsImg1Norm = cvCreateMat(PointsImg1->height, PointsImg1->width, CV_32FC1);
	PointsImg2Norm = cvCreateMat(PointsImg2->height, PointsImg2->width, CV_32FC1);

	DataNormalization(PointsImg1,PointsImg1Norm, T1);
	DataNormalization(PointsImg2,PointsImg2Norm, T2);

	cvFindHomography(PointsImg1Norm,PointsImg2Norm,Hnorm);
	//CalHomography(PointsImg1Norm,PointsImg2Norm,Hnorm,1);
	

	// denormalization : H = invT2 * Htmp * T1 <- Htmp = T2 * H * invT1
	
	cvInvert(T2, invT2);
	cvMatMul(invT2, Hnorm, temp);
	//cvGEMM(T2,Hnorm,1,NULL,0,temp,CV_GEMM_A_T);
	cvMatMul(temp,T1, Homography);	
	
	cvReleaseMat(&T1); 
	cvReleaseMat(&T2);
	cvReleaseMat(&Hnorm);
	cvReleaseMat(&PointsImg1Norm);
	cvReleaseMat(&PointsImg2Norm);
	cvReleaseMat(&invT2); 
	cvReleaseMat(&temp);
	
	
return 1;
}


 

bool CalHomographyRANSAC_opencv2(CvMat* PointsImg1, CvMat* PointsImg2, CvMat* Homography, CvMat* InliersMask)
{
double standardesv =2;//standar desviation in measured error pixels
double t =sqrt(5.99*pow(standardesv,2));

//cvFindHomography(PointsImg1,PointsImg2,Homography,CV_RANSAC, t,InliersMask);	
//cvFindHomography(PointsImg1,PointsImg2,Homography,CV_RANSAC, t,NULL);
cvFindHomography(PointsImg1,PointsImg2,Homography,CV_LMEDS, t,NULL);
return 1;
}
//****************************************************************************
bool CalHomographyRANSAC(CvMat* PointsImg1, CvMat* PointsImg2, CvMat* Homography, CvPoint* FramePoints)
{
	//double alpha=0.95; //probability that one poins is an inlier
	double standardesv =2;//standar desviation in measured error pixels
	double  p=0.99999; //probability that at least one of the ramdodons samples of s is free from outliers
        int sample_count=0;
        //int concensus;  //notation zisserman Page 118
	//double w=0; // w= Probability that any selected point is an inlier
double e=0.5;  //e=1-w Probability that any selected point is an outlier initially e=0.5 worst stimated for begin
	//double e=0.95;  //e=1-w Probability that any selected point is an outlier initially e=0.5 worst stimated for begin
	int N; //N selections or test (each of s points) requiered
	int s;// number of point necesarry to made the test
	double T; // Threshold T for min concesus to finish the test
	double t =sqrt(5.99*pow(standardesv,2)); //d² =5.99*standardesv² distance threshold to consider a point inlier
	int chaos;
	int clan = 0;
	int best_clan = 0;
	double h11, h12, h13, h21, h22, h23, h31, h32, h33;
	double h11inv, h12inv, h13inv, h21inv, h22inv, h23inv, h31inv, h32inv, h33inv;
	double x, y, z, d;
	double x2, y2, z2;
	int correspond_n = PointsImg1->height;
	double best_d = 0xffffff;
	chaos =(int)rand();
	CvRNG rng = cvRNG(chaos);  //
	int HRANSACfound=0;
	clock_t init_bucle;
	double tmedio=0;
	int recalculateH=1;
        //int nextsample=0;

	//

//printf(" \nBeginning RANSAC Total Points = %d \n", PointsImg1->height);
	

	if ( correspond_n < 4 )
	{
		printf ("\n Returning 0 Ransac n<4");
		return 0;
	}

	CvMat* correspond_points1;
	CvMat* correspond_points2;
	CvMat* correspond_points3;
	CvMat* correspond_points4;
	CvMat* correspond_points5;
	CvMat* correspond_points6;
	CvMat* H1;
	CvMat* InvH;

	correspond_points1 = cvCreateMat( 4, 2, CV_32FC1 );
	correspond_points2 = cvCreateMat( 4, 2, CV_32FC1 );
	correspond_points3 = cvCreateMat( correspond_n, 2, CV_32FC1 );
	correspond_points4 = cvCreateMat( correspond_n, 2, CV_32FC1 );


	H1 = cvCreateMat( 3, 3, CV_32FC1 );
	InvH = cvCreateMat( 3, 3, CV_32FC1 );
	CvPoint2D32f Ximg1;
	CvPoint2D32f Ximg2;

	s=4;
	N=(int) log(1-p)/log(1-pow((1-e),s));
	T=(1-e)*correspond_n; // correspond_n =total  number of matched keypoints
	sample_count=0;
	best_clan= (int)(T);
	//best_clan=4;
	//printf("RANSAC:  T= %f  bestclan %d \n",T, best_clan );
	
	t= sqrt(5.99*pow(standardesv,2));
	

        int contadorwhile=0;
	//printf(" Before while \n");	
	init_bucle =clock();
	//printf(" Iniciando N %d best_clan %d \n", N,best_clan );
	while((int)N >(int)sample_count )
	{
		
		contadorwhile++;	
		int selector[s];
		for( int i = 0; i < s; i++ )
		{
		
                        while ( selector[i] = cvRandInt(&rng)%(correspond_n) )
			{
				
				bool recalc = false;
				for ( int j = 0; j < i; j++ )
					if ( selector[i] == selector[j] )
						recalc = true;
				if ( !recalc )
					break;
			}
			
			cvmSet(correspond_points1,i,0,cvmGet(PointsImg1,selector[i],0 ));
			cvmSet(correspond_points1,i,1,cvmGet(PointsImg1,selector[i],1 ));
			cvmSet(correspond_points2,i,0,cvmGet(PointsImg2,selector[i],0 ));
			cvmSet(correspond_points2,i,1,cvmGet(PointsImg2,selector[i],1 ));
						
		}// end for i

		/// INGRESAR EL CHEQUEO DE COLINEALIDAD DE LOS PUNTOS

		int collinear=0;
		for( int i = 0; i < 4; i++ )
		{
			 	/* double vx1 = cvmGet(PointsImg1,(i+1)%4,0)-cvmGet(PointsImg1,(i)%4,0 );
				double vy1 = cvmGet(PointsImg1,(i+1)%4,1)-cvmGet(PointsImg1,(i)%4,1 );
				double vx2 = cvmGet(PointsImg1,(i+2)%4,0)-cvmGet(PointsImg1,(i+1)%4,0 );
				double vy2 = cvmGet(PointsImg1,(i+2)%4,1)-cvmGet(PointsImg1,(i+1)%4,1 );
				*/
				 double vx1 = cvmGet(correspond_points1,(i+1)%4,0)-cvmGet(correspond_points1,(i)%4,0 );
				double vy1 = cvmGet(correspond_points1,(i+1)%4,1)-cvmGet(correspond_points1,(i)%4,1 );
				double vx2 = cvmGet(correspond_points1,(i+2)%4,0)-cvmGet(correspond_points1,(i+1)%4,0 );
				double vy2 = cvmGet(correspond_points1,(i+2)%4,1)-cvmGet(correspond_points1,(i+1)%4,1 );
				if(abs(vx2*vy1-vx1*vy2) <0.5)
				{
					collinear++;
				}
				
		}		

		if(collinear>=3)
		{
			printf(" Points are collinear  ciclo while %d N= %d \n", (int)contadorwhile, (int)N);
			//printf(" p1 (%d, %d ) p2 (%d, %d ) p3 (%d, %d ) p4 (%d, %d ) \n", (int)cvmGet(correspond_points1,0,0), (int)cvmGet(correspond_points1,0,1), (int)cvmGet(correspond_points1,1,0), (int)cvmGet(correspond_points1,1,1),(int)cvmGet(correspond_points1,2,0),(int)cvmGet(correspond_points1,2,1),(int)cvmGet(correspond_points1,3,0),(int)cvmGet(correspond_points1,3,1) );
			//cvWaitKey(0);
			//break;
			continue;
		}
		// Estimate the homography using a normalization 
		CalHomographyNorm(correspond_points1,correspond_points2, H1);
		
		// Estimates the homography without normalization...
		//CalHomography(correspond_points1,correspond_points2, H1,1);
		
		

		h11 = cvmGet( H1, 0, 0 );
		h12 = cvmGet( H1, 0, 1 );
		h13 = cvmGet( H1, 0, 2 );
		h21 = cvmGet( H1, 1, 0 );
		h22 = cvmGet( H1, 1, 1 );
		h23 = cvmGet( H1, 1, 2 );
		h31 = cvmGet( H1, 2, 0 );
		h32 = cvmGet( H1, 2, 1 );
		h33 = cvmGet( H1, 2, 2 );
		
		cvInvert(H1,InvH,CV_LU);

		h11inv = cvmGet( InvH, 0, 0 );
		h12inv = cvmGet( InvH, 0, 1 );
		h13inv = cvmGet( InvH, 0, 2 );
		h21inv = cvmGet( InvH, 1, 0 );
		h22inv = cvmGet( InvH, 1, 1 );
		h23inv = cvmGet( InvH, 1, 2 );
		h31inv = cvmGet( InvH, 2, 0 );
		h32inv = cvmGet( InvH, 2, 1 );
		h33inv = cvmGet( InvH, 2, 2 );
		

		d = 0;
		clan = 0;
		int j=0;
		
		for ( int i = 0; i < correspond_n; i++ )
		{
					
			Ximg1=cvPoint2D32f(cvmGet(PointsImg1,i,0),cvmGet(PointsImg1,i,1));
			Ximg2=cvPoint2D32f(cvmGet(PointsImg2,i,0),cvmGet(PointsImg2,i,1));

		
			/// x, y, x2, y2 Is the distance of point and pxojecto X'=HX-x'
			z = 1/(h31*Ximg1.x+h32*Ximg1.y+h33);
			x = (h11*Ximg1.x+h12*Ximg1.y+h13)*z-Ximg2.x;  //Hx-x'
			y = (h21*Ximg1.x+h22*Ximg1.y+h23)*z-Ximg2.y; 
			
			z2 = 1/(h31inv*Ximg2.x+h32inv*Ximg2.y+h33inv);
			x2 = (h11inv*Ximg2.x+h12inv*Ximg2.y+h13inv)*z2-Ximg1.x; //x'-Inv(H)x
			y2 = (h21inv*Ximg2.x+h22inv*Ximg2.y+h23inv)*z2-Ximg1.y;


			if ((pow(x,2)+pow(y,2)+pow(x2,2)+pow(y2,2))<t*t) //d²<t² ?
			{
				d=d+(pow(x,2)+pow(y,2)+pow(x2,2)+pow(y2,2));
				clan++;
				cvmSet(correspond_points3,j,0,cvmGet(PointsImg1,i,0 ));
				cvmSet(correspond_points3,j,1,cvmGet(PointsImg1,i,1 ));
				cvmSet(correspond_points4,j,0,cvmGet(PointsImg2,i,0 ));
				cvmSet(correspond_points4,j,1,cvmGet(PointsImg2,i,1 ));	
			
				j++;
			}
						//printf("\n Ximg1(%f,%f), Ximg2(%f,%f), d %f \n",Ximg1.x,Ximg1.y,Ximg2.x,Ximg2.y,(pow(x,2)+pow(y,2)+pow(x2,2)+pow(y2,2)));

		}
		//printf ("\n Best Clan = %d, best_d = %f, clan =%d, d= %f, (d/clan) =%f, j=%d \n", best_clan, best_d, clan, d,(d/clan), j);

	//if ((((int)(10000000*d/clan)==0)&&(j>0)&&(clan >=best_clan )&&(clan >=4 ))||(clan >= best_clan )&&((d/clan)< best_d*1.2 )&&(j>0)&&(clan >=4 ))// best consensus and less standar desviation (error) on concesus
		//if ( ( clan > best_clan ))
		if ( ( clan >= round( best_clan) )&&(sqrt(d/clan)< best_d*1.1 )&&( clan >= 4 ))	
		{
			HRANSACfound++;
			
			if ((clan>=4))
			{	
				//printf("\n RECALCULATING HOMOGRAPHY *************\n");
				
				correspond_points5 = cvCreateMat(j, 2, CV_32FC1 );
				correspond_points6 = cvCreateMat(j, 2, CV_32FC1 );
				
				for (int i=0; i<j; i++)
				{
					cvmSet(correspond_points5,i,0,cvmGet(correspond_points3,i,0 ));
					cvmSet(correspond_points5,i,1,cvmGet(correspond_points3,i,1 ));
					cvmSet(correspond_points6,i,0,cvmGet(correspond_points4,i,0 ));
					cvmSet(correspond_points6,i,1,cvmGet(correspond_points4,i,1 ));	
				}
								
				
				//CalHomographyNorm(correspond_points5,correspond_points6, H1);
				
				// Estimates the homography without normalization...
				if(recalculateH==1)
				{	
					CalHomographyNorm(correspond_points5,correspond_points6, H1);		
					//CalHomography(correspond_points5,correspond_points6, H1,1);
				 //	printf("\n RECALCULATING HOMOGRAPHY 2  HFOUND %d \n", HRANSACfound);
				}
				else
				{
				//	printf("\n WITHOUT RECALCULATE  HOMOGRAPHY 2  HFOUND %d \n", HRANSACfound);
				}
				
				h11 = cvmGet( H1, 0, 0 );
				h12 = cvmGet( H1, 0, 1 );
				h13 = cvmGet( H1, 0, 2 );
				h21 = cvmGet( H1, 1, 0 );
				h22 = cvmGet( H1, 1, 1 );
				h23 = cvmGet( H1, 1, 2 );
				h31 = cvmGet( H1, 2, 0 );
				h32 = cvmGet( H1, 2, 1 );
				h33 = cvmGet( H1, 2, 2 );


				printf("Homography Before LV \n [%f %f %f] \n [%f %f %f] \n [%f %f %f] \n", h11, h12, h13, h21,h22,h23,h31,h32,h33); 

/*				if (Levenberg_Marquartd_JC(H1,correspond_points5, correspond_points6)==1)
				//if (LVHomography(H1,correspond_points5, correspond_points6)==1)
				{
					printf("\n Levenber Marquad Optimization Done  \n");
					h11 = cvmGet( H1, 0, 0 );
				h12 = cvmGet( H1, 0, 1 );
				h13 = cvmGet( H1, 0, 2 );
				h21 = cvmGet( H1, 1, 0 );
				h22 = cvmGet( H1, 1, 1 );
				h23 = cvmGet( H1, 1, 2 );
				h31 = cvmGet( H1, 2, 0 );
				h32 = cvmGet( H1, 2, 1 );
				h33 = cvmGet( H1, 2, 2 );


				printf("Homography After LV \n [%f %f %f] \n [%f %f %f] \n [%f %f %f] \n", h11, h12, h13, h21,h22,h23,h31,h32,h33);
					
				}
				else
				{
					//printf("\n Levenber Marquad Optimization NOT DONE \n");
//					cvReleaseMat( &correspond_points5 );
//					cvReleaseMat( &correspond_points6 );
				}
                                */
				cvReleaseMat( &correspond_points5 );
					cvReleaseMat( &correspond_points6 );
				
			}	
			
			
			
			cvCopy(H1,Homography, NULL);
			/*for ( int i = 0; i < 4; i++ )
			{
				z = 1./(points[i].x*h31+points[i].y*h32+h33);
				corner[i] = cvPoint( (int)((points[i].x*h11+points[i].y*h12+h13)*z),
						     (int)((points[i].x*h21+points[i].y*h22+h23)*z) );
			}
			*/
			best_d =(d/clan);
			best_clan = (int)clan;
			e=1.0-(double)clan/(double)correspond_n;
			N=(int) log(1-p)/log(1-pow((1-e),s));
		}
		sample_count++;
		tmedio=(clock()-init_bucle)/(double)CLOCKS_PER_SEC;
		//printf(" N %d best_clan %d \n", N,best_clan );
		//printf ("\n Time since Ransac Beging %f \n",tmedio );
		if (tmedio>0.5)
		{
			
			printf ("\n Returning 0 Ransac Time Limit \n");
			//printf ("\n Returning 0 Ransac Time Limit \n");
			cvReleaseMat( &H1 );
			cvReleaseMat( &InvH );
			cvReleaseMat( &correspond_points1 );
			cvReleaseMat( &correspond_points2 );
			cvReleaseMat( &correspond_points3 );
			cvReleaseMat( &correspond_points4 );
			return 0;
		}

	}

	if ( HRANSACfound==0 )
	{
		cvSet(Homography,cvRealScalar(0), NULL);
		printf ("\n Returning 0 Ransac Not found");
		cvReleaseMat( &H1 );
		cvReleaseMat( &InvH );
		cvReleaseMat( &correspond_points1 );
		cvReleaseMat( &correspond_points2 );
		cvReleaseMat( &correspond_points3 );
		cvReleaseMat( &correspond_points4 );	
		return 0;
	}
	if ( best_clan < 4 )
	{
		cvSet(Homography,cvRealScalar(0), NULL);
		printf ("\n Returning 0 Ransac Best Clan<4");
		cvReleaseMat( &H1 );
		cvReleaseMat( &InvH );
		cvReleaseMat( &correspond_points1 );
		cvReleaseMat( &correspond_points2 );
		cvReleaseMat( &correspond_points3 );
		cvReleaseMat( &correspond_points4 );	
		return 0;
	}

	//printf ("\n Releasing matrix RANSAC \n");
	cvReleaseMat( &H1 );
	cvReleaseMat( &InvH );
	cvReleaseMat( &correspond_points1 );
	cvReleaseMat( &correspond_points2 );
	cvReleaseMat( &correspond_points3 );
	cvReleaseMat( &correspond_points4 );
	//cvReleaseMat( &correspond_points5 );
	//cvReleaseMat( &correspond_points6 );	


//printf ("\n Returning 1 Ransac");		
return 1;

}
//************************

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
