
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <thread_mosaico.h>

#include <qfiledialog.h>
#include <QImage>
#include <QMessageBox>
#include <QLabel>
#include <QIcon>
#include <QDockWidget>
#include <QListWidget>
#include <QGroupBox>
#include <QString>
#include <QPainter>


//#include <cxcore.h>
//#include <cv.h>
//#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/video/video.hpp>
#include "opencv2/video/tracking.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <string.h>
#include <cstring>
#include <cmath>


#include <validate_user.h>
#include <calibration.h>
#include <load_calibration_data.h>
#include <videoname.h>
//#include <blackmagic/bmcapture.h>

using namespace cv;
using namespace std;

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow)
{
    //***************************************************************************************
    //                          Zona de inicialización
    //**************************************************************************************

    system(" rm imagenes/*.* ");
    system(" rm mosaicos/*.* ");
    system(" rm homogs/*.* ");
    system(" rm corners/*.* ");

    // videoName=NULL;

    ui->setupUi(this);


    createActions();
    createStatusBar();
    ui->groupBox_2->setHidden(true);
    ui->frame->setHidden(true);
    ui->frame_13->setHidden(true);
    ui->frame_4->setHidden(true);
    ui->frame_3->setHidden(true);
    ui->frame_2->setHidden(false);
    ui->frame_6->setHidden(true);
    ui->frame_7->setHidden(true);
    ui->frameImageControl->setHidden(true);
    ui->frameVideoControl->setHidden(true);

    ui->playButton->setDisabled(true);
    ui->pauseButton->setDisabled(true);
    ui->stopButton->setDisabled(true);

    ui->actionHomography->setChecked(true);
    ui->actionAffine->setChecked(false);
    ui->actionEuclidean->setChecked(false);
    //ui->dial->setValue(50);
    ui->Black_reset->setValue(50);
    ui->Marca_1_button->setDisabled(true);
    ui->Marca_2_button->setDisabled(true);
    ui->Marca_3_button->setDisabled(true);
    ui->Marca_4_button->setDisabled(true);
    ui->Marca_5_button->setDisabled(true);
    ui->Marca_6->setDisabled(true);
    ui->Marca_7->setDisabled(true);
    ui->Marca_8->setDisabled(true);
    ui->Marca_9->setDisabled(true);
    ui->Marca_10->setDisabled(true);
    ui->Intro_marca->setDisabled(true);
    ui->actionFirewire->setChecked(true);
    ui->Black_reset->setHidden(true);
    ui->actionInterlaced->setChecked(true);;

    ui->Process_button->setDisabled(true);

    uninterlaced=1;

    image_mark_matrix[0][0]=0;
    image_mark_matrix[1][0]=0;
    image_mark_matrix[2][0]=0;
    image_mark_matrix[3][0]=0;
    image_mark_matrix[4][0]=0;
    image_mark_matrix[5][0]=0;
    image_mark_matrix[6][0]=0;
    image_mark_matrix[7][0]=0;
    image_mark_matrix[8][0]=0;
    image_mark_matrix[9][0]=0;
    mosaic_limit_matrix[0][0]=0;
    mosaic_limit_matrix[1][0]=0;
    mosaic_limit_matrix[2][0]=0;
    mosaic_limit_matrix[3][0]=0;
    mosaic_limit_matrix[4][0]=0;
    mosaic_limit_matrix[5][0]=0;
    mosaic_limit_matrix[6][0]=0;
    mosaic_limit_matrix[7][0]=0;
    mosaic_limit_matrix[8][0]=0;
    mosaic_limit_matrix[9][0]=0;

    ui->menuProjection_Model->setEnabled(false);
    ui->menuCamera->setEnabled(false);

    intrinsics_parameters_file="default_Intrinsics.xml";
    distortion_parameters_file="default_Distortion.xml";

    viewType=0;
    BlackResetRatio=50;

    mark_index=0;
    imgA=NULL;
    eig_image=NULL;
    tmp_image=NULL;

    iniciado=0;
    q_limite=1;

    desbloqueo=0;

    // uninterlaced=0;
    undistort=0;
    procesar=0;
    transformationKind =1;   //  1 Homography, 2 Affine, 3 Euclidean
    selected_camera=1;
    zoomfactor=0;
    fromPause=0;

    reiniciar_mosaico=0;

    moveImage=0;

    iniciado=0;
    totalvideoframes=0;

    saveVideo=0;
    pauseTime=0;
    pauseVideo=0;
    stopCamera=0;

    initImage.load("icons/Logo_Dolba.jpg",0);

    pthread_mutex_init(&m_mutexPpal, NULL);
    pthread_cond_init(&sleepPrincipal, NULL);

    blackMagicOn=1;
    if (blackMagicOn==1)
        ui->actionBlackMagic->setChecked(true);

    int status=53;
   // if (blackMagicOn==1)
     //   blackMagic.initBlackMagic();

    connect(&mosaicing, SIGNAL(show_mosaic(IplImage*)),this, SLOT(visualiza_mosaico(IplImage*))); // Signal pertenece al archivo thread y Slot al principal y tienen los mismos parametros aunque solo se ponen los tipos
    //connect(&captureThread, SIGNAL(resultadoCaptura(int)),this, SLOT(actualizaEstadoCaptura(int)));
 cvNamedWindow("backImage");

}

MainWindow::~MainWindow()
{
     stop=1;
     stopCamera=1;
     capturando=0;
     //captureThread.terminate();
    delete ui;
}

void MainWindow::changeEvent(QEvent *e)
{
    QMainWindow::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void MainWindow::initialDemo()
{
    ui->off_line_button->click();
    ui->video_file_button->click();
    imagefilename = "/home/mprat/Documents/Michele/upm_mosaic/demo_from_ivan.AVI";
    ui->Caja_frame_video->setValue(54);
    load_video_other();
}

void MainWindow::on_AboutAction_triggered()
{

    QMessageBox information;
    information.setText("DOLBA project Software V 0.5 \n Done by CVG-UPM 2010");
    information.exec();

}


void MainWindow::createActions()
{//The actions variables must be declared on the .h file on a private class

    ///********************  Menu File *********************************************


    quitAction = new QAction(QIcon("../DOLBAproject/icons/quit.png"),tr("&Quit"),this);
    quitAction->setShortcut(tr("Ctrl+Q"));
    connect(quitAction,SIGNAL(triggered()),this,SLOT(close()));

    AboutAction = new QAction(QIcon("../DOLBAproject/icons/about.png"),tr("&About"),this);
    connect(AboutAction,SIGNAL(triggered()),this  ,SLOT(on_AboutAction_triggered()));


}

void MainWindow::createStatusBar()
{
    statusmessage= "Ready";
    statusBar()->showMessage(statusmessage);
}

void MainWindow::on_input_Avi_file_clicked()
{
    ui->frameVideoControl->setHidden(false);

}

void MainWindow::on_playButton_clicked()
{

    ui->playButton->setDisabled(1);
    ui->stopButton->setDisabled(0);
    ui->pauseButton->setDisabled(0);
    ui->LoadVideo->setDisabled(true);



    ui->Intro_marca->setDisabled(false);
    ui->Process_button->setDisabled(false);

    ui->Undistord->setDisabled(1);
    ui->Interlaced->setDisabled(1);

    pauseVideo=0;
    viewType=1;
    //tiempo_captura=tiempo_captura-tiempoPausa;
    //  tiempo_captura.restart();


    if(iniciado==0)
    {
        frameI=ui->Caja_frame_video->value();

        frameF=99;
        stop=0;
        process(frameI,frameF);
    }


}


void MainWindow::on_stopButton_clicked()
{

    ui->stopButton->setDisabled(1);
    ui->playButton->setDisabled(0);
    ui->pauseButton->setDisabled(1);
    ui->Undistord->setDisabled(0);
    ui->LoadVideo->setDisabled(false);
    ui->frame_7->setVisible(false);



    //meter la imagen de entrada
   // QImage initImage;
   // initImage.load("icons/Logo_Dolba.jpg",0);
    ui->Imagen_1->setPixmap(QPixmap::fromImage(initImage));

    printf("boton_stop_pulsado\n");

    statusmessage="System Stop";
    statusBar()->showMessage(statusmessage);
    pauseVideo=0;
    iniciado=0;
    stop=1;
    procesar=0;

    ui->Caja_frame_video->setValue(0);
    ui->Barra_control_video->setValue(0);

    ui->Intro_marca->setDisabled(true);
    ui->Process_button->setDisabled(true);

}

void MainWindow::ui_change_load_video()
{
    ui->groupBox_2->setEnabled(true);
    ui->frameVideoControl->setHidden(false);
    ui->playButton->setDisabled(false);
    ui->pauseButton->setDisabled(true);
    ui->stopButton->setDisabled(true);
}

void MainWindow::load_video_other()
{
    QByteArray aux =imagefilename.toLatin1();

    imagefilenameOpenCV=aux.data();

    ui->stopButton->setDisabled(1);
    ui->playButton->setDisabled(0);
    ui->pauseButton->setDisabled(1);
    input=2;
    statusmessage="Selected Video File:  ";
    statusmessage.append(imagefilenameOpenCV);
    statusBar()->showMessage(statusmessage);

    ui->Clean_marks_button->click();
}

void MainWindow::on_LoadVideo_clicked()
{
    imagefilename = QFileDialog::getOpenFileName(this,"Image to Open", "/home", "videos (*.avi *.AVI *,wmv *.mov *.mpg *.mp4 *.VOB");
    if(!imagefilename.isEmpty())
    {
        ui_change_load_video();
    }
    else
    {
        return;
    }

    load_video_other();
}


void MainWindow::on_actionExit_triggered()
{
    stop=1;
    close();

}

void MainWindow::on_actionImage_set_triggered()
{
    ui->input_image->click();
    ui->frame->setEnabled(true);
    input=3;
    ui->frameImageControl->setHidden(false);
    ui->groupBox_2->setHidden(true);
    ui->frameVideoControl->setHidden(true);
    ui->frame_13->setHidden(true);
    ui->frame_2->setHidden(true);
    ui->frame_6->setHidden(true);

    if ( ui->input_image->isChecked())
    {ui->frame->setEnabled(true);
        ui->frameImageControl->setHidden(false);
        ui->frame->setHidden(false);
        input=3;
    }
    else
    { ui->frame->setDisabled(true);
        ui->frameImageControl->setHidden(true);
        ui->frame->setHidden(true);}
}

void MainWindow::on_actionAvi_file_triggered()
{
    ui->input_Avi_file->click();
    input=2;
    ui->frame->setHidden(true);
    ui->frameImageControl->setHidden(true);
    ui->frame_13->setHidden(true);
    ui->frame_2->setHidden(true);
    ui->frame_6->setHidden(true);


    if (ui->input_Avi_file->isChecked())
    {
        ui->groupBox_2->setEnabled(true);
        ui->frameVideoControl->setHidden(false);
        ui->groupBox_2->setHidden(false);
        input=2;
    }
    else
    {ui->groupBox_2->setDisabled(true);
        ui->frameVideoControl->setHidden(true);
        ui->groupBox_2->setHidden(true);}
}

void MainWindow::on_actionStart_Capturing_triggered()
{
    ui->input_Live_camera->click();
    input=1;
    ui->frame_2->setHidden(true);
    ui->frame->setHidden(true);
    ui->frameImageControl->setHidden(true);
    ui->frame_6->setHidden(true);
    ui->frameVideoControl->setHidden(true);
    ui->groupBox_2->setHidden(true);
    ui->frame_13->setHidden(false);
    ui->timeEdit->setTime(tiempo_salida);
    ui->Clean_marks_button->click();
}



void MainWindow::on_StartCamera_clicked()
{
    capturando=1;
    input=selected_camera;
    frameI=1;
    frameF=99999999;
    stop=0;
    ui->Intro_marca->setDisabled(false);
    ui->Process_button->setDisabled(false);
    stopCamera=0;
    viewType=0;

    //int status;
    //status=90;
    resultado=9;
    printf("blackMAgic= %d\n",blackMagicOn);

    if (blackMagicOn==1)
    {
        //captureThread.bmCapture(status);
        usleep(1000);
        printf("status = %d\n",status);
        if (status!=0 && status!=4 && status!=3)
            process(frameI,frameF);
}
    else
       if(process(frameI,frameF)==6)
           printf("no se puede capturar\n");

    //sleep(3);

//    printf("status %d\n",status);

   // exit(0);
    //if (status==1)
    //    process(frameI,frameF);

}


void MainWindow::on_pushButton_clicked()
{
    imagefilename = QFileDialog::getOpenFileName(this,"Browse directory", "/home", "Images (*.tif *.png *.jpg *.bmp)");
    ui->Initial_frame_lineEdit->setText(imagefilename);
}

void MainWindow::on_Start_image_clicked()
{
    image_filenameOpenCV=imagefilename;
    image_filenameOpenCV.chop(8);
    QString imagekind1=imagefilename;
    imagekind1.remove(image_filenameOpenCV,Qt::CaseSensitive);
    imagekind1.remove(0,4);
    std::string str_imgkind= imagekind1.toStdString();
    imagekind=str_imgkind.c_str();

    QString framenumber=imagefilename;
    framenumber.remove(image_filenameOpenCV,Qt::CaseSensitive);
    framenumber.chop(4);
    frameI=framenumber.toInt(0,10);
    QString endframenumber;
    endframenumber=ui->End_frame_name_lineEdit->text();
    frameF=endframenumber.toInt(0,10);

    viewType=0;
    process(frameI,frameF);

}


void MainWindow::on_Stop_clicked()
{
    stop=1;
    stopCamera=1;
    capturando=0;
    ui->Intro_marca->setDisabled(true);
    ui->Process_button->setDisabled(true);
    ui->frame_7->setVisible(false);
    ui->Imagen_1->setPixmap(QPixmap::fromImage(initImage));

    //  printf("fin captura\n");

   // captureThread.blackMagic.parar=1;


   // captureThread.blackMagic.closeCamera();


   // captureThread.terminate();
}

void MainWindow::on_toolButton_clicked()
{
    stop=1;
}



int MainWindow:: process(int frameI,int frameF)
{

    stop=0;

    clock_t comienzo, tiempo_proceso;
    if (iniciado==0)
    {
        QTime tiempo_salida(0,0,0);
        segundos_ini=tiempo_salida.elapsed();
        iniciado=1;
        comienzo =clock();
        int segundos;
    }


    //double totalvideoframes=0;

    FILE *fichero_error_acumulado;
    fichero_error_acumulado=fopen("fichero_error_acumulado.log","w");
    FILE *fichero;
    fichero=fopen("fichero_error_total.log","w");


    int numero_mosaicos=0;

    int reinicializaciones=0;

    // int q;
    // int q_reinicio=1;
    char image_filename[255];
    //char imagekind[255]="tif";

   // imagekind="tif";

    const int MAX_CORNERS = 500;
    int channels;
    int nofound=0;
    int countFound;
    int reinicializar=0;
    int reiniciado=0;
    int last_computed=1;

    int interruptresize=0;
    int limite_numero_puntos=300;
    int pyramids_level=3;
    double diferencia_error_det=0.1;//0.5;
    double limite_error_acumulado=0.07;//0.03;
    int numero_mosaicos_minimo=2;

    IplImage* imgCaptured = NULL;
    imgCaptured=NULL;
    IplImage* img0 = NULL;
    img0=NULL;

    IplImage* img_buffer = NULL;
    imgA=NULL;
    IplImage* imgB = NULL; // imagen de fondo
    IplImage* imgC = NULL; // imagen para dibujar el optical flow sobre fondo negro
    QPixmap imagen_directo_process, imagen_directo_bad;


    IplImage* imgestabilizacion = NULL; // imagen de fondo

    IplImage* swap_temp;
    IplImage* mapx = NULL;
    IplImage* mapy = NULL;

    mapx=NULL;
    mapy=NULL;

    CvMat* Hall_e = NULL;
    CvMat* Hall_m = NULL;
    CvMat* aux1_e = NULL;
    CvMat* aux1_m = NULL;
    CvMat* H = NULL;
    CvMat* Hinv = NULL;
    CvMat* HacuInv = NULL;

    CvMat* Hold_e = NULL;
    CvMat* Hold_m = NULL;

    Hall_e = cvCreateMat(3,3,CV_32FC1);
    Hall_m = cvCreateMat(3,3,CV_32FC1);
    cvSetIdentity(Hall_m);

    aux1_e = cvCreateMat(3,3,CV_32FC1);
    aux1_m = cvCreateMat(3,3,CV_32FC1);
    H = cvCreateMat(3,3,CV_32FC1);
    Hinv = cvCreateMat(3,3,CV_32FC1);
    HacuInv = cvCreateMat(3,3,CV_32FC1);

    cvSetIdentity(Hinv);
    cvSetIdentity(HacuInv);


    Hold_e = cvCreateMat(3,3,CV_32FC1);
    Hold_m = cvCreateMat(3,3,CV_32FC1);

    //CvCapture* capture;



    double tmedio=0;
    int flags=0;
    int Matchedfeatures=0;
    int not_homography=0;
    int ROISIZE=20;
    int start_closed_search_num = 1;


    CvMat* PointImg1;
    CvMat* PointImg2;

    IplImage* pyrA;
    IplImage* pyrB;
    IplImage *imgrotated_esta;
    IplImage *imgrotated;
    IplImage *imgrgb_Esta;

    char mosaico_name[30];

    CvMat mask = imread("mask_none_white.tif", 0);
    int num_imgs_buffer = 20;
    int step_size = 5;

    double Homo_Direct_error;
    double Homo_Inv_error;
    double Homo_error_total;
    double Homo_error_acumulado;
    double Homo_error_total_rec;

    char homografia_numero[200];
    char imagen_numero[200];

    MOSAIC *mosaic_2;
    mosaic_2 =new(MOSAIC);
    // printf("Creando Mosaic Structure\n"); //meter Text LabelH !!!!

    mosaic_2->finalsizex=3; mosaic_2->finalsizey =3;
    mosaic_2->levelx=1; mosaic_2->levely =1;

    //make a new mosaic structure for the recomputed mosaic
    MOSAIC *mosaic_recomp;
    mosaic_recomp = new(MOSAIC);

    mosaic_recomp->finalsizex=3;
    mosaic_recomp->finalsizey=3;
    mosaic_recomp->levelx=1;
    mosaic_recomp->levely=1;


    ///************************************************

    CvMat *intrinsic = (CvMat*)cvLoad(intrinsics_parameters_file);
    CvMat *distortion = (CvMat*)cvLoad(distortion_parameters_file);



    printf("\nmatriz de distorsion 2 = %s\n",distortion_parameters_file);
    printf("\nmatriz de intrinsics 2 = %s\n",intrinsics_parameters_file);

    numero_imagenes=1;
    numero_homo=1;

    //   printf("input %d\n",input);


    if(input==1)
    {
        printf("blackMagicOn =%d\n",blackMagicOn);

        if (blackMagicOn==0)
        {
            frameF=10000;
            frameI=1;
            capture = cvCaptureFromCAM(0);
            if(!cvGrabFrame(capture))
            {
                printf("No se puede grabar frame\n");
               // 0);
            // goto exit;
                stop=0;
                return 6;//problema con la captura
            }
            else
                imgCaptured=cvQueryFrame(capture);
        }
        else
        {
            //imgCaptured=cvCreateImage(cvGetSize(captureThread.blackMagic.image_RGB),captureThread.blackMagic.image_RGB->depth,captureThread.blackMagic.image_RGB->nChannels);


//            while(captureThread.blackMagic.nodisponible==0)
//            {
//           //     printf("durmiendo %d\n",captureThread.blackMagic.nodisponible);
//                usleep(1000);
//            }

//            imgCaptured=cvCreateImage(cvGetSize(captureThread.blackMagic.image_RGB),captureThread.blackMagic.image_RGB->depth,captureThread.blackMagic.image_RGB->nChannels);
//            cvCopyImage(captureThread.blackMagic.image_RGB,imgCaptured);

          //  cvReleaseImage(&captureThread.blackMagic.image_RGB);


//            captureThread.blackMagic.nodisponible=0;

            sleep(1);

        }
      /*  cvNamedWindow( "imageCap", 1 );
        cvShowImage( "imageCap", imgCaptured ); // se muestran las imagenes en dichas ventanas
                //                cvMoveWindow( "imageC",0, 0 );
        cvWaitKey(0);

         1);*/


    } //fin de input==1
    else if (input==2) //captura desde video.
    {
        //printf("Capturing from VideoFile \n");
        capture = cvCaptureFromFile(imagefilename.toAscii());
        totalvideoframes= cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_COUNT );

        frameI=(int)(totalvideoframes*((double)(frameI)/100));
        frameF=totalvideoframes-1;
        cvSetCaptureProperty(capture,CV_CAP_PROP_POS_FRAMES,(double)(frameI));
        statusmessage=QString("Total video framse %1, Inicio %2, Fin %3:").arg(totalvideoframes).arg(frameI).arg(frameF);
       // statusBar()->showMessage(statusmessage);
        imgCaptured=cvQueryFrame(capture);

    } //fin de input==2
    else if (input==3)
    {
        char image_filename[255];
        capture=NULL;
        std::string str= image_filenameOpenCV.toStdString();
        sprintf(image_filename,"%s%04d%s",str.c_str(),frameI,imagekind);

        printf("nombre archivo %s\n",image_filename);

        imgCaptured = cvLoadImage(image_filename);

    }// fin input==3



    if(undistort==1)
    {
        mapx = cvCreateImage( cvGetSize(imgCaptured), IPL_DEPTH_32F, 1 );
        mapy = cvCreateImage( cvGetSize(imgCaptured), IPL_DEPTH_32F, 1 );
        cvInitUndistortMap(intrinsic,distortion,mapx,mapy);
    }


    img0=correctImage( imgCaptured,img0, undistort, uninterlaced, ROISIZE, intrinsic, distortion, mapx,mapy);


    imgA = cvCreateImage(cvGetSize(img0), IPL_DEPTH_8U,1 );
    imgB = cvCreateImage(cvGetSize(img0), IPL_DEPTH_8U,1 );


    if (img0->origin)
    {
        cvFlip(img0, NULL, 0);
        img0->origin= 0;
    }
    if(img0->nChannels==3)
    {
        cvCvtColor(img0,imgA,CV_BGR2GRAY);
        channels=3;
    }
    else
    {
        cvCopy(img0,imgA,NULL);
        channels=1;
    }
    
    // iniciado=1;


    imgC=  cvCreateImage( cvGetSize(imgA), 8, 3 );

    mosaic_2->imgDoble=  cvCreateImage( cvSize(mosaic_2->finalsizex*imgA->width,mosaic_2->finalsizey*imgA->height), 8, channels );
    mosaic_2->imgDobleLast=cvCreateImage( cvSize(mosaic_2->finalsizex*imgA->width,mosaic_2->finalsizey*imgA->height), 8, channels );
    mosaic_recomp->imgDoble=  cvCreateImage( cvSize(mosaic_recomp->finalsizex*imgA->width,mosaic_recomp->finalsizey*imgA->height), 8, channels );
    mosaic_recomp->imgDobleLast=cvCreateImage( cvSize(mosaic_recomp->finalsizex*imgA->width,mosaic_recomp->finalsizey*imgA->height), 8, channels );

    imgestabilizacion=  cvCreateImage( cvGetSize(imgA), 8, channels );

    CvSize img_sz   = cvGetSize( imgA );
    win_size = 10;
    eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
    tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );

    corner_count = MAX_CORNERS;
    CvSize pyr_sz = cvSize( imgA->width+8, imgA->height/3 );
    pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
    pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
    cornersA  = new CvPoint2D32f[ MAX_CORNERS ];

    CvPoint2D32f* swap_points;
    //cvGoodFeaturesToTrack(imgA,eig_image,tmp_image,cornersA,&corner_count,0.01,5.0,0,3,0,0.04);
//    cvGoodFeaturesToTrack(imgA,eig_image,tmp_image,cornersA,&corner_count,0.01,7.0,0,3,1,0.04);
    cvGoodFeaturesToTrack(imgA,eig_image,tmp_image,cornersA,&corner_count,0.01,7.0,&mask,3,1,0.04);
    cvFindCornerSubPix(imgA,cornersA,corner_count,cvSize(win_size,win_size),cvSize(-1,-1),
                       cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));

//    sprintf(imagen_numero, "corners/corner%04d.xml", numero_imagenes);
//    cvSave(imagen_numero, cornersA);

    char  features_found[ MAX_CORNERS ];
    float feature_errors[ MAX_CORNERS ];
    CvPoint2D32f* cornersB   = new CvPoint2D32f[ MAX_CORNERS ];

    int restarTime;



    img_buffer=cvCreateImage(cvGetSize(img0), img0->depth, img0->nChannels);


    //*************************************************************************
    //         Inicializo variables para la correccion de las homografias
    //*************************************************************************
    HAcuGuardadaMain = cvCreateMat(3,3,CV_32FC1);
    HABucleMain = cvCreateMat(3,3,CV_32FC1);
    cornersGuardadaMain  = new CvPoint2D32f[ MAX_CORNERS ];
    pyrGuardadaMain = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
    imgGuardadaMain = cvCreateImage(cvGetSize(imgA), imgA->depth, imgA->nChannels);
    cvSetIdentity(HAcuGuardadaMain);
    cvSetIdentity(HABucleMain);
    cvCopy(imgA,imgGuardadaMain);
//    cvCopy(cornersA,cornersGuardadaMain);
    cornersGuardadaMain=cornersA;

//    sprintf(homografia_numero,"homogs/cumhom%04d.xml", 1);
//    cvSave(homografia_numero,Hall_m);

    for (q = frameI; q < frameF; q++)
    {
        if(stop==1)
        {
            statusmessage="System Stop";
            statusBar()->showMessage(statusmessage);
            break;
        }

        if (pauseVideo==0)
        {
        //    printf("from pause = %d\n",fromPause);

            if (fromPause==1)//si vengo de un pause
            {
                fromPause=0;
                restarTime=tiempo_salida.elapsed();
                segundos_ini=segundos_ini-pauseTime+restarTime;
            }

            tiempo_proceso=clock();
            cvCopy(img0,img_buffer);//Se va a corresponder con la imagen A.

            segundos=tiempo_salida.elapsed()-segundos_ini;

            tiempo_captura=tiempo_salida;
            tiempo_captura=tiempo_captura.addMSecs(segundos);


//printf("antes de la captura\n");

//printf("antes de salir\n");
//1);
            if (input==1)
            {
                if(blackMagicOn==0)
                {
                    ui->timeEdit->setTime(tiempo_captura);
                    if(!cvGrabFrame(capture))
                    {
                        printf("No se puede grabar frame\n");
                       // exit(0);
                     //   goto exit;
                        stop= 1;
                        return 0;
                    }
                    else
                        imgCaptured=cvRetrieveFrame(capture);
                }
                else
                {

                //    printf("capturando bMB\n");

                  //  exit(1);

//                    while((captureThread.blackMagic.nodisponible==0)&&(stopCamera==0))
//                    {
//                   //     printf("durmiendo\n");
//                        usleep(1000);

//                    }

//                    if(stopCamera==1)
//                    {
//                        printf("antes de break\n");
//                        break;
//                    }

//                    imgCaptured=cvCreateImage(cvGetSize(captureThread.blackMagic.image_RGB),captureThread.blackMagic.image_RGB->depth,captureThread.blackMagic.image_RGB->nChannels);
//                    cvCopyImage(captureThread.blackMagic.image_RGB,imgCaptured);

//                    cvReleaseImage(&captureThread.blackMagic.image_RGB);


//                    captureThread.blackMagic.nodisponible=0;

                   // cvCopy(threadcaptura.capturaImagen_Th,imgCaptured);
                }
            }
            if (input==2)
            {
                imgCaptured=cvQueryFrame(capture);
            }
            if (input==3)
            {
                std::string str= image_filenameOpenCV.toStdString();

                str= image_filenameOpenCV.toStdString();
                sprintf(image_filename,"%s%04d%s",str.c_str(),q,imagekind);
                imgCaptured=cvLoadImage(image_filename);
                if( ! imgCaptured )
                {

                    printf( "unable to load image B from %s", image_filename );
                }
            }

            img0=correctImage(imgCaptured,img0, undistort, uninterlaced, ROISIZE, intrinsic, distortion, mapx,mapy);

            if (img0->origin)
            {
                cvFlip(img0, NULL, 0);
                img0->origin= 0;
            }
            if(channels==3)
            {
                cvCvtColor(img0,imgB,CV_BGR2GRAY);
            }
            else
            {
                cvCopy(img0,imgB,NULL);
            }
/*
            cvNamedWindow( "Prueba Captura B ", 1 );
              cvShowImage( "Prueba Captura B ", imgB); // se muestran las imagenes en dichas ventanas
              cvMoveWindow( "Prueba Captura B ",0, 0 );
              cvWaitKey(2);

*/
            //    printf("entrelazado =%d\n",uninterlaced);
            if(blackMagicOn==1&&input==1)
                cvReleaseImage(&imgCaptured);

            if (procesar==1) //flag para que muestre las imagenes sin procesar
            {

             //   printf("hasta aqui\n");




                ui->Intro_marca->setDisabled(false);
                if (q%10==0 || reiniciado==1)// reinicia todo de 1 de cada 10 INLUIR Si vendo de reiniciar.
                    //if(1)
                {
                    if (reiniciado==1)
                        printf("reinicio de datos\n");

                    reiniciado=0;

                  //  cvGoodFeaturesToTrack(imgA,eig_image,tmp_image,cornersA,&corner_count,0.01,5.0,0,3,0,0.04);
//                    cvGoodFeaturesToTrack(imgA,eig_image,tmp_image,cornersA,&corner_count,0.01,5.0,0,3,1,0.04);
                    cvGoodFeaturesToTrack(imgA,eig_image,tmp_image,cornersA,&corner_count,0.01,5.0, &mask,3,1,0.04);
                    cvFindCornerSubPix(imgA,cornersA,corner_count,cvSize(win_size,win_size),cvSize(-1,-1),
                                       cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));

//                    sprintf(imagen_numero, "corners/corner%04d.xml", numero_imagenes - 1);
//                    cvSave(imagen_numero, cornersA);
                }

                cvCalcOpticalFlowPyrLK(
                        imgA,//Initial Image
                        imgB,//Final Image
                        pyrA,//buffer to store pyramid Image  of imgA
                        pyrB,//buffer to store pyramid Image  of imgB
                        cornersA,//points to which motions it to be found
                        cornersB,// new locations of futures in cornersA are located
                        corner_count,//number of points on cornersA list
                        cvSize( win_size,win_size ),//windows used to local coherent motion
                        pyramids_level,//pyramids level estaba set a 5 con esto se reduce a 0.11 el tiempo
                        features_found,//size (count), 1 if point found, 0, is not point found
                        feature_errors,//error between patch points on images. Can be set to NULL
                        cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),
                        flags//CV_LKFLOW_PYR_A_READY//flags// Special Flags CV_LKFLOW_PYR_A_READY, CV_LKFLOW_PYR_B_READY, CV_LKFLOW_INITIAL_GUESSES
                        );

                //flags = CV_LKFLOW_PYR_A_READY; //We now have Pyr A Ready
                //   flags |= CV_LKFLOW_PYR_A_READY; //We now have Pyr A Ready
                countFound=0;



                for( int i=0; i<corner_count; i++ )
                {
                    if( features_found[i]==0|| feature_errors[i]>limite_numero_puntos )
                    {
                        continue;
                    }
                    countFound++;
                }
                if(countFound<=0)
                {
                    countFound=1;  //ADD TO SOLVE CREATION OF A MATRIX OF SIZE O ERROR
                }
                PointImg1=cvCreateMat(countFound,2,CV_32F);
                PointImg2=cvCreateMat(countFound,2,CV_32F);
                countFound=0;

                for( int i=0; i<corner_count; i++ )
                {
                    if( features_found[i]==0|| feature_errors[i]>limite_numero_puntos )
                    {
                        continue;
                    }
                    CvPoint p0 = cvPoint(cvRound( cornersA[i].x ),cvRound( cornersA[i].y )   );
                    CvPoint p1 = cvPoint(cvRound( cornersB[i].x ),cvRound( cornersB[i].y )   );

                    cvmSet(PointImg1,countFound,0,p0.x);
                    cvmSet(PointImg1,countFound,1,p0.y);
                    cvmSet(PointImg2,countFound,0,p1.x);
                    cvmSet(PointImg2,countFound,1,p1.y);
                    countFound++;

                    //  cvLine( imgC, p0, p1, CV_RGB(255,0,0),2 );
                    //  cvCircle(imgC,p1,3,CV_RGB(0,255,0),1,8);
                    Matchedfeatures++;
                }

                if(Matchedfeatures >= 10)
                {
                    if(CalHomography(PointImg2,PointImg1,H, transformationKind)==1)
                    {
                        Homo_Direct_error=0; //esto se puede hacer en la inicializacion
                        Homo_Inv_error=0;

                        // Reprojection_error(H, PointImg1, PointImg2, &Homo_Direct_error, &Homo_Inv_error);

                        //Homo_error_total=Homo_Direct_error+Homo_Inv_error;

                        Homo_error_total=cvDet(H);


 //                       if (Homo_error_total>(1+diferencia_error_det) || Homo_error_total<(1-diferencia_error_det))//(Homo_error_total>3 )
                        if (diferencia_error_det < fabs(1-Homo_error_total)) //cuando el deterinante de H es muy diferente a 1, es decir, no se parece a la imagen anterior.
                        {
                            printf("error H\n");
                            if (not_homography==1)
                            {
                                nofound++; //llevo un conteo de homografias erroneas consecutivas.
                            }
                            not_homography=1; //para evitar que entre donde no debe al detectar el error

                            // esto creo que me lo puedo evitar haciendo if not_homagaphy en el swap
                   /*         CV_SWAP(imgA,imgB,swap_temp); // me olvido de la imagen nueva y considera a la antigua como válida
                            CV_SWAP( pyrA, pyrB, swap_temp );
                            CV_SWAP( cornersA, cornersB, swap_points );
                            */

                            cvSetIdentity(H);// Para el paso introduzco una homografía unidad.

                      //      cvCopy(img_buffer,img0); //
                        }
                        else
                        {
                          //  CalHomography(PointImg1,PointImg2,Hinv, transformationKind);

                            cvInvert(H,Hinv,CV_SVD);

                            not_homography=0;// si tengo homografía, limpio todo y vuelvo a iniciar la cuenta.
                            nofound=0;




//                             cvGEMM(Hinv,HacuInv,1,NULL,NULL,HacuInv,0);


                            if(q%1==0) // homografias cada 2
                            {
                              //  printf("q = %d homo= %d\n",q, numero_imagenes);

                                sprintf(homografia_numero,"imagenes/homografia%04d.xml",numero_imagenes);
                               // cvCopy(Hinv,HacuInv);
                                cvSave(homografia_numero,H);
//                                cvSave(homografia_numero,Hinv);
                               // cvCopy(Hinv,HacuInv);
                                cvSetIdentity(HacuInv);


                                // first cumhum entry is the same as the first homography calculation
                                if (numero_imagenes == 1)
                                {
                                    sprintf(homografia_numero,"homogs/cumhom%04d.xml", numero_imagenes);
                                    cvSave(homografia_numero,H);
                                }
                                numero_homo++;

                            }
                            if(q%1==0) //imagenes cada 2 para que el video sea mas continuo. Ojo que cuando las recupere tiene que ser cada dos.
                            {

                                //  sprintf(imagen_numero,"/media/DATOS/ARIES/resultados/mosaico%04d.png",numero_imagenes);
                                sprintf(imagen_numero,"imagenes/mosaico%04d.tif",numero_imagenes);
                                cvSaveImage(imagen_numero,img0,0); //Debo guardar la imagen 0

                                numero_imagenes++;
                            }


                            //     Homo_error_total=cvDet(H);

//                            fprintf(fichero,"%f\n",Homo_error_total);

                            if (q==frameI)
                            {
                                cvCopy(H,Hall_e,NULL);
                                cvCopy(H,Hall_m,NULL);
                                sprintf(homografia_numero,"homogs/cumhom%04d.xml", numero_imagenes - 1);
                                cvSave(homografia_numero,Hall_m);
                            }
                            else
                            {
                                int testType = 3; //1 = using homographies to calculate overlap
                                                  //2 = using optical flow calculations to calculate overlap


                                cvCopy(Hall_e,Hold_e,NULL); //Hold es la imagen antigua que la devolvere en el cas de que H sea mala
                                cvCopy(Hall_e,aux1_e,NULL);
                                //cvGEMM(H,aux1_e,1,NULL,NULL,Hall_e,0); //H es la homo caluclada, aux la homo que llevo acumulada y Hall la salida H(i-1)*H(i)
                                cvGEMM(Hall_e,H,1,NULL,NULL,Hall_e,0); //H es la homo caluclada, aux la homo que llevo acumulada y Hall la salida H(i-1)*H(i)
  //                              cvCopy(Hall_m,Hold_m,NULL); //Hold es la imagen antigua que la devolvere en el cas de que H sea mala
  //                              cvCopy(Hall_m,aux1_m,NULL);
    //                            cvGEMM(H,aux1_m,1,NULL,NULL,Hall_m,0); //H es la homo caluclada, aux la homo que llevo acumulada y Hall la salida H(i-1)*H(i)
                                
                                cvGEMM(Hall_m, H, 1,NULL,NULL,Hall_m,0);//esta correcto el centro no se ha salido.

                                switch(testType){
                                case 1:
                                {

                                    //store Hall_m for all homographies (keep track of all accumulated homographies)
                                    // to see if it ever returns to the original (within a threshold)

    //                                double detH = cvDet(H);
    //                                printf("determinant of H = %f\n", detH);

                                    //save all the accumulated homographies
                                    sprintf(homografia_numero,"homogs/cumhom%04d.xml",numero_imagenes - 1);
                                    cvSave(homografia_numero,Hall_m);

    //                                cvNamedWindow("Current image");
    //                                cvShowImage("Current image", img0);   // color

    //                                cvNamedWindow("imgB");
    //                                cvShowImage("imgB", imgB);        // black and white

    //                                printf("Hall_m for img %d \n", numero_imagenes);
    //                                print_cv_matrix(Hall_m);

                                     // don't count the last 10 images
                                    for (int loopi = start_closed_search_num; loopi < numero_imagenes - 1 - num_imgs_buffer; loopi+=step_size)
                                    {
                                        CvMat* dif_temp = cvCreateMat(3, 3, CV_32FC1);
                                        sprintf(homografia_numero,"homogs/cumhom%04d.xml",loopi);
                                        CvMat* H_loopi = (CvMat*)cvLoad(homografia_numero);

    //                                    printf("old hall for image %d\n", loopi);
    //                                    print_cv_matrix(H_loopi);

                                        cvAbsDiff(H_loopi, Hall_m, dif_temp);
    //                                    cvSub(H_loopi, Hall_m, dif_temp);

    //                                    printf("absolute difference\n");
    //                                    print_cv_matrix(dif_temp);

    //                                    double val = cvSum(dif_temp).val[0];
    //                                    double val = cvDet(dif_temp);
                                        cvPow(dif_temp, dif_temp, 2);
                                        double val = sqrt(cvSum(dif_temp).val[0]);

    //                                    printf("val = %f \n", val);

                                        bool process = true;

                                        if (process && abs(val) <= 50)
                                        {
    //                                        printf("val = %f \n", val);
                                            printf("Current img number (numero_imagenes - 1) %d and return num (loopi) %d\n", numero_imagenes - 1, loopi);
                                            //printf("Difference val = %f \n", val);

                                            //recalibrate homography estimation with the error?
                                            //nope, that really doesn't work.
                                            //cvGEMM(Hall_m, Hall_e, 1, NULL, NULL, Hall_m, 0);

                                            //recompute optical flow/homography when an overlap is hit
                                            //if it's an overlap you know that the homography is good and you can save a few steps


                                            IplImage* overlap_img = cvCreateImage(cvGetSize(imgB), IPL_DEPTH_8U, 1);
                                            IplImage* temp_overlap = cvCreateImage(cvGetSize(imgB), IPL_DEPTH_8U, 1);
                                            sprintf(imagen_numero,"imagenes/mosaico%04d.tif", loopi);
                                            overlap_img = cvLoadImage(imagen_numero);

                                            if(overlap_img->nChannels==3)
                                            {
                                                cvCvtColor(overlap_img,temp_overlap,CV_BGR2GRAY);
                                                channels=3;
                                            }
                                            else
                                            {
                                                cvCopy(overlap_img,temp_overlap,NULL);
                                                channels=1;
                                            }

    //                                        cvNamedWindow("Overlap image (loopi)");
    //                                        cvShowImage("Overlap image (loopi)", temp_overlap);

//                                            cvGoodFeaturesToTrack(temp_overlap,eig_image,tmp_image,cornersA,&corner_count,0.01,5.0,0,3,1,0.04);
                                            cvGoodFeaturesToTrack(temp_overlap,eig_image,tmp_image,cornersA,&corner_count,0.01,5.0,&mask,3,1,0.04);
                                            cvFindCornerSubPix(temp_overlap,cornersA,corner_count,cvSize(win_size,win_size),cvSize(-1,-1),
                                                               cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));

                                            cvCalcOpticalFlowPyrLK(
                                                    temp_overlap,//Initial Image (current image, black and white)
                                                    imgB,//Final Image (overlapped)
                                                    pyrA,//buffer to store pyramid Image  of imgA
                                                    pyrB,//buffecvSetIdentityr to store pyramid Image  of imgB
                                                    cornersA,//points to which motions it to be found
                                                    cornersB,// new locations of futures in cornersA are located
                                                    corner_count,//number of points on cornersA list
                                                    cvSize( win_size,win_size ),//windows used to local coherent motion
                                                    pyramids_level,//pyramids level estaba set a 5 con esto se reduce a 0.11 el tiempo
                                                    features_found,//size (count), 1 if point found, 0, is not point found
                                                    feature_errors,//error between patch points on images. Can be set to NULL
                                                    cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),
                                                    flags//CV_LKFLOW_PYR_A_READY//flags// Special Flags CV_LKFLOW_PYR_A_READY, CV_LKFLOW_PYR_B_READY, CV_LKFLOW_INITIAL_GUESSES
                                                    );

                                            //flags = CV_LKFLOW_PYR_A_READY; //We now have Pyr A Ready
                                            //   flags |= CV_LKFLOW_PYR_A_READY; //We now have Pyr A Ready
                                            countFound=0;

                                            for( int i=0; i<corner_count; i++ )
                                            {
                                                if( features_found[i]==0|| feature_errors[i]>limite_numero_puntos )
                                                {
                                                    continue;
                                                }
                                                countFound++;
                                            }
                                            if(countFound<=0)
                                            {
                                                countFound=1;  //ADD TO SOLVE CREATION OF A MATRIX OF SIZE O ERROR
                                            }
                                            PointImg1=cvCreateMat(countFound,2,CV_32F);
                                            PointImg2=cvCreateMat(countFound,2,CV_32F);
                                            countFound=0;

                                            for( int i=0; i<corner_count; i++ )
                                            {
                                                if( features_found[i]==0|| feature_errors[i]>limite_numero_puntos )
                                                {
                                                    continue;
                                                }
                                                CvPoint p0 = cvPoint(cvRound( cornersA[i].x ),cvRound( cornersA[i].y )   );
                                                CvPoint p1 = cvPoint(cvRound( cornersB[i].x ),cvRound( cornersB[i].y )   );

                                                cvmSet(PointImg1,countFound,0,p0.x);
                                                cvmSet(PointImg1,countFound,1,p0.y);
                                                cvmSet(PointImg2,countFound,0,p1.x);
                                                cvmSet(PointImg2,countFound,1,p1.y);
                                                countFound++;

                                                //  cvLine( imgC, p0, p1, CV_RGB(255,0,0),2 );
                                                //  cvCircle(imgC,p1,3,CV_RGB(0,255,0),1,8);
                                                Matchedfeatures++;
                                            }

                                            CvMat* H_new = cvCreateMat(3, 3, CV_32FC1);
                                            CvMat* H_newinv = cvCreateMat(3, 3, CV_32FC1);

                                            if(Matchedfeatures >= 10)
                                            {
                                                if(CalHomography(PointImg2,PointImg1,H_new, transformationKind)==1)
                                                {
                                                    //H has homography from current to the overlapped image
                                                    cvInvert(H_new, H_newinv, CV_SVD);

                                                    //for the indices of the images in the cycle
                                                    for (int j = loopi + 1; j < numero_imagenes; j++)
                                                    {
                                                        //iterative recomputing of homographies
                                                        sprintf(homografia_numero,"homogs/cumhom%04d.xml",j);
                                                        CvMat* H_j = (CvMat*)cvLoad(homografia_numero);

    //                                                    printf("Old H_j matrix for j = %d\n", j);
    //                                                    print_cv_matrix(H_j);

                                                        CvMat* H_j_prime = cvCreateMat(3, 3, CV_32FC1);
                                                        cvSetIdentity(H_j_prime);
                                                        CvMat* H_j_new = cvCreateMat(3, 3, CV_32FC1);
                                                        cvSetIdentity(H_j_new);

    //                                                    printf("\nj = %d\n", j);

    //                                                    for (int mult_index = j + 1; mult_index < numero_imagenes; mult_index++)
    //                                                    {
    //                                                         printf("mi = %d", mult_index);
    //                                                        sprintf(homografia_numero,"imagenes/homografia%04d.xml",mult_index);
    //                                                        CvMat* temp_H = (CvMat*)cvLoad(homografia_numero);
    //                                                        cvGEMM(H_j_prime, temp_H, 1, NULL, NULL, H_j_prime);
    //                                                    }

                                                        for (int mult_index = numero_imagenes - 1; mult_index > j; mult_index--)
                                                        {
    //                                                        printf("mi = %d ", mult_index);
                                                            sprintf(homografia_numero,"imagenes/homografia%04d.xml",mult_index);
                                                            CvMat* temp_H = (CvMat*)cvLoad(homografia_numero);
                                                            cvGEMM(temp_H, H_j_prime, 1, NULL, NULL, H_j_prime);
                                                        }

    //                                                    printf("\n H_j_prime: ");
    //                                                    print_cv_matrix(H_j_prime);

    //                                                    cvInvert(H_j_prime, H_j_prime, CV_SVD);
    //                                                    // cvGEMM(H, H_j_prime, 1, NULL, NULL, H_j_prime);
    //                                                    // cvGEMM(H_loopi, H_j_prime, 1, NULL, NULL, H_j_prime);
    //                                                    cvGEMM(H, H_loopi, 1, NULL, NULL, H_loopi);
    //                                                    cvGEMM(H_j_prime, H_loopi, 1, NULL, NULL, H_j_prime);


//                                                        printf("detHnew = %f, detH_loopi = %f, det h_j_prime = %f \n", cvDet(H_new), cvDet(H_loopi), cvDet(H_j_prime));

                                                        cvInvert(H_j_prime, H_j_prime, CV_SVD);
                                                        cvGEMM(H_new, H_j_prime, 1, NULL, NULL, H_j_prime);
                                                        cvGEMM(H_loopi, H_j_prime, 1, NULL, NULL, H_j_prime);

    //                                                    printf("new prime \n");
    //                                                    print_cv_matrix(H_j_prime);

                                                        double h_j_const = 1.0* (numero_imagenes - j - 1)/(numero_imagenes - loopi - 2);
                                                        double h_j_prime_const = 1.0*(j - loopi - 1)/(numero_imagenes - loopi - 2);

                                                        // FOR DEBUGGING
    //                                                     h_j_const = 0.0;
    //                                                     h_j_prime_const = 1.0;

    //                                                    printf("hj = %f hjprime = %f\n", h_j_const, h_j_prime_const);

                                                        cvCvtScale(H_j, H_j, h_j_const);
                                                        cvCvtScale(H_j_prime, H_j_prime, h_j_prime_const);
                                                        cvAdd(H_j, H_j_prime, H_j_new);

                                                        double detHjnew = cvDet(H_j_new);
//                                                        printf("det h_j_new before adjust = %f \n", detHjnew);

    //                                                    cvCvtScale(H_j_new, H_j_new, pow(1.0/abs(detHjnew), 1.0/3.0));
    //                                                    if (detHjnew < 0)
    //                                                    {
    //                                                        cvCvtScale(H_j_new, H_j_new, -1.0);
    //                                                    }
    //                                                    detHjnew = cvDet(H_j_new);
    //                                                    printf("det h_j_new after adjust = %f \n", detHjnew);

                                                        //save new H_j_new estimate
                                                         sprintf(homografia_numero,"homogs/cumhom%04d.xml", j);
//                                                         if (abs(detHjnew - 1.0) <= 0.2)
//                                                         {
                                                            cvSave(homografia_numero,H_j_new);
//                                                         }
    //                                                     else
    //                                                     {
    //                                                         cvSave(homografia_numero, H_j);
    //                                                     }

    //                                                     printf("j = %d \n", j);
    //                                                     printf("old H_j \n");
    //                                                     print_cv_matrix(H_j);

    //                                                     printf("new H_j_new \n");
    //                                                     print_cv_matrix(H_j_new);
                                                    }

                                                    //need to account for special case of the current image homography
                                                    //H is current homography
    //                                                //printf("regularly computed h_numero_imagines-1\n");
    //                                                //print_cv_matrix(Hall_m);
    //                                                sprintf(homografia_numero,"homogs/cumhom%04d.xml",numero_imagenes - 1);
    //                                                //printf("newly computed\n");
    //                                                //print_cv_matrix(H);
    //                                                cvGEMM(H_loopi, Hinv, 1, NULL, NULL, H);
                                                    //cvGEMM(H_loopi, H, 1, NULL, NULL, H);
    //                                                cvSave(homografia_numero, H);
    //                                                //cvSave(homografia_numero, Hall_m);

    //                                                //print_cv_matrix(H);

                                                    //redraw homographies
                                                    drawMosaicFromHomographies(mosaic_recomp, channels, loopi + 1, numero_imagenes, last_computed, true);
                                                    last_computed = numero_imagenes - 1;


                                                    sprintf(imagen_numero, "mosaicos/mosrec%04d.tif", last_computed);
                                                    cvSaveImage(imagen_numero, mosaic_recomp->imgDoble);

                                                    cvNamedWindow("Redrawn Homography", 0);
                                                    cvShowImage("Redrawn Homography", mosaic_recomp->imgDoble);

                                                    sprintf(imagen_numero, "mosaicos/mosnormal%04d.tif", numero_imagenes - 1);
                                                    cvSaveImage(imagen_numero, mosaic_2->imgDoble);

                                                    cvReleaseMat(&PointImg1);
                                                    cvReleaseMat(&PointImg2);
                                                }
                                            }
                                        }


                                    }
                                    drawMosaic3(mosaic_recomp, img0, Hall_m, Hold_m, channels, q, nofound, interruptresize);
                                    cvNamedWindow("cumhomall", 0);
                                    cvShowImage("cumhomall", mosaic_recomp->imgDoble);

                                    // reconstruccionH.centroOk(HAcuGuardadaMain,HABucleMain,Hall_m,H,cornersA,cornersGuardadaMain,pyrA,pyrGuardadaMain,imgA,imgGuardadaMain,transformationKind);

//                                    break;
                                }
                                break;
                                    // END CASE 1
                                 case 2:
                                {
                                    bool update = false;
                                    //save all the accumulated homographies
                                    sprintf(homografia_numero,"homogs/cumhom%04d.xml",numero_imagenes - 1);
                                    cvSave(homografia_numero,Hall_m);

                                    // don't count the last n images
                                    int num_imgs_buffer = 10;
                                    CvMat* H_loopi = cvCreateMat(3, 3, CV_32FC1);
                                    cvSetIdentity(H_loopi);

//                                    int maskheight = 400;//3*(imgA->height)/4;
//                                    int maskwidth = 128;//3*(imgA->width)/4;
//                                    int xpos = 100; //(imgA->width - maskwidth)/2;
//                                    int ypos = 100; //(imgA->height - maskheight)/2;
//                                    CvRect mask = cvRect(xpos, ypos, maskwidth, maskheight);

                                    for (int loopi = start_closed_search_num; loopi < numero_imagenes - num_imgs_buffer; loopi+=5)
                                    {
                                        update = false;
                                        sprintf(homografia_numero,"homogs/cumhom%04d.xml",loopi);
                                        H_loopi = (CvMat*)cvLoad(homografia_numero);

                                        IplImage* overlap_img = cvCreateImage(cvGetSize(imgB), IPL_DEPTH_8U, 1);
                                        IplImage* temp_overlap = cvCreateImage(cvGetSize(imgB), IPL_DEPTH_8U, 1);
                                        sprintf(imagen_numero,"imagenes/mosaico%04d.tif", loopi);
                                        temp_overlap = cvLoadImage(imagen_numero);
                                        if(temp_overlap->nChannels==3)
                                        {
                                            cvCvtColor(temp_overlap,overlap_img,CV_BGR2GRAY);
                                            channels=3;
                                        }
                                        else
                                        {
                                            cvCopy(temp_overlap,overlap_img,NULL);
                                            channels=1;
                                        }

                                        cvGoodFeaturesToTrack(overlap_img,eig_image,tmp_image,cornersA,&corner_count,0.01,5.0,&mask,3,1,0.04);
//                                        cvSetImageROI(overlap_img, mask);
//                                        cvSetImageROI(imgB, mask);
//                                        cvGoodFeaturesToTrack(overlap_img,eig_image,tmp_image,cornersA,&corner_count,0.01,5.0,0,3,1,0.04);
                                        cvFindCornerSubPix(overlap_img,cornersA,corner_count,cvSize(win_size,win_size),cvSize(-1,-1),
                                                           cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
//                                        cvResetImageROI(overlap_img);
//                                        cvResetImageROI(imgB);

//                                        sprintf(imagen_numero, "corners/corner%04d.xml", loopi);
//                                        cornersA = (CvPoint2D32f*)cvLoad(homografia_numero);

                                        //optical flow
                                        cvCalcOpticalFlowPyrLK(
                                                overlap_img,//Initial Image (overlapped)
                                                imgB,//Final Image (current image, black adn white)
                                                pyrA,//buffer to store pyramid Image  of imgA
                                                pyrB,//buffecvSetIdentityr to store pyramid Image  of imgB
                                                cornersA,//points to which motions it to be found
                                                cornersB,// new locations of futures in cornersA are located
                                                corner_count,//number of points on cornersA list
                                                cvSize( win_size,win_size ),//windows used to local coherent motion
                                                pyramids_level,//pyramids level estaba set a 5 con esto se reduce a 0.11 el tiempo
                                                features_found,//size (count), 1 if point found, 0, is not point found
                                                feature_errors,//error between patch points on images. Can be set to NULL
                                                cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),
                                                flags//CV_LKFLOW_PYR_A_READY//flags// Special Flags CV_LKFLOW_PYR_A_READY, CV_LKFLOW_PYR_B_READY, CV_LKFLOW_INITIAL_GUESSES
                                                );


                                        countFound=0;
                                        for( int i=0; i<corner_count; i++ )
                                        {
                                            if( features_found[i]==0|| feature_errors[i]>limite_numero_puntos )
                                            {
                                                continue;
                                            }
                                            countFound++;
                                        }
                                        if(countFound<=0)
                                        {
                                            countFound=1;  //ADD TO SOLVE CREATION OF A MATRIX OF SIZE O ERROR
                                        }
                                        PointImg1=cvCreateMat(countFound,2,CV_32F);
                                        PointImg2=cvCreateMat(countFound,2,CV_32F);
                                        countFound=0;

                                        IplImage* imgC = cvCreateImage(cvGetSize(overlap_img), overlap_img->depth, 1);
                                        cvCopy(overlap_img, imgC);

                                        for( int i=0; i<corner_count; i++ )
                                        {
                                            if( features_found[i]==0|| feature_errors[i]>limite_numero_puntos )
                                            {
                                                continue;
                                            }
                                            CvPoint p0 = cvPoint(cvRound( cornersA[i].x ),cvRound( cornersA[i].y )   );
                                            CvPoint p1 = cvPoint(cvRound( cornersB[i].x ),cvRound( cornersB[i].y )   );

                                            cvmSet(PointImg1,countFound,0,p0.x);
                                            cvmSet(PointImg1,countFound,1,p0.y);
                                            cvmSet(PointImg2,countFound,0,p1.x);
                                            cvmSet(PointImg2,countFound,1,p1.y);
                                            countFound++;

                                            cvLine( imgC, p0, p1, CV_RGB(255,0,0),2 );
                                            cvCircle(imgC,p1,3,CV_RGB(0,255,0),1,8);
                                            cvNamedWindow("debugcur");
                                            cvShowImage("debugcur", imgC);

                                            Matchedfeatures++;
                                        }

                                        CvMat* H_new = cvCreateMat(3, 3, CV_32FC1);
                                        CvMat* H_newinv = cvCreateMat(3, 3, CV_32FC1);

                                        if(Matchedfeatures >= 10)
                                        {
                                            if(CalHomography(PointImg2,PointImg1,H_new, transformationKind)==1)
                                            {
                                                double detHnew = cvDet(H_new); //homography error

//                                                printf("detHnew = %f \n", detHnew);
//                                                print_cv_matrix(H_new);

                                                if (diferencia_error_det > fabs(1-detHnew))
                                                {
                                                    cvInvert(H_new, H_newinv, CV_SVD);

//                                                    cvNamedWindow("overlap");
//                                                    cvShowImage("overlap", overlap_img);

//                                                    cvNamedWindow("imgB (current)");
//                                                    cvShowImage("imgB (current)", imgB);

                                                    printf("img number = %d, loopi = %d, dethnew = %f\n", numero_imagenes - 1, loopi, detHnew);
                                                    update = true;

                                                    CvMat* H_j_prime = cvCreateMat(3, 3, CV_32FC1);
                                                    CvMat* H_j_new = cvCreateMat(3, 3, CV_32FC1);

                                                    //for the indices of the images in the cycle
                                                    for (int j = loopi + 1; j < numero_imagenes; j++)
                                                    {
                                                        //iterative recomputing of homographies
                                                        sprintf(homografia_numero,"homogs/cumhom%04d.xml",j);
                                                        CvMat* H_j = (CvMat*)cvLoad(homografia_numero);

                                                        cvSetIdentity(H_j_prime);
                                                        cvSetIdentity(H_j_new);

//                                                        printf("j = %d \n", j);

                                                        for (int mult_index = numero_imagenes - 1; mult_index > j; mult_index--)
                                                        {
//                                                            printf("indx = %d\n", mult_index);
                                                            sprintf(homografia_numero,"imagenes/homografia%04d.xml",mult_index);
                                                            CvMat* temp_H = (CvMat*)cvLoad(homografia_numero);
                                                            cvGEMM(temp_H, H_j_prime, 1, NULL, NULL, H_j_prime);
                                                        }

//                                                        printf("detHnew = %f, detH_loopi = %f, det h_j_prime = %f \n", cvDet(H_new), cvDet(H_loopi), cvDet(H_j_prime));

                                                        cvInvert(H_j_prime, H_j_prime, CV_SVD);
                                                        cvGEMM(H_new, H_j_prime, 1, NULL, NULL, H_j_prime);
                                                        cvGEMM(H_loopi, H_j_prime, 1, NULL, NULL, H_j_prime);

                                                        double h_j_const = 1.0* (numero_imagenes - j - 1)/(numero_imagenes - loopi - 2);
                                                        double h_j_prime_const = 1.0*(j - loopi - 1)/(numero_imagenes - loopi - 2);

                                                        // FOR DEBUGGING
//                                                         h_j_const = 0.0;
//                                                         h_j_prime_const = 1.0;

    //                                                    printf("hj = %f hjprime = %f\n", h_j_const, h_j_prime_const);

                                                        cvCvtScale(H_j, H_j, h_j_const);
                                                        cvCvtScale(H_j_prime, H_j_prime, h_j_prime_const);
                                                        cvAdd(H_j, H_j_prime, H_j_new);

                                                        double detHjnew = cvDet(H_j_new);
                                                        printf("det h_j_new before adjust = %f \n", detHjnew);

                                                        // only save it if the newly computed thing is a homography too
                                                        // this means that there is a slower accumulation of error
                                                        // (and a maximum error at a certain point once there is a loop)
                                                        if (diferencia_error_det > fabs(1-detHjnew))
                                                        {
                                                            printf("updated\n");
                                                            //save new H_j_new estimate
                                                             sprintf(homografia_numero,"homogs/cumhom%04d.xml", j);
                                                             cvSave(homografia_numero,H_j_new);

//                                                             printf("new\n");
//                                                             print_cv_matrix(H_j_new);
                                                        }
                                                    }
                                                }
                                            }


                                        }

                                        if (update)
                                        {
                                            drawMosaicFromHomographies(mosaic_recomp, channels, loopi + 1, numero_imagenes, last_computed, false);
                                            last_computed = numero_imagenes - 1;

                                            sprintf(imagen_numero, "mosaicos/mosrec%04d.tif", last_computed);
                                            cvSaveImage(imagen_numero, mosaic_recomp->imgDoble);

                                            sprintf(imagen_numero, "mosaicos/mosnormal%04d.tif", last_computed);
                                            cvSaveImage(imagen_numero, mosaic_2->imgDoble);

                                            cvNamedWindow("Redrawn Homography", 0);
                                            cvShowImage("Redrawn Homography", mosaic_recomp->imgDoble);
                                        }
                                    }

                                }
                                break;
                                    // END CASE 2
                                default:
                                    printf("pick a valid type! \n");
                                    break;
                                }
                            }

                          //  Homo_error_acumulado=cvDet(Hall_m);//solo para la H del mosaico
                         //   fprintf(fichero_error_acumulado,"%f\n",Homo_error_acumulado);

                           /* if (fabs(1-Homo_error_acumulado)>limite_error_acumulado){

                                printf("error acumulado\n");
                                reinicializar=1;
                                cvSetIdentity(Hall_m);

                            }*/


                            //Mosaicos hacia delante

                            if((viewType==1) && (not_homography==0)) //moasico activado
                            {
                                ui->Black_reset->setHidden(true);

                                if (reinicializar==0){

                                    drawMosaic3(mosaic_2,img0,Hall_m,Hold_m,channels,q,nofound, interruptresize);
                                }

                                IplImage *imgrgb_Mos;
                                if (true)
                                {
                                    imgrgb_Mos=cvCreateImage(cvGetSize(mosaic_2->imgDoble), IPL_DEPTH_8U,3 );

                                    if(img0->nChannels==3)
                                    {
                                        cvCvtColor(mosaic_2->imgDoble,imgrgb_Mos,CV_BGR2RGB);
                                    }
                                    else                                {
                                        cvCvtColor(mosaic_2->imgDoble,imgrgb_Mos,CV_GRAY2RGB);
                                    }
                                }
                                else
                                {
                                    imgrgb_Mos=cvCreateImage(cvGetSize(mosaic_recomp->imgDoble), IPL_DEPTH_8U,3 );

                                    if(img0->nChannels==3)
                                    {
                                        cvCvtColor(mosaic_recomp->imgDoble,imgrgb_Mos,CV_BGR2RGB);
                                    }
                                    else                                {
                                        cvCvtColor(mosaic_recomp->imgDoble,imgrgb_Mos,CV_GRAY2RGB);
                                    }
                                }

                                if (ang_rotation!=0)
                                {
                                    imgrotated=cvCreateImage(cvGetSize(imgrgb_Mos), IPL_DEPTH_8U,3);
                                    rotatedImage(imgrgb_Mos, imgrotated, ang_rotation, 1.0);
                                    cvCopy(imgrotated,imgrgb_Mos,NULL);
                                    cvReleaseImage(&imgrotated);
                                }

                                moveImage=0;

                                QImage qimg_Mos((uchar*)imgrgb_Mos->imageData,imgrgb_Mos->width,imgrgb_Mos->height,QImage::Format_RGB888);
                                ui->Imagen_1->setPixmap(QPixmap::fromImage(qimg_Mos));

                                cvWaitKey(2);
                                cvReleaseImage(&imgrgb_Mos);
                            }


                            if ((viewType==0)&&(not_homography==0))//estabilizacion activada
                            {
                                ui->frame_7->setHidden(true);
                                ui->Black_reset->setHidden(false);
                                cvWarpPerspective(img0,imgestabilizacion,Hall_e);
                                cvSetImageCOI(imgestabilizacion,1);
                                if (  (int) (100*cvCountNonZero(imgestabilizacion)/(imgestabilizacion->width*imgestabilizacion->height)<100-BlackResetRatio) || nofound>4 )
                                {
                                    printf("reinicio fondo\n");
                                    cvSetIdentity(Hall_e); //reinicializacion de la H de estabilizacion.
                                    //start_closed_search_num = numero_imagenes;
                                }

                                cvSetImageCOI(imgestabilizacion,0);
                                imgrgb_Esta=cvCreateImage(cvGetSize(imgestabilizacion), IPL_DEPTH_8U,3 );
                                if(img0->nChannels==3)
                                {
                                    cvCvtColor(imgestabilizacion,imgrgb_Esta,CV_BGR2RGB);
                                }
                                else
                                {
                                    cvCvtColor(imgestabilizacion,imgrgb_Esta,CV_GRAY2RGB);
                                }
                                if (ang_rotation!=0)
                                {
                                    imgrotated_esta=cvCreateImage(cvGetSize(imgrgb_Esta), IPL_DEPTH_8U,3);
                                    rotatedImage(imgrgb_Esta, imgrotated_esta, ang_rotation, 1.0);
                                    cvCopy(imgrotated_esta,imgrgb_Esta,NULL);
                                    cvReleaseImage(&imgrotated_esta);
                                }

                                moveImage=0;

                                QImage qimg_Esta((uchar*)imgrgb_Esta->imageData,imgrgb_Esta->width,imgrgb_Esta->height,QImage::Format_RGB888);
                                ui->Imagen_1->setPixmap(QPixmap::fromImage(qimg_Esta));
                                cvReleaseImage(&imgrgb_Esta);

                              //  printf("estabilizacion\n");

                            }
                            if(viewType==2)//vista en directo
                            {
                                ui->frame_7->setHidden(true);
                                ui->Black_reset->setHidden(true);

                                IplImage *imgRGB_process=cvCreateImage(cvGetSize(img0), IPL_DEPTH_8U,3 ); //ojo lo genero siempre
                                if(img0->nChannels==3)
                                {
                                    cvCvtColor(img0,imgRGB_process,CV_BGR2RGB); //en la imagen en directo tengo que ver lo que va llegando. No tengo que quitar la imagen
                                }
                                else
                                {
                                    cvCvtColor(img0,imgRGB_process,CV_GRAY2RGB);
                                }

                                moveImage=0;

                                QImage qimg_raw_process((uchar*)imgRGB_process->imageData,imgRGB_process->width,imgRGB_process->height,QImage::Format_RGB888);
                                imagen_directo_process=QPixmap::fromImage(qimg_raw_process);

                                ui->Imagen_1->setPixmap(imagen_directo_process);

                                cvReleaseImage(&imgRGB_process);

                            }


                        }//fin del else cuando el det(H) es bueno
                    }// end if(CalHomographyRANSAC(PointImg1,PointImg2,H, FramePoints))

                    else //no tengo homografía
                    {
                      /*  printf("H no found, change points \n");
                        nofound++;
                        CV_SWAP( imgA, imgB, swap_temp );
                        CV_SWAP( pyrA, pyrB, swap_temp );
                        CV_SWAP( cornersA, cornersB, swap_points );
                        cvCopy(img_buffer,img0);*/

                        printf("error H\n");
                        if (not_homography==1)
                        {
                            nofound++; //llevo un conteo de homografias erroneas consecutivas.
                        }
                        not_homography=1;

                        cvSetIdentity(H);
                    }
                } //fin de puntos encuentrados ms que 4

                else // número de puntos encontrados menor que 4
                {
                    printf("puntos_menos que los requeridos\n");
                    if (not_homography==1)
                    {
                        nofound++;
                    }

                    not_homography=1;

              /*      CV_SWAP( imgA, imgB, swap_temp );
                    CV_SWAP( pyrA, pyrB, swap_temp );
                    CV_SWAP( cornersA, cornersB, swap_points );
                    cvCopy(img_buffer,img0);*/
                    cvSetIdentity(H);
                } // fin de la matchedfeatures>4 y tipo de transformacion 1

                //*********************************gestion de los errores.

                if(nofound>3) // Ya se han cambiado la imagen y las H y lo de mas antes. Tengo que identificar este q para el limite del mosaico.
                {
                    printf("mas de 4 H\n");
                    reinicializar=1;
                    // q_limite=q;
                };

                if (not_homography==0)
                {
                    CV_SWAP( imgA, imgB, swap_temp );
                    CV_SWAP( pyrA, pyrB, swap_temp );
                    CV_SWAP( cornersA, cornersB, swap_points );
                }



                if(reinicializar==1)
                {
                    marca=numero_imagenes-5;
                    reiniciado=1;

                    printf("numero_imagenn %d, Q_limite= %d\n",numero_imagenes,q_limite);

                    if (numero_imagenes-q_limite>numero_mosaicos_minimo)
                    {
                        numero_mosaicos++;
                        sprintf(mosaico_name,"mosaicos/mosaico%04d.tif",numero_mosaicos);
                        cvSaveImage(mosaico_name,mosaic_2->imgDoble,0); //Debo guardar la imagen 0

                    }


                    mosaic_2->finalsizex=3;
                    mosaic_2->finalsizey=3;
                    mosaic_2->lastsizex=3;
                    mosaic_2->lastsizey=3;
                    cvSetZero(mosaic_2->imgDoble);
                    cvSetZero(mosaic_2->imgDobleLast);


                    if ((marca-q_limite)>6)
                    {
                        printf("reinicio H en marca= %d q_limite=%d\n",marca, q_limite);
                        //  mosaicing.make_mosaic(marca,q_limite,abort);
                    }
                    // q_limite=numero_imagenes;


                    reinicializar=0;
                    q_limite=numero_imagenes;

                    if(img0->nChannels==3) //img0 es la ultima que he cogido
                    {
                        cvCvtColor(img0,imgA,CV_BGR2GRAY);
                    }
                    else
                    {
                        cvCopy(img0,imgA,NULL);
                    }
                    if (imgA->origin)
                    {
                        cvFlip(imgA, NULL, 0);
                        imgA->origin= 0;
                    }

                    reinicializaciones++;

// Que muestre la imagen que tenga



                    nofound=0;
//                    cvGoodFeaturesToTrack(imgA,eig_image,tmp_image,cornersA,&corner_count,0.01,5.0,0,3,0,0.04);
                    cvGoodFeaturesToTrack(imgA,eig_image,tmp_image,cornersA,&corner_count,0.01,5.0,&mask,3,0,0.04);
                    cvFindCornerSubPix(imgA,cornersA,corner_count,cvSize(win_size,win_size),cvSize(-1,-1),
                                       cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));

                    if(viewType==0)//solo mostrar la imagen mala cuando se esta estabilizando.
                    {

                        IplImage *imgRGB_bad=cvCreateImage(cvGetSize(img0), IPL_DEPTH_8U,3 ); //ojo lo genero siempre
                        if(img0->nChannels==3)
                        {
                            cvCvtColor(img0,imgRGB_bad,CV_BGR2RGB); //en la imagen en directo tengo que ver lo que va llegando. No tengo que quitar la imagen
                        }
                        else
                        {
                            cvCvtColor(img0,imgRGB_bad,CV_GRAY2RGB);
                        }



                        moveImage=0;
                        QImage qimg_raw_bad((uchar*)imgRGB_bad->imageData,imgRGB_bad->width,imgRGB_bad->height,QImage::Format_RGB888);
                        imagen_directo_bad=QPixmap::fromImage(qimg_raw_bad);

                        ui->Imagen_1->setPixmap(imagen_directo_bad);
                    }

                    /*QMessageBox *errorProceso;
                    errorProceso =new QMessageBox;
                    errorProceso->setModal(false);

                    errorProceso->setText("Error en el procesamiento");
                    errorProceso->show();*/

                }

                //                cvNamedWindow( "imageC", 1 );
                //                cvShowImage( "imageC", imgC ); // se muestran las imagenes en dichas ventanas
                //                cvMoveWindow( "imageC",0, 0 );
                //                cvWaitKey(1);
                // cvSetZero(imgC);

                cvWaitKey(1);//es el unico que se tiene que quedar


                Matchedfeatures=0;
                cvReleaseMat( &PointImg1 );
                cvReleaseMat( &PointImg2 );
                //cvReleaseMat( &InliersMask );

            }//fin de if (procesar==)


            if (procesar==0)
            {
                ui->frame_7->setHidden(true);
                ui->Intro_marca->setDisabled(true);

                estabilizacion=0;
                mosaico=0;
                IplImage *imgRGB=cvCreateImage(cvGetSize(img0), IPL_DEPTH_8U,3 ); //ojo lo genero siempre
                if(img0->nChannels==3)
                {
                    cvCvtColor(img0,imgRGB,CV_BGR2RGB);
                }
                else
                {
                    cvCvtColor(img0,imgRGB,CV_GRAY2RGB);
                }

                QImage qimg_raw((uchar*)imgRGB->imageData,imgRGB->width,imgRGB->height,QImage::Format_RGB888);
                imagen_directo=QPixmap::fromImage(qimg_raw);

                ui->Imagen_1->setPixmap(imagen_directo);
                cvReleaseImage(&imgRGB);

                cvWaitKey(1); // puede que lo tenga que mantener
            }



            tmedio=(clock()-comienzo)/(double)CLOCKS_PER_SEC;
            comienzo =clock();
           // statusmessage=QString("Tiempo bulcle %1: %2 (useg), %3 (fps):").arg(q).arg(tmedio).arg(1/tmedio);
           // statusBar()->showMessage(statusmessage);

            if(input==2 && stop!=1)
            {
                ui->Caja_frame_video->setValue(((double)100*q/totalvideoframes));

            }


        }//fin de pause==0
        else
        {
            q--;
         //   printf("q= %d\n",q);
            cvWaitKey(2);
        }




    }// End for q

    // mosaico_iterativo2.mosaicing_iteractive(transformationKind,numero_mosaicos);

    if (stop==1)
        {
         ui->Imagen_1->setPixmap(QPixmap::fromImage(initImage));
     }

    if(capture)
    {
        if(blackMagicOn==0)
        {
            cvReleaseCapture( &capture );
            printf("Capture release\n");
            cvWaitKey(1);
        }
    }

    cvReleaseImage( &imgA );
    cvReleaseImage( &imgB );
    cvReleaseImage( &imgC );
    cvReleaseImage( &imgestabilizacion);
    cvReleaseMat (&H);

    fclose(fichero_error_acumulado);
    fclose(fichero);

    printf("numero_mosaicos %d\n",numero_mosaicos);

    printf("numero de reiniciios = %d\n",reinicializaciones);

    return 1 ;
}

void MainWindow::on_Barra_control_video_sliderMoved(int position)
{
    ui->Caja_frame_video->setValue(ui->Barra_control_video->value());
}

void MainWindow::on_Caja_frame_video_valueChanged(int )
{
    ui->Barra_control_video->setValue(ui->Caja_frame_video->value());
}

void MainWindow::on_Restart_clicked()
{
    if (viewType==5)
        viewType=0;
    else if (viewType==1)
    {
        reiniciar_mosaico=1;
        viewType++;
    }
    else if (viewType==2)
    {
        viewType=0;
    }
    else
        viewType++;
    printf("viewtype=%d\n",viewType);

}


void MainWindow::on_Intro_marca_clicked()
{
    if ((mark_index<10)&&(procesar==1))
    {
        printf("\n\ndiferencia %d\n\n",(numero_imagenes-q_limite));
        if ((numero_imagenes-q_limite)>10)
            {
            marca=numero_imagenes;
            //  printf("marca_1 %d\n",marca);
            //ui->timeEdit_2->setTime(tiempo_captura);
            image_mark_matrix[mark_index][0]=marca;
            mosaic_limit_matrix[mark_index][0]=q_limite;
            qtime_mark_matrix[mark_index][0]=tiempo_captura;

            ui->timeMarca_1->setTime(qtime_mark_matrix[0][0]);
            ui->timeMarca_2->setTime(qtime_mark_matrix[1][0]);
            ui->timeMarca_3->setTime(qtime_mark_matrix[2][0]);
            ui->timeMarca_4->setTime(qtime_mark_matrix[3][0]);
            ui->timeMarca_5->setTime(qtime_mark_matrix[4][0]);
            ui->timeMarca_6->setTime(qtime_mark_matrix[5][0]);
            ui->timeMarca_7->setTime(qtime_mark_matrix[6][0]);
            ui->timeMarca_8->setTime(qtime_mark_matrix[7][0]);
            ui->timeMarca_9->setTime(qtime_mark_matrix[8][0]);
            ui->timeMarca_10->setTime(qtime_mark_matrix[9][0]);

            if (image_mark_matrix[0][0]!=0)
                ui->Marca_1_button->setDisabled(false);
            if (image_mark_matrix[1][0]!=0)
                ui->Marca_2_button->setDisabled(false);
            if (image_mark_matrix[2][0]!=0)
                ui->Marca_3_button->setDisabled(false);
            if (image_mark_matrix[3][0]!=0)
                ui->Marca_4_button->setDisabled(false);
            if (image_mark_matrix[4][0]!=0)
                ui->Marca_5_button->setDisabled(false);
            if (image_mark_matrix[5][0]!=0)
                ui->Marca_6->setDisabled(false);
            if (image_mark_matrix[6][0]!=0)
                ui->Marca_7->setDisabled(false);
            if (image_mark_matrix[7][0]!=0)
                ui->Marca_8->setDisabled(false);
            if (image_mark_matrix[8][0]!=0)
                ui->Marca_9->setDisabled(false);
            if (image_mark_matrix[9][0]!=0)
                ui->Marca_10->setDisabled(false);
            mark_index++;
        }
        else
        {

            QMessageBox *malaMarca;
            malaMarca =new QMessageBox;
            malaMarca->setModal(false);

            malaMarca->setText("No hay imagenes suficientes para hacer un mosaico");
            malaMarca->show();
           // malaMarca->activateWindow();
            //malaMarca.setModal(false);
           // malaMarca.exec();


            /*statusmessage= "No hay imagenes suficientes para hacer un mosaico";
            statusBar()->showMessage(statusmessage);*/


        }

    }
}

void MainWindow::on_actionInterlaced_triggered()
{
    if (uninterlaced==0)
        uninterlaced=1;
    else
        uninterlaced=0;
}

void MainWindow::on_actionUndistord_triggered()
{
    if(undistort==0)
        undistort=1;
    else
        undistort=0;
}

/*void MainWindow::on_dial_dialMoved(int value)
{
    salida_dial=ui->dial->value();
    ang_rotation=180-salida_dial*180/50;
    printf("salida dial %d\n",salida_dial);
    printf("rotate angle %f\n",ang_rotation);
}*/

void MainWindow::on_ang_mas_clicked()
{
    ang_rotation=ang_rotation+5;
}


void MainWindow::on_angMenos_clicked()
{
    ang_rotation=ang_rotation-5;
}


void MainWindow::on_actionHomography_triggered()
{
    transformationKind=1;
    ui->actionAffine->setChecked(false);
    ui->actionEuclidean->setChecked(false);
    if(!(ui->actionHomography->isChecked()))
    {
        ui->actionHomography->setChecked(true);
    }
}


void MainWindow::on_actionAffine_triggered()
{
    transformationKind=2;
    ui->actionHomography->setChecked(false);
    ui->actionEuclidean->setChecked(false);

    if(!(ui->actionAffine->isChecked()))
    {
        ui->actionAffine->setChecked(true);
    }
}

void MainWindow::on_actionEuclidean_triggered()
{
    transformationKind=3;
    ui->actionHomography->setChecked(false);
    ui->actionAffine->setChecked(false);

    if(!(ui->actionEuclidean->isChecked()))
    {
        ui->actionEuclidean->setChecked(true);
    }
}

void MainWindow::on_Black_reset_sliderMoved(int position)
{
    BlackResetRatio=ui->Black_reset->value();
}



void MainWindow::on_actionFirewire_triggered()
{
    ui->actionFirewire->setChecked(true);
    ui->actionUSB->setChecked(false);
    selected_camera=1;

}

void MainWindow::on_actionUSB_triggered()
{
    ui->actionFirewire->setChecked(false);
    ui->actionUSB->setChecked(true);
    selected_camera=4;
}

void MainWindow::on_On_line_button_clicked()
{
    ui->actionStart_Capturing->trigger();
    ui->frame_2->setHidden(true);
    ui->frame_13->setHidden(false);
}

void MainWindow::on_off_line_button_clicked()
{
    ui->frame_2->setHidden(true);
    ui->frame_6->setHidden(false);
}

void MainWindow::on_Image_set_Button_2_clicked()
{
    ui->frame_6->setHidden(true);
    ui->actionImage_set->trigger();
}

void MainWindow::on_video_file_button_clicked()
{
    ui->frame_6->setHidden(true);
    ui->actionAvi_file->trigger();

}

void MainWindow::on_Back_button_clicked()
{
    ui->frame_6->setHidden(true);
    ui->frame_2->setHidden(false);
}

void MainWindow::on_Back_button_online_clicked()
{
    ui->frame_13->setHidden(true);
    ui->frame_2->setHidden(false);
}

void MainWindow::on_image_back_button_clicked()
{
    ui->frame->setHidden(true);
    ui->frameImageControl->setHidden(true);
    ui->frame_6->setHidden(false);
}

void MainWindow::on_video_back_button_clicked()
{
    ui->groupBox_2->setHidden(true);
    ui->frameVideoControl->setHidden(true);
    ui->frame_6->setHidden(false);
}

void MainWindow::visualiza_mosaico(IplImage* mosaico_alreves)
{

    IplImage* imgMostrar;
    imgMostrar=cvCreateImage(cvGetSize(mosaico_alreves), IPL_DEPTH_8U,3);

    printf("emision senal\n");

    moveImage=1;
    zoomfactor=0;

    IplImage *imgrotated_2;

    if (ang_rotation!=0)
    {
        imgrotated_2=cvCreateImage(cvGetSize(mosaico_alreves), IPL_DEPTH_8U,3);
        rotatedImage(mosaico_alreves, imgrotated_2, ang_rotation, 1.0);
        cvCopy(imgrotated_2,mosaico_alreves,NULL);
        cvReleaseImage(&imgrotated_2);
    }

    if(mosaico_alreves->nChannels==3)

    {
        cvCvtColor(mosaico_alreves,imgMostrar,CV_BGR2RGB);
    }
    else
    {
        cvCvtColor(mosaico_alreves,imgMostrar,CV_GRAY2RGB);
    }



   // QImage qimg_mosc_inv((uchar*)mosaico_alreves->imageData,mosaico_alreves->width,mosaico_alreves->height,QImage::Format_RGB888);
    
    QImage qimg_mosc_inv((uchar*)imgMostrar->imageData,imgMostrar->width,imgMostrar->height,QImage::Format_RGB888);

    qImgExange=qimg_mosc_inv.copy();


    imagen_directo=QPixmap::fromImage(qimg_mosc_inv);




    ui->Imagen_1->setPixmap(imagen_directo);
    //ui->Imagen_1->setPixmap(QPixmap::fromImage(qimg_mosc_inv));
    //  cvReleaseImage(&mosaico_alreves);
    //stop=1;
    viewType=5;
    ui->frame_7->setHidden(false);

}


void MainWindow::on_Process_button_clicked()
{
    if (stop==0)
    {
        if (procesar==0){
            printf("process pulsado\n");
            procesar=1;

            if (input==1){
                ui->Stop->click();
                if (blackMagicOn==0)
                    cvReleaseCapture(&capture);
                usleep(200);
                ui->StartCamera->click();
            }



            if(input==2){

                stop=1;
                procesar=1;
                usleep(200);
                frameI=ui->Caja_frame_video->value();
              //  frameI=q;
                stop=0;
                process(frameI,frameF);
            }

        }

        else
        {
            procesar=0;

        }
    }
}

void MainWindow::on_Marca_1_button_clicked()
{
    statusmessage= "Processing marck";
    statusBar()->showMessage(statusmessage);
    marca=image_mark_matrix[0][0];
    q_limite=mosaic_limit_matrix[0][0];
    mosaicing.make_mosaic(marca,q_limite,abort);
}

void MainWindow::on_Marca_2_button_clicked()
{
    statusmessage= "Processing marck";
    statusBar()->showMessage(statusmessage);
    marca=image_mark_matrix[1][0];
    q_limite=mosaic_limit_matrix[1][0];
    mosaicing.make_mosaic(marca,q_limite,abort);
}

void MainWindow::on_Marca_3_button_clicked()
{
    statusmessage= "Processing marck";
    statusBar()->showMessage(statusmessage);
    marca=image_mark_matrix[2][0];
    q_limite=mosaic_limit_matrix[2][0];
    mosaicing.make_mosaic(marca,q_limite,abort);
}

void MainWindow::on_Marca_4_button_clicked()
{
    statusmessage= "Processing marck";
    statusBar()->showMessage(statusmessage);
    marca=image_mark_matrix[3][0];
    q_limite=mosaic_limit_matrix[3][0];
    mosaicing.make_mosaic(marca,q_limite,abort);
}

void MainWindow::on_Marca_5_button_clicked()
{
    statusmessage= "Processing marck";
    statusBar()->showMessage(statusmessage);
    marca=image_mark_matrix[4][0];
    q_limite=mosaic_limit_matrix[4][0];
    mosaicing.make_mosaic(marca,q_limite,abort);
}

void MainWindow::on_Marca_6_clicked()
{
    statusmessage= "Processing marck";
    statusBar()->showMessage(statusmessage);
    marca=image_mark_matrix[5][0];
    q_limite=mosaic_limit_matrix[5][0];
    mosaicing.make_mosaic(marca,q_limite,abort);
}


void MainWindow::on_Marca_7_clicked()
{
    statusmessage= "Processing marck";
    statusBar()->showMessage(statusmessage);
    marca=image_mark_matrix[6][0];
    q_limite=mosaic_limit_matrix[6][0];
    mosaicing.make_mosaic(marca,q_limite,abort);

}


void MainWindow::on_Marca_8_clicked()
{
    statusmessage= "Processing marck";
    statusBar()->showMessage(statusmessage);
    marca=image_mark_matrix[7][0];
    q_limite=mosaic_limit_matrix[7][0];
    mosaicing.make_mosaic(marca,q_limite,abort);
}


void MainWindow::on_Marca_9_clicked()
{
    statusmessage= "Processing marck";
    statusBar()->showMessage(statusmessage);
    marca=image_mark_matrix[8][0];
    q_limite=mosaic_limit_matrix[8][0];
    mosaicing.make_mosaic(marca,q_limite,abort);

}


void MainWindow::on_Marca_10_clicked()
{
    statusmessage= "Processing marck";
    statusBar()->showMessage(statusmessage);
    marca=image_mark_matrix[9][0];
    q_limite=mosaic_limit_matrix[9][0];
    mosaicing.make_mosaic(marca,q_limite,abort);
}

/*void MainWindow::click_mouse(press_leftbutton)
{
//    if (Mouse_event->button()==Qt::LeftButton)
//    {
//        visualization_center_x=Mouse_event->x();
//        visualization_center_y=Mouse_event->y();
//    }
    QPoint pba_1 ;
    pba_1=press_leftbutton->pos();

    visualization_center_x=pba_1.x();
    visualization_center_y=pba_1.y();


//    visualization_center_x=Mouse_event->x();
//    visualization_center_y=Mouse_event->y();


    printf("visualization centre x %d",visualization_center_x);
    printf("visualization centre y %d",visualization_center_y);
}*/

void MainWindow::on_actionValidate_User_triggered()
{

    Validate_user pass_window;
    if (pass_window.exec())
        desbloqueo=pass_window.getValor();
    //   printf("valor %d\n",desbloqueo);
    if (desbloqueo==1)
    {
        ui->menuProjection_Model->setEnabled(true);
        ui->menuCamera->setEnabled(true);
    }
    else
    {
        ui->menuProjection_Model->setEnabled(false);
        ui->menuCamera->setEnabled(false);
    }


}

void MainWindow::on_Clean_marks_button_clicked()
{
    ui->Marca_1_button->setDisabled(true);
    ui->Marca_2_button->setDisabled(true);
    ui->Marca_3_button->setDisabled(true);
    ui->Marca_4_button->setDisabled(true);
    ui->Marca_5_button->setDisabled(true);
    ui->Marca_6->setDisabled(true);
    ui->Marca_7->setDisabled(true);
    ui->Marca_8->setDisabled(true);
    ui->Marca_9->setDisabled(true);
    ui->Marca_10->setDisabled(true);


    image_mark_matrix[0][0]=0;
    image_mark_matrix[1][0]=0;
    image_mark_matrix[2][0]=0;
    image_mark_matrix[3][0]=0;
    image_mark_matrix[4][0]=0;
    image_mark_matrix[5][0]=0;
    image_mark_matrix[6][0]=0;
    image_mark_matrix[7][0]=0;
    image_mark_matrix[8][0]=0;
    image_mark_matrix[9][0]=0;

    mosaic_limit_matrix[0][0]=0;
    mosaic_limit_matrix[1][0]=0;
    mosaic_limit_matrix[2][0]=0;
    mosaic_limit_matrix[3][0]=0;
    mosaic_limit_matrix[4][0]=0;
    mosaic_limit_matrix[5][0]=0;
    mosaic_limit_matrix[6][0]=0;
    mosaic_limit_matrix[7][0]=0;
    mosaic_limit_matrix[8][0]=0;
    mosaic_limit_matrix[9][0]=0;
    QTime reset_marks_time(0,0,0);

    qtime_mark_matrix[0][0]=reset_marks_time;
    qtime_mark_matrix[1][0]=reset_marks_time;
    qtime_mark_matrix[2][0]=reset_marks_time;
    qtime_mark_matrix[3][0]=reset_marks_time;
    qtime_mark_matrix[4][0]=reset_marks_time;
    qtime_mark_matrix[5][0]=reset_marks_time;
    qtime_mark_matrix[6][0]=reset_marks_time;
    qtime_mark_matrix[7][0]=reset_marks_time;
    qtime_mark_matrix[8][0]=reset_marks_time;
    qtime_mark_matrix[9][0]=reset_marks_time;

    ui->timeMarca_1->setTime(qtime_mark_matrix[0][0]);
    ui->timeMarca_2->setTime(qtime_mark_matrix[1][0]);
    ui->timeMarca_3->setTime(qtime_mark_matrix[2][0]);
    ui->timeMarca_4->setTime(qtime_mark_matrix[3][0]);
    ui->timeMarca_5->setTime(qtime_mark_matrix[4][0]);
    ui->timeMarca_6->setTime(qtime_mark_matrix[5][0]);
    ui->timeMarca_7->setTime(qtime_mark_matrix[6][0]);
    ui->timeMarca_8->setTime(qtime_mark_matrix[7][0]);
    ui->timeMarca_9->setTime(qtime_mark_matrix[8][0]);
    ui->timeMarca_10->setTime(qtime_mark_matrix[9][0]);


    mark_index=0;

}



void MainWindow::on_actionCaibrate_Camera_triggered()
{
    Calibration calibration_dialog;
    calibration_dialog.exec();


}

void MainWindow::on_actionLoad_Calibration_Data_triggered()
{
    Load_calibration_data Load_calibration_dialog;
    //   Load_calibration_dialog.exec();
    if (Load_calibration_dialog.exec()){
        distortion_file=Load_calibration_dialog.get_distortion_name();
        std::string str_distortion_filename= distortion_file.toStdString();
        //distortion_name=str_distortion_filename.c_str();
        printf("nueva matriz de distrosion= %s\n",str_distortion_filename.c_str());
        distortion_parameters_file=strdup(str_distortion_filename.c_str());



        intrinsics_file=Load_calibration_dialog.get_intrinsics_name();
        std::string str_intrinsics_filename= intrinsics_file.toStdString();
        printf("nueva matriz de intrinsics= %s\n",str_intrinsics_filename.c_str());
        intrinsics_parameters_file=strdup(str_intrinsics_filename.c_str());
    }

    // printf("nueva matriz de distrosion= %s\n",distortion_parameters_file);

    //intrinsic_parameters_file=Load_calibration_data::calibration_name;

}

void MainWindow::on_actionExit_User_triggered()
{
    ui->menuProjection_Model->setEnabled(false);
    ui->menuCamera->setEnabled(false);
}

//drag tool for mosiacing image, It changes the center of the pixmap

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    if (moveImage==1 && zoomfactor>1)
    {
        if (event->button() == Qt::LeftButton)
            lastDragPos = event->pos();
    }
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
    if (moveImage==1 && zoomfactor>1)
    {
        if (event->buttons() & Qt::LeftButton) {
            pixmapOffset += event->pos() - lastDragPos;
            lastDragPos = event->pos();
            update();
        }
    }
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{
    if (moveImage==1 && zoomfactor>1)
    {
        if (event->button() == Qt::LeftButton) {

            pixmapOffset += event->pos() - lastDragPos;
            lastDragPos = QPoint();

            //   if (moveImage==1 && zoomfactor>1)
            if(zoomfactor>1)
            {
                ui->Imagen_1->clear();

                printf("zoom factor_move %d\n", zoomfactor);


                int zfactor=zoomfactor+1;
                QPixmap copiaMoverImagen;
                copiaMoverImagen=imagen_directo.copy();


                //            if((copiaMoverImagen.width()/4-pixmapOffset.x())<(copiaMoverImagen.width()/2))
               // if(abs(pixmapOffset.x())<((copiaMoverImagen.width()*zfactor)/(zfactor+1)))
                if(abs(pixmapOffset.x())<(copiaMoverImagen.width()/3))
                {
                    offsetX=pixmapOffset.x();
                    // offsetY=pixmapOffset.y();
                }
                //if(abs(pixmapOffset.y())<(copiaMoverImagen.height()/(zfactor+1)))
                if(abs(pixmapOffset.y())<(copiaMoverImagen.height()/3))
                {
                    offsetY=pixmapOffset.y();

                }

                printf("offset x %d offset y %f\n", pixmapOffset.x(),pixmapOffset.y());
                printf("limitesImage X %d\n",(copiaMoverImagen.width()/3));




                QPainter painterMoverImagen(this);
                painterMoverImagen.begin(&copiaMoverImagen);
                painterMoverImagen.scale(zfactor,zfactor);
                QRectF targetMoverImagen(0,0, copiaMoverImagen.width(), copiaMoverImagen.height());
                //QRectF sourceMoverImagen((copiaMoverImagen.width()/4-pixmapOffset.x()), (copiaMoverImagen.height()/4-pixmapOffset.y()), copiaMoverImagen.width(), copiaMoverImagen.height());
                QRectF sourceMoverImagen((copiaMoverImagen.width()/4-offsetX), (copiaMoverImagen.height()/4-offsetY), copiaMoverImagen.width(), copiaMoverImagen.height());

                painterMoverImagen.drawPixmap(targetMoverImagen, copiaMoverImagen, sourceMoverImagen);//paint the portion selected in source on target
                painterMoverImagen.end();

                //            double limiteImagen=pixmapOffset.y();
                //            printf("limite imagen %f \n",limiteImagen);
                //
                //            printf("ancho imagen %d\n", copiaMoverImagen.width());
                //            printf("alto imagen %d\n",copiaMoverImagen.height());


                ui->Imagen_1->setPixmap(copiaMoverImagen);
            }
        }


    }



//    if(qImgExange.isNull())
//        printf("imagen intercambio nula\n");
//    else
//        printf("imagen intercambio distinta de 0\n");

}


void MainWindow::on_zoom_mas_clicked()
{
    pixmapOffset.setX(0);
    pixmapOffset.setY(0);
    zoomfactor++;
    QPixmap copia;
    copia=imagen_directo.copy();
    QPainter painter(this);
    painter.begin(&copia);
    painter.scale(zoomfactor,zoomfactor);

    QRectF target(0.0, 0.0, copia.width(), copia.height());

    //    QRectF source(copia.width()/4, copia.height()/4, copia.width()*0.5, copia.height()*.5);

//    QRectF source(copia.width()/4, copia.height()/4, copia.width()/2, copia.height()/2);
//    QRectF source((copia.width()*zoomfactor)/4, (copia.height()*zoomfactor)/4, copia.width()/(2*zoomfactor), copia.height()/(2*zoomfactor));

//    QRectF source((copia.width()*zoomfactor*0.1), (copia.height()*zoomfactor*0.1), (copia.width()*zoomfactor*(1-(0.1*2))), (copia.height()*zoomfactor*(1-(0.1*2))));

    QRectF source((copia.width()*zoomfactor*0.1), (copia.height()*zoomfactor*0.1), (copia.width()*(1-0.1*zoomfactor)), (copia.height()*(1-0.1*zoomfactor)));

    //QPixmap pixmap(":myPixmap.png");

    printf("ancho, alto de copia %d, %d\n",copia.width(),copia.height());

    //QPainter(this);
    painter.drawPixmap(target, copia, source);

    //painter.drawPixmap(0,0,imgA->width*2,imgA->height*2,imagen_directo);
    painter.end();
    ui->Imagen_1->setPixmap(copia);
    update();
    printf("zoom factor %d\n",zoomfactor);
    if (zoomfactor>0)
        ui->zoom_menos->setDisabled(false);
}

void MainWindow::on_zoom_menos_clicked()
{
    pixmapOffset.setX(0);
    pixmapOffset.setY(0);
    zoomfactor--;
    if (zoomfactor==0)
    {
        ui->zoom_menos->setDisabled(true);
    }
    // else
    //     ui->zoom_menos->setDisabled(false);
    QPixmap copia_menos;
    copia_menos=imagen_directo.copy();
    QPainter painter_menos(this);
    painter_menos.begin(&copia_menos);
    painter_menos.scale(zoomfactor,zoomfactor);
    QRectF target_menos(0.0, 0.0, copia_menos.width(), copia_menos.height());

  //  QRectF source_menos(copia_menos.width()/4, copia_menos.height()/4, copia_menos.width()*0.5, copia_menos.height()*.5);
    QRectF source_menos((copia_menos.width()*zoomfactor*0.1), (copia_menos.height()*zoomfactor*0.1), (copia_menos.width()*(1-0.1*zoomfactor)), (copia_menos.height()*(1-0.1*zoomfactor)));
    //QPixmap pixmap(":myPixmap.png");

    //QPainter(this);
    painter_menos.drawPixmap(target_menos, copia_menos, source_menos);

    painter_menos.end();
    ui->Imagen_1->setPixmap(copia_menos);
    update();
    printf("zoom menos factor %d\n",zoomfactor);

}




int MainWindow::captureFunction(CvCapture *capture2, IplImage *imgCapturedFunct, int indiceQ,char *image_filenameFunction,char *imageKindFunction)
{
    //imgCapturedFunct = NULL;
    //CvCapture* capture2;
    //capture2=NULL;
    //    printf("input %d\n", input);

    if (input==1)
    {

        if(!cvGrabFrame(capture2))
        {
            printf("No se puede grabar frame\n");
      //      exit(0);
            return 0;

        }
        else
            imgCapturedFunct=cvRetrieveFrame(capture2);
    }
    if (input==2)
    {
        imgCapturedFunct=cvQueryFrame(capture2);
    }
    if (input==3)
    {
        std::string str= image_filenameOpenCV.toStdString();

        str= image_filenameOpenCV.toStdString();
        sprintf(image_filenameFunction,"%s%04d.%s",str.c_str(),indiceQ,imageKindFunction);
        imgCapturedFunct=cvLoadImage(image_filenameFunction);
        if( ! imgCapturedFunct )
        {

            printf( "unable to load image B from %s", image_filenameFunction );
        }
    }


    //                    cvNamedWindow( "imageC", 1 );
    //                    cvShowImage( "imageC", imgCapturedFunct ); // se muestran las imagenes en dichas ventanas
    //                    cvMoveWindow( "imageC",0, 0 );
    //                    cvWaitKey(0);
    //                    cvDestroyWindow("imageC");



}


void MainWindow::on_saveVideo_clicked() // para poner un nombre al video que se grave con la camara en directo
{
/*    const char *videoName;
    videoName=NULL;
    QString VideoNameQS;
    VideoName getVideoNameWindow;
    if (getVideoNameWindow.exec()){
        VideoNameQS=getVideoNameWindow.getName();
        std::string videoNameStr;
        videoNameStr=VideoNameQS.toStdString();
        videoName=videoNameStr.c_str();
    }


    printf("nuevo video %s\n",videoName);

    if (videoName!=NULL){
        saveVideo=1;
        char secuenciaVideo[128];

        sprintf(secuenciaVideo," ffmpeg -f image2 -i ./imagenes/mosaico%%4d.tif %s.avi ", videoName);

        system(secuenciaVideo);

    }*/

//    const char *videoName;

    char videoName[80];
    time(&rawtime);
    timeinfo=localtime(&rawtime);


    strftime( videoName,80,"%Y_%m_%d_%H:%M.avi",timeinfo);

//    printf("The current date/time is: %s", asctime (timeinfo) );


    printf("hola\n");
                printf("The current date/time is: %s\n", videoName );

//        videoName=asctime(timeinfo);


        if (videoName!=NULL){
            saveVideo=1;
            char secuenciaVideo[128];

            sprintf(secuenciaVideo," ffmpeg -f image2 -i ./imagenes/mosaico%%4d.tif %s", videoName);

            system(secuenciaVideo);

        }


}

void MainWindow::on_pauseButton_clicked()
{

    ui->playButton->setDisabled(false);
    ui->pauseButton->setDisabled(true);
    ui->stopButton->setDisabled(false);

    fromPause=1;
    pauseVideo=1;

    pauseTime=tiempo_salida.elapsed();
    //tiempoPausa=tiempo_captura.currentTime();


    printf("pause\n");
}





//void MainWindow::on_actionBlackMagic_triggered()
//{
//    if (blackMagicOn==0)
//    {
//       blackMagicOn=1;
//    }
//    else
//    {
//        blackMagicOn=0;
//        //blackMagic.closeCamera();
//    }
//}

void MainWindow::actualizaEstadoCaptura(int resultado)
{

    this->resultado=resultado; // para que copie la variable y sea la misma en los dos lados.
    status=resultado;

    printf("resultado 2 %d\n",resultado);

    switch (status)
    {
   /* case 0: {
            printf("no se que hacer\n");
            break;
        };
    case 1:{ printf("ok\n");
            process(frameI,frameF);
            break;
        }*/
    case 2: {
            QMessageBox *noBlackMaficDrivers;
            noBlackMaficDrivers =new QMessageBox;
            noBlackMaficDrivers->setModal(false);

            noBlackMaficDrivers->setText("Capture Error: No Black Magic Drivers on your Computer");
            noBlackMaficDrivers->show();
            printf("faltan drivers\n");
            break;
        }
    case 3: {
            QMessageBox *noPCICard;
            noPCICard =new QMessageBox;
            noPCICard->setModal(false);

            noPCICard->setText("Capture Error: No PCI Card on your Computer");
            noPCICard->show();

            printf("no hay tarjeta PCI\n");
            break;
        }
    case 4: {
            QMessageBox *captError;
            captError =new QMessageBox;
            captError->setModal(false);

            captError->setText("Undefined capture error: Please check your camera settings");
            captError->show();

            printf("error de captura\n");
            break;
        }
    case 5:
        {
            QMessageBox *otherApiRunning;
            otherApiRunning =new QMessageBox;
            otherApiRunning->setModal(false);

            otherApiRunning->setText("Capture Error: Another process is using the PCI Card, please close this process before starting capturing");
            otherApiRunning->show();

            printf("Otra aplicacion tiene la tarjeta\n");
            break;
        }
    default:
        {
             process(frameI,frameF);
        }
    }


}
