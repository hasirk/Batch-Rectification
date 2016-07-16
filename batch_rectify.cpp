/*
 *  batch_rectify.cpp
 *  after calibration
 *
 *  Created by Zhang Handuo on 3/08/2016.
 *  NTU Robotics
 *
 */

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
 #include <opencv2/video/video.hpp>
 #include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#ifdef LINUX
#include <unistd.h>
#endif
#include <stdio.h>
#include <string>
#include <ctime>

using namespace cv;

bool CreateDirectory(char *datechar, char *timechar);


char datestr[20];
char timestr[20];
time_t currentdate = time(NULL);
time_t now;
struct tm *timenow;
char folderstring[14];
char finalarray[50];
int cut = 0;
int cull = 0;

int main(int argc, char** argv)
{
    const char* scale_opt = "--scale=";
    const char* intrinsic_filename = 0;
    const char* extrinsic_filename = 0;
    std::string left_filename_fmt, right_filename_fmt;
    char left_filename_r[128], right_filename_r[128];
    int num = 1000;
    int index = 0;
    char buf[1024];
    float scale = 1.f;
    Rect validRoi1;
    Rect validRoi2;
    // left_filename_fmt = argv[1];
    // right_filename_fmt = argv[2];
    for( int i = 1; i < argc; i++ )
    {
        if( strncmp(argv[i], scale_opt, strlen(scale_opt)) == 0 )
        {
            if( sscanf( argv[i] + strlen(scale_opt), "%f", &scale ) != 1 || scale < 0 )
            {
                printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
                return -1;
            }
        }
        else if( strcmp(argv[i], "-l" ) == 0 )
            left_filename_fmt = argv[++i];
        else if( strcmp(argv[i], "-r" ) == 0 )
            right_filename_fmt = argv[++i];        
        else if( strcmp(argv[i], "-i" ) == 0 )
            intrinsic_filename = argv[++i];
        else if( strcmp(argv[i], "-e" ) == 0 )
            extrinsic_filename = argv[++i];
        else if( strcmp(argv[i], "-n" ) == 0 )
            num = atoi(argv[++i]);   
        else if( strcmp(argv[i], "-cut" ) == 0 )
            cut = atoi(argv[++i]);
        else if( strcmp(argv[i], "-cull" ) == 0 )
            cull = atoi(argv[++i]);
        else
        {
            printf("Command-line parameter error: unknown option %s\n", argv[i]);
            return -1;
        }
    }
    if( (intrinsic_filename != 0) ^ (extrinsic_filename != 0) )
    {
        printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return -1;
    }
    time(&now);
    strftime(datestr, 20, "%b. %d, %Y", localtime(&currentdate));
    timenow = localtime(&now);
    strftime(timestr,20,"%H %M",timenow);
    if(!CreateDirectory(datestr,timestr))
    {
        printf("Directory is not created!");
        return -1;
    }

    for (index = 0; index < num ; index++){

        sprintf(buf, left_filename_fmt.c_str(), index);
        cv::Mat img1 = cv::imread(buf, -1);
        sprintf(buf, right_filename_fmt.c_str(), index);
        cv::Mat img2 = cv::imread(buf, -1);
        cv::Mat img1f, img2f;

        if( scale != 1.f )
        {
            Mat temp1, temp2;
            int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
            resize(img1, temp1, Size(), scale, scale, method);
            img1 = temp1;
            resize(img2, temp2, Size(), scale, scale, method);
            img2 = temp2;
        }

        Size img_size = img1.size();

        Rect roi1, roi2;
        Mat Q;
        validRoi1 = Rect( cull, cut + 0, img_size.width - cull , img_size.height - cut - cut );
        validRoi2 = Rect( 0 , cut + 0, img_size.width - cull , img_size.height - cut - cut );        
        if( intrinsic_filename )
        {
            // reading intrinsic parameters
            FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
            if(!fs.isOpened())
            {
                printf("Failed to open file %s\n", intrinsic_filename);
                return -1;
            }

            Mat M1, D1, M2, D2;
            fs["M1"] >> M1;
            fs["D1"] >> D1;
            fs["M2"] >> M2;
            fs["D2"] >> D2;

            M1 *= scale;
            M2 *= scale;

            fs.open(extrinsic_filename, CV_STORAGE_READ);
            if(!fs.isOpened())
            {
                printf("Failed to open file %s\n", extrinsic_filename);
                return -1;      
            }

            Mat R, T, R1, P1, R2, P2;
            fs["R"] >> R;
            fs["T"] >> T;

            stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

            Mat map11, map12, map21, map22;
            initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
            initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

            Mat img1r, img2r;
            remap(img1, img1r, map11, map12, INTER_LINEAR);
            remap(img2, img2r, map21, map22, INTER_LINEAR);

            img1f = img1r(validRoi1);
            img2f = img2r(validRoi2);
        }
        sprintf(left_filename_r,"%s/L%05d.pgm",finalarray,index);
        sprintf(right_filename_r,"%s/R%05d.pgm",finalarray,index);
       // Mat disp, disp8;                              
        cv::imwrite(left_filename_r, img1f);
        cv::imwrite(right_filename_r, img2f);   

    }                             


    return 0;
}

bool CreateDirectory(char *datechar, char *timechar)
{
    char* datestr = datechar;
    char* timestr = timechar;
    char folderstring[14];
    bool isDir;

    folderstring[0]=datestr[0];
    folderstring[1]=datestr[1];
    folderstring[2]=datestr[2];
    folderstring[3]=datestr[5];
    folderstring[4]=datestr[6];
    folderstring[5]=datestr[9];
    folderstring[6]=datestr[10];
    folderstring[7]=datestr[11];
    folderstring[8]=datestr[12];
    folderstring[9]=timestr[0];
    folderstring[10]=timestr[1];
    folderstring[11]=':';
    folderstring[12]=timestr[3];
    folderstring[13]=timestr[4];    

    printf("folderstring is %s \n",folderstring);
    char *home = getenv("HOME");
    char pathex[30];
    sprintf(pathex,"%s/rectified",home);
    if(NULL == opendir(pathex))
    mkdir(pathex,0777);
    sprintf(finalarray,"%s/%s",pathex,folderstring);
    mkdir(finalarray,0777);
    //makeing files
    //actually here we should check whether the directory exists using stat()
    struct stat statbuf;
    if (stat(finalarray, &statbuf) != -1) {
       if (S_ISDIR(statbuf.st_mode)) {
            return true;
       }
    } else {
                return false;
    }
}
