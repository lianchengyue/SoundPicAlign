#include "ProcessInterface.h"

ProcessInterface::ProcessInterface(LogReader * logRead, cv::Mat * Intrinsics)
 : ThreadObject("ProcessInterfaceThread"),
   endRequested(false),
   logRead(logRead),
   frame_idx(0),
   firstRun(true)
{
    printf("ProcessInterface\n");
    frontend = new FrontProcessor(Intrinsics);
    reset();

    //
    euler_arc_X = 2*PI*CAMERA_EULER_X/360.f;
    euler_arc_Y = 2*PI*CAMERA_EULER_Y/360.f;
    euler_arc_Z = 2*PI*CAMERA_EULER_Z/360.f;
    printf("euler_arc_X=%f, euler_arc_Y=%f, euler_arc_Z=%f\n", euler_arc_X, euler_arc_Y, euler_arc_Z);

    calcRMatrix();
    calcTMatrix();
    calcCameraPose();

///    process(Intrinsics); //flq.temp
}

ProcessInterface::~ProcessInterface()
{
    delete frontend;
}

void ProcessInterface::reset()
{
    frame_idx = 0;
    frontend->reset();
}

//预览video的函数
bool /*inline*/ ProcessInterface::process()
{
#ifdef REALTIME_DEBUG
    printf("ProcessInterface::process() start!\n");
#endif
    if(firstRun)
    {
        firstRun = false;
    }

    if(!threadPack.pauseCapture.getValue())
    {
        TICK(threadIdentifier);

        uint64_t start = Stopwatch::getCurrentSystemTime();

        bool returnVal = true;

        bool shouldEnd = endRequested.getValue();

        if(!logRead->grabNext(returnVal, frame_idx) || shouldEnd)
        {
//            threadPack.pauseCapture.assignValue(true);
//            threadPack.finalised.assignValue(true);

//            finalise();

            return shouldEnd ? false : returnVal;
        }

        //
        //flqq, new processFrame(, in FrontProcessor
        frontend->processVideoFrame(logRead->decompressedImage);
        frame_idx++;

        uint64_t duration = Stopwatch::getCurrentSystemTime() - start;

        if(threadPack.limit.getValue() && duration < 33333)
        {
            int sleepTime = std::max(int(33333 - duration), 0);
            usleep(sleepTime);
        }

        TOCK(threadIdentifier);
    }

    return true;
}

//触发定位时的处理函数
bool /*inline*/ ProcessInterface::process(cv::Mat * Intrinsics, vector<Point3f> p3d)
{
#if 1
    printf("ProcessInterface::process()\n");

    vector<Point2f> p2d;
    vector<Point2f> p2d_Cam2;

    if(firstRun)
    {
//        cudaSafeCall(cudaSetDevice(ConfigArgs::get().gpu));
        firstRun = false;
    }

    if(!threadPack.pauseCapture.getValue())
    {
        TICK(threadIdentifier);

        uint64_t start = Stopwatch::getCurrentSystemTime();

        bool returnVal = true;

        bool shouldEnd = endRequested.getValue();

        if(!logRead->grabTrigerdNext(returnVal/*, frame_idx*/)/* || shouldEnd*/)
        {
            threadPack.pauseCapture.assignValue(true);
            threadPack.finalised.assignValue(true);

            return shouldEnd ? false : returnVal;
        }


        TICK("processTrigerdFrame");

        //定位到像素坐标系,p2d为输出
        calc2DCoordinate(Intrinsics, p3d, p2d);
        calcSecondCam2DCoordinate(Intrinsics, p3d, p2d_Cam2);

        //加水印
        frontend->processFrame(logRead->TrigerSavedImage, p2d);//decompressedImage
        TOCK("processTrigerdFrame");

        uint64_t duration = Stopwatch::getCurrentSystemTime() - start;

        if(threadPack.limit.getValue() && duration < 33333)
        {
            int sleepTime = std::max(int(33333 - duration), 0);
            usleep(sleepTime);
        }

        TOCK(threadIdentifier);
    }
#endif
    return true;
}

bool  ProcessInterface::calcRMatrix()
{
    //相机的初始欧拉角度
    Vec3f angle = Vec3f(euler_arc_X, euler_arc_Y, euler_arc_Z);
    ThreadDataPack::get().RMatrix  = frontend->eulerAnglesToRotationMatrix(angle);
    std::cout<< "R:" << std::endl <<ThreadDataPack::get().RMatrix<<std::endl;

    return 0;
}

void InitMat(Mat& m,float* num)
{
    for(int i=0;i<m.rows;i++)
        for(int j=0;j<m.cols;j++)
            m.at<float>(i,j)=*(num+i*m.rows+j);
}

bool  ProcessInterface::calcTMatrix()
{
    float m0[]={ 1,2,3,
        6,5,4,
        7,8,9 };
    Mat M0(3,3,CV_32F);
    InitMat(M0,m0);

    ThreadDataPack::get().TMatrix  = frontend->setTMatrix();

    std::cout<< "T:" << std::endl <<ThreadDataPack::get().TMatrix<<std::endl;

    return 0;
}

bool ProcessInterface::calcCameraPose(/*Eigen::Matrix4f& pose*/)
{
    //R
    ThreadDataPack::get().finalpose(0,0) = ThreadDataPack::get().RMatrix.at<double>(0, 0);
    ThreadDataPack::get().finalpose(0,1) = ThreadDataPack::get().RMatrix.at<double>(0, 1);
    ThreadDataPack::get().finalpose(0,2) = ThreadDataPack::get().RMatrix.at<double>(0, 2);

    ThreadDataPack::get().finalpose(1,0) = ThreadDataPack::get().RMatrix.at<double>(1, 0);
    ThreadDataPack::get().finalpose(1,1) = ThreadDataPack::get().RMatrix.at<double>(1, 1);
    ThreadDataPack::get().finalpose(1,2) = ThreadDataPack::get().RMatrix.at<double>(1, 2);

    ThreadDataPack::get().finalpose(2,0) = ThreadDataPack::get().RMatrix.at<double>(2, 0);
    ThreadDataPack::get().finalpose(2,1) = ThreadDataPack::get().RMatrix.at<double>(2, 1);
    ThreadDataPack::get().finalpose(2,2) = ThreadDataPack::get().RMatrix.at<double>(2, 2);

    ThreadDataPack::get().finalpose(3,3) = 1;

    //T
    ThreadDataPack::get().finalpose(0,3) = ThreadDataPack::get().TMatrix.at<double>(0, 0);
    ThreadDataPack::get().finalpose(1,3) = ThreadDataPack::get().TMatrix.at<double>(0, 1);
    ThreadDataPack::get().finalpose(2,3) = ThreadDataPack::get().TMatrix.at<double>(0, 2);

#if 0
    //R
    ThreadDataPack::get().finalpose(0,0) = ThreadDataPack::get().RMatrix.at<double>(0, 0);
    ThreadDataPack::get().finalpose(0,1) = ThreadDataPack::get().RMatrix.at<double>(0, 1);
    ThreadDataPack::get().finalpose(0,2) = -ThreadDataPack::get().RMatrix.at<double>(0, 2);

    ThreadDataPack::get().finalpose(1,0) = ThreadDataPack::get().RMatrix.at<double>(1, 0);
    ThreadDataPack::get().finalpose(1,1) = ThreadDataPack::get().RMatrix.at<double>(1, 1);
    ThreadDataPack::get().finalpose(1,2) = ThreadDataPack::get().RMatrix.at<double>(1, 2);

    ThreadDataPack::get().finalpose(2,0) = -ThreadDataPack::get().RMatrix.at<double>(2, 0);
    ThreadDataPack::get().finalpose(2,1) = ThreadDataPack::get().RMatrix.at<double>(2, 1);
    ThreadDataPack::get().finalpose(2,2) = ThreadDataPack::get().RMatrix.at<double>(2, 2);

    ThreadDataPack::get().finalpose(3,3) = 1;

    //T
    ThreadDataPack::get().finalpose(0,3) = ThreadDataPack::get().TMatrix.at<double>(0, 0);
    ThreadDataPack::get().finalpose(1,3) = -ThreadDataPack::get().TMatrix.at<double>(0, 1);
    ThreadDataPack::get().finalpose(2,3) = ThreadDataPack::get().TMatrix.at<double>(0, 2);
#endif
    std::cout<< std::endl << "test pose:" << std::endl << ThreadDataPack::get().finalpose << std::endl;
    return 0;
}

//flq,3D坐标投影到2维图像
/*/
 * oid projectPoints(InputArray objectPoints, InputArray rvec, InputArray tvec, InputArray cameraMatrix, InputArray distCoeffs, OutputArray imagePoints, OutputArray jacobian=noArray(), double aspectRatio=0 )
 //Function: (Projects 3D points to an image plane)
 * objectPoints – Array of object points, 3xN/Nx3 1-channel or 1xN/Nx1 3-channel (or vector<Point3f> ), where N is the number of points in the view.
 * rvec – Rotation vector. See Rodrigues() for details.
 * tvec – Translation vector.
 * cameraMatrix – Camera matrix A = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{_1} .
 * distCoeffs – Input vector of distortion coefficients (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) of 4, 5, or 8 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
 * imagePoints – Output array of image points, 2xN/Nx2 1-channel or 1xN/Nx1 2-channel, or vector<Point2f> .
 * jacobian – Optional output 2Nx(10+<numDistCoeffs>) jacobian matrix of derivatives of image points with respect to components of the rotation vector, translation vector, focal lengths, coordinates of the principal point and the distortion coefficients. In the old interface different components of the jacobian are returned via different output parameters.
 * aspectRatio – Optional “fixed aspect ratio” parameter. If the parameter is not 0, the function assumes that the aspect ratio (fx/fy) is fixed and correspondingly adjusts the jacobian matrix.
*/
int ProcessInterface::calc2DCoordinate(cv::Mat* Intrinsics, vector<Point3f> points3d, vector<Point2f>& points2d)
{
    //旋转向量
    Mat rvec;
    //畸变系数
    Mat distCoeff(1,5,CV_64F,Scalar(0));//CV_32FC1

    //旋转矩阵转换为旋转向量
    Rodrigues(ThreadDataPack::get().RMatrix, rvec);

    std::cout << "rvec:" << rvec << std::endl;

#ifdef HIKVISION_GRAB_FUNC
///*
    //4mm, 1920x1080
    distCoeff.at<double>(0, 0) = -0.3842780507597445;
    distCoeff.at<double>(0, 1) = -0.01026628950727477;
    distCoeff.at<double>(0, 2) = -0.001097510941657248;
    distCoeff.at<double>(0, 3) = 0.0002966871455004014;
    distCoeff.at<double>(0, 4) = 0.7321469514653421;
//*/
/*
    //12mm, 4096x2160
    distCoeff.at<double>(0, 0) = -0.1829310138260891;
    distCoeff.at<double>(0, 1) = 0.921006884774823;
    distCoeff.at<double>(0, 2) = -0.006458022608597794;
    distCoeff.at<double>(0, 3) = 0.007211980182173494;
    distCoeff.at<double>(0, 4) = -3.849852944304393;
*/
/*
    //25mm, 4096x2160
    distCoeff.at<double>(0, 0) = -0.892716031334611;
    distCoeff.at<double>(0, 1) = 12.98805686689792;
    distCoeff.at<double>(0, 2) = 0.01352535548838887;
    distCoeff.at<double>(0, 3) = -0.02306571153764621;
    distCoeff.at<double>(0, 4) = -92.62687091981161;
*/
#else
    distCoeff.at<double>(0, 0) = -0.2756734608366758;
    distCoeff.at<double>(0, 1) = -0.001303202285062331;
    distCoeff.at<double>(0, 2) = 0.001005134230599892;
    distCoeff.at<double>(0, 3) = -0.0008562559253269711;
    distCoeff.at<double>(0, 4) = 0.2240573152028148;
#endif

//added for 三维重建中旋转矩阵与平移矩阵思想误区
    //https://blog.csdn.net/weixin_34346607/article/details/52988221
    Mat CMatrix=Mat(3,3,CV_64F,Scalar::all(0));
    //R的逆的转置
    CMatrix = ThreadDataPack::get().RMatrix.inv();
    //矩阵的转置
    transpose(CMatrix, CMatrix);

    Mat tvec = Mat_<double>(1,3);
    tvec = /*-*/ThreadDataPack::get().TMatrix * CMatrix;
    //std::cout<< "True T:" << std::endl << tvec <<std::endl;
//added end

    /*
    float vals[] = {525., 0., 3.1950000000000000e+02,
                    0., 525., 2.3950000000000000e+02,
                    0., 0., 1.};
    Mat cameraMatrix = Mat(3,3,CV_64F,vals);//CV_32FC1
    */

    //世界坐标系中的一个点
////    vector<Point2f> points2d;
    //像素坐标系中的一个点
////    vector<Point3f> points3d(1);

//    points3d[0].x = 10.f;
//    points3d[0].y = 10.f;
//    points3d[0].z = 10.f;
    //points3d[0] = Point3f((double)16.3f, (double)2.2f, (double)2.2f); //set x
////    points3d[0] = Point3f((double)0.0f, (double)2.2f, (double)24.6f); //set x
    std::cout << "points3d:" << points3d << std::endl;

    projectPoints(Mat(points3d), rvec, tvec, *Intrinsics/*cameraMatrix*/, distCoeff, points2d);

    std::cout << "Final: calc2DCoordinate(), result:" << std::endl << points2d << std::endl;

    return 0;
}


int ProcessInterface::calcSecondCam2DCoordinate(cv::Mat * Intrinsics, vector<Point3f> points3d, vector<Point2f>& points2d)
{
    //畸变系数
    Mat distCoeff(1,5,CV_64F,Scalar(0));
    //4mm, 1920x1080
    distCoeff.at<double>(0, 0) = -0.3842780507597445;
    distCoeff.at<double>(0, 1) = -0.01026628950727477;
    distCoeff.at<double>(0, 2) = -0.001097510941657248;
    distCoeff.at<double>(0, 3) = 0.0002966871455004014;
    distCoeff.at<double>(0, 4) = 0.7321469514653421;

    Mat CMatrix=Mat(3,3,CV_64F,Scalar::all(0));
    //R的逆的转置
    CMatrix = ThreadDataPack::get().RMatrix.inv();
    //矩阵的转置
    transpose(CMatrix, CMatrix);

    Mat tvec = Mat_<double>(1,3);
    tvec = /*-*/ThreadDataPack::get().TMatrix * CMatrix;
    std::cout << "FLQ tvec:" << std::endl << tvec << std::endl;

    ///第一次旋转
    Mat RT1 = Mat(4,4,CV_64F,Scalar::all(0));
    RT1.at<double>(0, 0) = ThreadDataPack::get().RMatrix.at<double>(0, 0);
    RT1.at<double>(0, 1) = ThreadDataPack::get().RMatrix.at<double>(0, 1);
    RT1.at<double>(0, 2) = ThreadDataPack::get().RMatrix.at<double>(0, 2);
    RT1.at<double>(0, 3) = tvec.at<double>(0, 0);

    RT1.at<double>(1, 0) = ThreadDataPack::get().RMatrix.at<double>(1, 0);
    RT1.at<double>(1, 1) = ThreadDataPack::get().RMatrix.at<double>(1, 1);
    RT1.at<double>(1, 2) = ThreadDataPack::get().RMatrix.at<double>(1, 2);
    RT1.at<double>(1, 3) = tvec.at<double>(0, 1);

    RT1.at<double>(2, 0) = ThreadDataPack::get().RMatrix.at<double>(2, 0);
    RT1.at<double>(2, 1) = ThreadDataPack::get().RMatrix.at<double>(2, 1);
    RT1.at<double>(2, 2) = ThreadDataPack::get().RMatrix.at<double>(2, 2);
    RT1.at<double>(2, 3) = tvec.at<double>(0, 2);

    RT1.at<double>(3, 0) = 0;
    RT1.at<double>(3, 1) = 0;
    RT1.at<double>(3, 2) = 0;
    RT1.at<double>(3, 3) = 1;

    ///再次旋转平移,C = RT x W
    //Input R2
    Mat R2 = Mat(3,3,CV_64F,Scalar::all(0));
    //写入旋转矩阵:
    //R2.at<double>(0, 0) = 1;
    //R2.at<double>(0, 1) = 0;
    //R2.at<double>(0, 2) = 0;
    //R2.at<double>(1, 0) = 0;
    //R2.at<double>(1, 1) = 1;
    //R2.at<double>(1, 2) = 0;
    //R2.at<double>(2, 0) = 0;
    //R2.at<double>(2, 1) = 0;
    //R2.at<double>(2, 2) = 1;
    R2.at<double>(0, 0) = 9.9895269554121646e-001;
    R2.at<double>(0, 1) = 3.7410010228289346e-004;
    R2.at<double>(0, 2) = 4.5753383700566939e-002;
    R2.at<double>(1, 0) = -2.0569609566232247e-004;
    R2.at<double>(1, 1) = 9.9999318794707170e-001;
    R2.at<double>(1, 2) = -3.6853423950571969e-003;
    R2.at<double>(2, 0) = -4.5754450713062465e-002;
    R2.at<double>(2, 1) = 3.6720714271441632e-003;
    R2.at<double>(2, 2) = 9.9894596757351195e-001;

    //Input T2
    Mat T2 = Mat(1,3,CV_64F,Scalar::all(0));
    //写入平移矩阵
    //T2.at<double>(0, 0) = 0;
    //T2.at<double>(0, 1) = 0;
    //T2.at<double>(0, 2) = 0;
    T2.at<double>(0, 0) = -0.8773498;
    T2.at<double>(0, 1) = 0.0011230;
    T2.at<double>(0, 2) = 0.0000436;

    Mat RT2 = Mat(4,4,CV_64F,Scalar::all(0));
    RT2.at<double>(0, 0) = R2.at<double>(0, 0);
    RT2.at<double>(0, 1) = R2.at<double>(0, 1);
    RT2.at<double>(0, 2) = R2.at<double>(0, 2);
    RT2.at<double>(0, 3) = T2.at<double>(0, 0);

    RT2.at<double>(1, 0) = R2.at<double>(1, 0);
    RT2.at<double>(1, 1) = R2.at<double>(1, 1);
    RT2.at<double>(1, 2) = R2.at<double>(1, 2);
    RT2.at<double>(1, 3) = T2.at<double>(0, 1);

    RT2.at<double>(2, 0) = R2.at<double>(2, 0);
    RT2.at<double>(2, 1) = R2.at<double>(2, 1);
    RT2.at<double>(2, 2) = R2.at<double>(2, 2);
    RT2.at<double>(2, 3) = T2.at<double>(0, 2);

    RT2.at<double>(3, 0) = 0;
    RT2.at<double>(3, 1) = 0;
    RT2.at<double>(3, 2) = 0;
    RT2.at<double>(3, 3) = 1;

    ///最终旋转平移
    Mat RTFinal = RT1 * RT2;
    std::cout<< "最终旋转平移:" << std::endl << RTFinal <<std::endl;

    //相机2的旋转平移向量
    //旋转向量
    Mat rvecFinal;

    Mat RFinal = Mat(3,3,CV_64F,Scalar::all(0));
    RFinal.at<double>(0, 0) = RTFinal.at<double>(0, 0);
    RFinal.at<double>(0, 1) = RTFinal.at<double>(0, 1);
    RFinal.at<double>(0, 2) = RTFinal.at<double>(0, 2);
    RFinal.at<double>(1, 0) = RTFinal.at<double>(1, 0);
    RFinal.at<double>(1, 1) = RTFinal.at<double>(1, 1);
    RFinal.at<double>(1, 2) = RTFinal.at<double>(1, 2);
    RFinal.at<double>(2, 0) = RTFinal.at<double>(2, 0);
    RFinal.at<double>(2, 1) = RTFinal.at<double>(2, 1);
    RFinal.at<double>(2, 2) = RTFinal.at<double>(2, 2);

    Mat TFinal = Mat(1,3,CV_64F,Scalar::all(0));
    TFinal.at<double>(0, 0) = RTFinal.at<double>(0, 3);
    TFinal.at<double>(0, 1) = RTFinal.at<double>(1, 3);
    TFinal.at<double>(0, 2) = RTFinal.at<double>(2, 3);

    Rodrigues(RFinal, rvecFinal);

    //投影
    projectPoints(Mat(points3d), rvecFinal, TFinal, *Intrinsics/*cameraMatrix*/, distCoeff, points2d);

    std::cout << "Final: calcSecondCam2DCoordinate(), result:" << std::endl << points2d << std::endl;

    return 0;
}
