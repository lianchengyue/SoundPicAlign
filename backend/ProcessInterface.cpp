#include "ProcessInterface.h"

ProcessInterface::ProcessInterface(LogReader * logRead, cv::Mat * Intrinsics)
 : ThreadObject("ProcessInterfaceThread"),
   endRequested(false),
   logRead(logRead),
   currentFrame(0),
   firstRun(true)
{
    printf("ProcessInterface\n");
    frontend = new FrontProcessor(Intrinsics);
    reset();

    calcRMatrix();
    calcTMatrix();
    calcCameraPose();

///    calc2DCoordinate(Intrinsics);

    process(Intrinsics); //flq.temp
}

ProcessInterface::~ProcessInterface()
{
    delete frontend;
}

void ProcessInterface::reset()
{
    currentFrame = 0;
    frontend->reset();
}

bool inline ProcessInterface::process(cv::Mat * Intrinsics)
{
    printf("ProcessInterface::process()\n");

    vector<Point2f> p2d;

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

        if(!logRead->grabNext(returnVal, currentFrame)/* || shouldEnd*/)
        {
            threadPack.pauseCapture.assignValue(true);
            threadPack.finalised.assignValue(true);

//            finalise();

//            while(!threadPack.cloudSliceProcessorFinished.getValueWait())
//            {
//                frontend->cloudSignal.notify_all();
//            }

            return shouldEnd ? false : returnVal;
        }

///        rgb24.data = (PixelRGB *)logRead->decompressedImage;
        
        currentFrame++;

///        rgb24.step = Resolution::get().width() * 3;
///        rgb24.rows = Resolution::get().rows();
///        rgb24.cols = Resolution::get().cols();

///        colors_device.upload(rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);

        TICK("processFrame");
/*
        frontend->processFrame(logRead->decompressedImage,   //flq, rgbImage
                               logRead->decompressedDepth,
                               logRead->timestamp,
                               logRead->isCompressed,
                               logRead->compressedDepth,
                               logRead->compressedDepthSize,
                               logRead->compressedImage,
                               logRead->compressedImageSize);
*/
        //定位到像素坐标系,p2d为输出
        calc2DCoordinate(Intrinsics, p2d);

        //加水印
        frontend->processFrame(logRead->decompressedImage, p2d);
        TOCK("processFrame");

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

bool  ProcessInterface::calcRMatrix()
{
    //相机的初始欧拉角度
    Vec3f angle = Vec3f(0, 0, 0);
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
#if 0
    ThreadDataPack::get().finalpose(0,0) = 1;
    ThreadDataPack::get().finalpose(0,1) = 0;
    ThreadDataPack::get().finalpose(0,2) = 0;
    ThreadDataPack::get().finalpose(0,3) = 0;//-6.9917633511038035e+00;

    ThreadDataPack::get().finalpose(1,0) = 0;
    ThreadDataPack::get().finalpose(1,1) = 1;
    ThreadDataPack::get().finalpose(1,2) = 0;
    ThreadDataPack::get().finalpose(1,3) = 2.2f;//2.5151918580111243e-01;

    ThreadDataPack::get().finalpose(2,0) = 0;
    ThreadDataPack::get().finalpose(2,1) = 0;
    ThreadDataPack::get().finalpose(2,2) = 1;
    ThreadDataPack::get().finalpose(2,3) = 0;//8.6742681541653210e-02;

    ThreadDataPack::get().finalpose(3,0) = 0;
    ThreadDataPack::get().finalpose(3,1) = 0;
    ThreadDataPack::get().finalpose(3,2) = 0;
    ThreadDataPack::get().finalpose(3,3) = 1;
    std::cout<< "test pose:" << ThreadDataPack::get().finalpose << std::endl;
#endif
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

    std::cout<< "test pose:" << ThreadDataPack::get().finalpose << std::endl;
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
int ProcessInterface::calc2DCoordinate(cv::Mat* Intrinsics, vector<Point2f>& points2d)
{
    //旋转向量
    Mat rvec;
    //畸变系数
    Mat distCoeff(1,5,CV_64F,Scalar(0));//CV_32FC1

    //旋转矩阵转换为旋转向量
    Rodrigues(ThreadDataPack::get().RMatrix, rvec);

#if 1
    distCoeff.at<double>(0, 0) = -0.2756734608366758;
    distCoeff.at<double>(0, 1) = -0.001303202285062331;
    distCoeff.at<double>(0, 2) = 0.001005134230599892;
    distCoeff.at<double>(0, 3) = -0.0008562559253269711;
    distCoeff.at<double>(0, 4) = 0.2240573152028148;
#else
#endif
    /*
    float vals[] = {525., 0., 3.1950000000000000e+02,
                    0., 525., 2.3950000000000000e+02,
                    0., 0., 1.};
    Mat cameraMatrix = Mat(3,3,CV_64F,vals);//CV_32FC1
    */

    //世界坐标系中的一个点
///    vector<Point2f> points2d;
    //像素坐标系中的一个点
    vector<Point3f> points3d(1);

//    points3d[0].x = 10.f;
//    points3d[0].y = 10.f;
//    points3d[0].z = 10.f;
    //points3d[0] = Point3f((double)16.3f, (double)2.2f, (double)2.2f); //set x
    points3d[0] = Point3f((double)0.0f, (double)2.2f, (double)24.6f); //set x

    projectPoints(Mat(points3d), rvec, ThreadDataPack::get().TMatrix, *Intrinsics/*cameraMatrix*/, distCoeff, points2d);

    std::cout << "Final: calc2DCoordinate(), result:" << std::endl << points2d << std::endl;

    return 0;
}
