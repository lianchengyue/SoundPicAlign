#ifndef PROCESSINTERFACE_H
#define PROCESSINTERFACE_H

#include <poll.h>
#include <opencv2/opencv.hpp>
#include <vector>

//#include "../frontend/Volume.h"
#include "../frontend/FrontProcessor.h"

#include "../utils/ThreadObject.h"
#include "../utils/LogReader.h"

class ProcessInterface : public ThreadObject
{
    public:
        ProcessInterface(LogReader * logRead, cv::Mat * Intrinsics);

        virtual ~ProcessInterface();

        void reset();

        FrontProcessor * getFrontend()
        {
            return frontend;
        }

        void finalise()
        {
//            frontend->finalise();
        }

        void setPark(const bool park)
        {
//            frontend->setParked(park);
        }

        void enableOverlap()
        {
//            frontend->setOverlap(2);
        }


        ThreadMutexObject<bool> endRequested;
        bool /*inline*/ process(cv::Mat * Intrinsics, vector<Point3f> p3d);
        bool /*inline*/ process(cv::Mat * Intrinsics);

    private:
//        bool inline process(cv::Mat * Intrinsics);
        bool calcRMatrix();
        bool calcTMatrix();
        bool calcCameraPose(/*Eigen::Matrix4f& pose*/);
        //Intrinsics:相机内参
        //points3d:输入的空间坐标
        //points2d:待计算的图像坐标系中的坐标
        int calc2DCoordinate(cv::Mat * Intrinsics, vector<Point3f> points3d, vector<Point2f>& points2d);
        int calc2DCoordinate(cv::Mat* Intrinsics, vector<Point2f>& points2d);

//        PtrStepSz<const unsigned short> depth;
///        PtrStepSz<const PixelRGB> rgb24;

        LogReader * logRead;

        FrontProcessor * frontend;

        
        int currentFrame;
        bool firstRun;
        //弧度制
        double euler_arc_X;
        double euler_arc_Y;
        double euler_arc_Z;
};

#endif /* PROCESSINTERFACE_H */
