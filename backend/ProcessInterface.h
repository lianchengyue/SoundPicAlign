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
        bool inline process(cv::Mat * Intrinsics);

    private:
//        bool inline process(cv::Mat * Intrinsics);
        bool calcRMatrix();
        bool calcTMatrix();
        bool calcCameraPose(/*Eigen::Matrix4f& pose*/);
        int calc2DCoordinate(cv::Mat * Intrinsics, vector<Point2f>& points2d);

//        PtrStepSz<const unsigned short> depth;
///        PtrStepSz<const PixelRGB> rgb24;

        LogReader * logRead;

        FrontProcessor * frontend;

        
        int currentFrame;
        bool firstRun;
};

#endif /* PROCESSINTERFACE_H */
