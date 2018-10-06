#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

#include <iostream>
#include <fstream>

#include <GL/glew.h>

#include "PangoVis.h"
#include "utils/LiveLogReader.h"
#include "utils/LogReader.h"
#include "utils/ThreadDataPack.h"
#include "backend/ProcessInterface.h"


#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

class MainController
{
    public:
        MainController(int argc, char * argv[]);
        virtual ~MainController();

        int start();

        static MainController * controller;

        //Proxy functions for the GUI
        void SnapShot(double, double, double);
        void complete();

        void save();

        void reset();

        void setPark(const bool park);

        void shutdown();

        uint64_t getMaxLag();

    private:
        bool setup();
        int mainLoop();

        void loadCalibration();

        ProcessInterface * processInterface;
        cv::Mat * Intrinsics;
        cv::Mat * distCoeff;

        PangoVis * pangoVis;
        LiveLogReader * liveRead;
        LogReader * logRead;

        //flq, start All thread
        boost::thread_group threads;
        std::vector<ThreadObject *> systemComponents;
};

#endif //MAINCONTROLLER_H
