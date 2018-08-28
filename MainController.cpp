#include "MainController.h"

#include <boost/filesystem.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <boost/algorithm/string.hpp>

#include "frontend/Resolution.h"

MainController * MainController::controller = 0;

MainController::MainController(int argc, char * argv[])
 : Intrinsics(0),
   pangoVis(0)
   /*,
   trackerInterface(0),
   meshGenerator(0),
   placeRecognition(0),
   cloudSliceProcessor(0),
   deformation(0),
   rawRead(0),
   liveRead(0),
   logRead(0)*/
{
//    ConfigArgs::get(argc, argv);

    assert(!MainController::controller);

    MainController::controller = this;
}

MainController::~MainController()
{
    if(Intrinsics)
    {
        delete Intrinsics;
    }
}

int MainController::start()
{
    if(setup())
    {
        return mainLoop();
    }
    else
    {
        return -1;
    }
}

bool MainController::setup()
{
    loadCalibration();

    liveRead = new LiveLogReader();
    //pangoVis = new PangoVis(Intrinsics);
    logRead = static_cast<LogReader *>(liveRead);

    processInterface = new ProcessInterface(logRead, Intrinsics);

    ThreadDataPack::get().assignFrontend(processInterface->getFrontend());

    pangoVis = new PangoVis(Intrinsics);


    return true;
}

int MainController::mainLoop()
{
    timeval start;
    gettimeofday(&start, 0);
    uint64_t beginning = start.tv_sec * 1000000 + start.tv_usec;

    if(pangoVis)
    {
//        pangoVis->start();
    }

    //threads.join_all();

    if(pangoVis)
    {
//        pangoVis->stop();
        delete pangoVis;
    }

    return 0;
}

void MainController::loadCalibration()
{
    Intrinsics = new cv::Mat(cv::Mat::zeros(3, 3, CV_64F));
#if 1
    Intrinsics->at<double>(0, 2) = 337.0261692101221;
    Intrinsics->at<double>(1, 2) = 234.7394852970879;
    Intrinsics->at<double>(0, 0) = 531.3068894976966;
    Intrinsics->at<double>(1, 1) = 531.8271376287391;
    Intrinsics->at<double>(2, 2) = 1;
#else
    Intrinsics->at<double>(0, 2) = 320;
    Intrinsics->at<double>(1, 2) = 267;
    Intrinsics->at<double>(0, 0) = 528.01442863461716;
    Intrinsics->at<double>(1, 1) = 528.01442863461716;
    Intrinsics->at<double>(2, 2) = 1;
#endif

    distCoeff = new cv::Mat(cv::Mat::zeros(1,5,CV_64F));
    distCoeff->at<double>(0, 0) = -0.2756734608366758;
    distCoeff->at<double>(0, 1) = -0.001303202285062331;
    distCoeff->at<double>(0, 2) = 0.001005134230599892;
    distCoeff->at<double>(0, 3) = -0.0008562559253269711;
    distCoeff->at<double>(0, 4) = 0.2240573152028148;

    Resolution::get(640, 480);
    std::cout << "loadCalibration(), Intrinsics:" << std::endl << Intrinsics << std::endl;
    std::cout << "loadCalibration(), distCoeff:" << std::endl << distCoeff << std::endl;
}

void MainController::complete()
{

}

void MainController::save()
{

}

void MainController::reset()
{
    if(pangoVis)
    {
        pangoVis->reset();
    }
}

void MainController::setPark(const bool park)
{
//    trackerInterface->setPark(park);
}

void MainController::shutdown()
{

    if(pangoVis)
    {
//        pangoVis->stop();
    }
}

uint64_t MainController::getMaxLag()
{
    uint64_t maxLag = 0;

//    for(size_t i = 0; i < systemComponents.size(); i++)
    {
//        maxLag = std::max(systemComponents.at(i)->lagTime.getValue(), maxLag);
    }

    return maxLag;
}

void MainController::SnapShot()
{
#if 1
    processInterface->process(Intrinsics);
#else
    printf("SnapShot()\n");
    //grabNext
    int currentFrame = 0;
    bool returnVal = true;
    logRead->grabNext(returnVal, currentFrame);
#endif
}
