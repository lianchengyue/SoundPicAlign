#include "MainController.h"

#include <boost/filesystem.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <boost/algorithm/string.hpp>

#include "frontend/Resolution.h"

MainController * MainController::controller = 0;

MainController::MainController(int argc, char * argv[])
 : Intrinsics(0),
   pangoVis(0),
   processInterface(0),
   liveRead(0),
   logRead(0)
{
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
    logRead = static_cast<LogReader *>(liveRead);

    ThreadDataPack::get();

    processInterface = new ProcessInterface(logRead, Intrinsics);

    //添加ProcessInterface线程
    systemComponents.push_back(processInterface);
    ThreadDataPack::get().assignFrontend(processInterface->getFrontend());

    pangoVis = new PangoVis(Intrinsics);


    return true;
}

int MainController::mainLoop()
{
    timeval start;
    gettimeofday(&start, 0);
    uint64_t beginning = start.tv_sec * 1000000 + start.tv_usec;

    //flq,import, start All Threads
    for(unsigned int i = 0; i < systemComponents.size(); i++)
    {
        threads.add_thread(new boost::thread(boost::bind(&ThreadObject::start, systemComponents.at(i))));
    }

    if(pangoVis)
    {
        pangoVis->start();
    }

    threads.join_all();

    for(unsigned int i = 0; i < systemComponents.size(); i++)
    {
        delete systemComponents.at(i);
    }

    if(pangoVis)
    {
        pangoVis->stop();
        delete pangoVis;
    }

    return 0;
}

void MainController::loadCalibration()
{
    Intrinsics = new cv::Mat(cv::Mat::zeros(3, 3, CV_64F));

#ifdef HIKVISION_GRAB_FUNC
///*
    //4mm, 1920x1080
    Intrinsics->at<double>(0, 2) = 944.740327771722;
    Intrinsics->at<double>(1, 2) = 587.2490988141938;
    Intrinsics->at<double>(0, 0) = 1325.47033073642;
    Intrinsics->at<double>(1, 1) = 1335.625929960983;
    Intrinsics->at<double>(2, 2) = 1;

    distCoeff = new cv::Mat(cv::Mat::zeros(1,5,CV_64F));
    distCoeff->at<double>(0, 0) = -0.3842780507597445;
    distCoeff->at<double>(0, 1) = -0.01026628950727477;
    distCoeff->at<double>(0, 2) = -0.001097510941657248;
    distCoeff->at<double>(0, 3) = 0.0002966871455004014;
    distCoeff->at<double>(0, 4) = 0.7321469514653421;
//*/
/*
    //12mm, 4096x2160
    Intrinsics->at<double>(0, 2) = 2058.807559042791;
    Intrinsics->at<double>(1, 2) = 1107.865716534388;
    Intrinsics->at<double>(0, 0) = 3451.112107600577;
    Intrinsics->at<double>(1, 1) = 3470.458679134347;
    Intrinsics->at<double>(2, 2) = 1;

    distCoeff = new cv::Mat(cv::Mat::zeros(1,5,CV_64F));
    distCoeff->at<double>(0, 0) = -0.1829310138260891;
    distCoeff->at<double>(0, 1) = 0.921006884774823;
    distCoeff->at<double>(0, 2) = -0.006458022608597794;
    distCoeff->at<double>(0, 3) = 0.007211980182173494;
    distCoeff->at<double>(0, 4) = -3.849852944304393;
*/
/*
    //25mm, 4096x2160
    Intrinsics->at<double>(0, 2) = 2047.826161062829;
    Intrinsics->at<double>(1, 2) = 1083.470877609614;
    Intrinsics->at<double>(0, 0) = 10052.25141379218;
    Intrinsics->at<double>(1, 1) = 10251.85599075164;
    Intrinsics->at<double>(2, 2) = 1;

    distCoeff = new cv::Mat(cv::Mat::zeros(1,5,CV_64F));
    distCoeff->at<double>(0, 0) = -0.892716031334611;
    distCoeff->at<double>(0, 1) = 12.98805686689792;
    distCoeff->at<double>(0, 2) = 0.01352535548838887;
    distCoeff->at<double>(0, 3) = -0.02306571153764621;
    distCoeff->at<double>(0, 4) = -92.62687091981161;
*/
#else
    Intrinsics->at<double>(0, 2) = 337.0261692101221;
    Intrinsics->at<double>(1, 2) = 234.7394852970879;
    Intrinsics->at<double>(0, 0) = 531.3068894976966;
    Intrinsics->at<double>(1, 1) = 531.8271376287391;
    Intrinsics->at<double>(2, 2) = 1;

    distCoeff = new cv::Mat(cv::Mat::zeros(1,5,CV_64F));
    distCoeff->at<double>(0, 0) = -0.2756734608366758;
    distCoeff->at<double>(0, 1) = -0.001303202285062331;
    distCoeff->at<double>(0, 2) = 0.001005134230599892;
    distCoeff->at<double>(0, 3) = -0.0008562559253269711;
    distCoeff->at<double>(0, 4) = 0.2240573152028148;
#endif

    Resolution::get(RESOLUTION_WIDTH, RESOLUTION_HEIGHT);
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
        pangoVis->stop();
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

void MainController::SnapShot(double px, double py, double pz)
{
    //像素坐标系中的一个点
    //vector<vector<Point3f>>  points3d(1);   //points3d[0]
    vector<Point3f>  points3d(1);


    points3d[0].x = px;
    points3d[0].y = py;
    points3d[0].z = pz;
//test data
    //points3d[0] = Point3f((double)1.0f, (double)2.2f, (double)24.6f); //set x
//test data end

    //触发定位时的处理函数
    processInterface->process(Intrinsics, points3d);  //进行logRead->grabNext(returnVal, currentFrame)
}
