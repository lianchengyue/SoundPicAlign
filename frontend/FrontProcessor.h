#ifndef FRONTPROCESSOR_H
#define FRONTPROCESSOR_H

#include "../utils/Stopwatch.h"

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>
#include <limits>
//#include <vector_types.h>
#include "Resolution.h"

#include "../utils/ThreadMutexObject.h"
//相机与声纳初始化位姿信息
#include "utils/utils.h"


#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>

using namespace cv;

class FrontProcessor
{
  public:
    ThreadMutexObject<bool> tsdfRequest;
    bool tsdfAvailable;
    boost::mutex tsdfMutex;

    bool imageAvailable;
    boost::mutex imageMutex;

    bool cycledMutex;
    boost::mutex cloudMutex;
    boost::condition_variable_any cloudSignal;

    FrontProcessor(cv::Mat * depthIntrinsics);

    virtual ~FrontProcessor();

    void processFrame(unsigned char * rgbImage, vector<Point2f> points2d);
    void processVideoFrame(unsigned char *rgbImage);

    void finalise();

    void reset();

    unsigned char* getRGBImage();

    cv::Mat eulerAnglesToRotationMatrix(Vec3f &theta);
    cv::Mat setTMatrix();

    ThreadMutexObject<uint64_t> init_utime;
    ThreadMutexObject<unsigned char *> firstRgbImage;
    unsigned char * lastRgbImage;
    unsigned char * lastVideoImage;


  private:
    /** \brief Frame counter */
    int global_time_;

    //To keep track of how far behind the backend is
    uint64_t lagTime;

    bool cycled;
    int overlap;
    bool parked;

    //Intr intr;

    uint64_t current_utime;
    uint64_t last_utime;

    Eigen::Vector3f currentGlobalCamera;

    Eigen::Vector3f lastPlaceRecognitionTrans;
    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> lastPlaceRecognitionRot;

    void outputPose(uint64_t & timestamp, Eigen::Matrix<float, 3, 3, Eigen::RowMajor> & Rcurr);

    void getImage();

    void mutexOutLiveImage();


};

#endif /* FRONTPROCESSOR_H */
