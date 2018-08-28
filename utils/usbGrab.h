#ifndef USBGRAB_H
#define USBGRAB_H

//opencv
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

class usbGrab
{
public:
    usbGrab();
    virtual ~usbGrab();

    int grab();
    cv::Mat* getCurrentFrame();
    int getFrameIdx();

    int getViewerStatus();
    cv::Mat CurrentFrame;

    static const int numBuffers = 10;
    unsigned char* frameBuffers[numBuffers];

private:
    void printfps(cv::Mat frame);

    //int mPreviewFrames;
    int mFPSCount;
    int mFrameIdx;
    struct timeval mPreviewStartTime;
    struct timeval mPreviewStopTime;


    int viewer_status;
};

#endif // USBGRAB_H
