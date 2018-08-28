#ifndef HIKGRAB_H
#define HIKGRAB_H

//opencv
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

class HikGrab
{
public:
    HikGrab();
    virtual ~HikGrab();

    int grab();
    cv::Mat getCurrentFrame();

    int getViewerStatus();
    cv::Mat CurrentFrame;

private:
    void printfps(cv::Mat frame);

    int mPreviewFrames;
    int mFPSCount;
    struct timeval mPreviewStartTime;
    struct timeval mPreviewStopTime;


    int viewer_status;
};

#endif // HIKGRAB_H
