#include <sys/time.h>
#include "usbGrab.h"

#include "frontend/Resolution.h"

#define OPENCV_WIN
#define FPS_LOG_FREQ 3

using namespace cv;
using namespace std;

usbGrab::usbGrab()
{
    mFPSCount = 0;
    mFrameIdx = 0;
 ///flq   decompressionBuffer = new unsigned char [Resolution::get().numPixels() * 2];
}

usbGrab::~usbGrab()
{

}


int usbGrab::grab()
{
//    int width = INPUT_WIDTH;
//    int height = INPUT_HEIGHT;
    int width = Resolution::get().width();
    int height = Resolution::get().height();

    if(1 == viewer_status) //0:off 1:on
    {
        namedWindow("usb camera",WINDOW_AUTOSIZE);
    }

    VideoCapture capture(0);
    //设置图片的大小
    capture.set(CV_CAP_PROP_FRAME_WIDTH, width);//1280 INPUT_WIDTH
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, height);//960 INPUT_HEIGHT
    while (1)
    {
        Mat frame;
        capture >> frame;

        //释放zbar相关
        //if(100 == mFrameIdx)
        //{
        //   delete m_scancode;
        //}

        //帧数及帧率
        if(0 == mFrameIdx)
        {
            printf("first usb frame arrives!\n");
        }

        ////printf("mFrameIdx=%d,", mFrameIdx);

        if(0 == mFPSCount)
        {
            gettimeofday(&mPreviewStartTime, NULL);
        }

        printfps(frame);

        char c = cv::waitKey(10); //100 CV_WAITKEY_INTERVAL
        if (c == 't')
        {
            break;
        }

        //cvtColor(frame,mScanImgData.imageGray,CV_RGB2GRAY);
        imshow("usb camera",frame);

        mFrameIdx++;
    }
}

cv::Mat* usbGrab::getCurrentFrame()
{
    int width = Resolution::get().width();
    int height = Resolution::get().height();

    if(1 == viewer_status) //0:off 1:on
    {
        namedWindow("usb camera",WINDOW_AUTOSIZE);
    }

    VideoCapture capture(0);
    //设置图片的大小
    capture.set(CV_CAP_PROP_FRAME_WIDTH, width);//1280 INPUT_WIDTH
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, height);//960 INPUT_HEIGHT


    capture >> CurrentFrame;
    //使反色正常
    cvtColor(CurrentFrame, CurrentFrame, CV_RGB2BGR);
    imwrite("tt.jpg", CurrentFrame);

    //test
/*
    cv::Mat Image(Resolution::get().height(),Resolution::get().width(),CV_8UC3,cv::Scalar(0));
    unsigned char *rgbImage;
    rgbImage =  CurrentFrame.data;
    Image.data = rgbImage;
    imwrite("outout.jpg", Image);
*/
    //test end

    return &CurrentFrame;
}

//print fps
void usbGrab::printfps(cv::Mat frame)
{
    //不显示帧率
    int elapse;
    int fps;

    mFPSCount++;
    gettimeofday(&mPreviewStopTime, NULL);
    elapse = ((mPreviewStopTime.tv_sec - mPreviewStartTime.tv_sec)*1000) + ((mPreviewStopTime.tv_usec - mPreviewStartTime.tv_usec)/1000);
    if(elapse/(1000*FPS_LOG_FREQ) > 0)
    {
        fps = mFPSCount / FPS_LOG_FREQ;
        printf("preview frames: %d, fps: %d\n", mFPSCount, fps);
        mFPSCount = 0;
    }
}

int usbGrab::getViewerStatus()
{
    char *sect;
    char *key;
    int intval;


    //小于3fps,设为3fps
    if(intval < 0)
    {
        return 0;
    }
    //大于30fps,设为30fps
    else if (intval > 3)
    {
        return 3;
    }
    else
    {
        printf("onoff=%d\n", intval);
        return intval;
    }
}

int usbGrab::getFrameIdx()
{
    return mFrameIdx;
}
