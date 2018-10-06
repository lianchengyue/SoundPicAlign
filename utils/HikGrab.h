#ifndef HIKGRAB_H
#define HIKGRAB_H

//opencv
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <QDir>
#include <fstream>
#include <unistd.h>
//#include "auto_entercs.h"

#include "HCNetSDK.h"
#include "PlayM4.h"
#include "LinuxPlayM4.h"

#define HPR_ERROR       -1
#define HPR_OK               0
#define USECOLOR          0

class HikGrab
{
public:
    HikGrab();
    virtual ~HikGrab();

    int grab();
    cv::Mat* getCurrentFrame();
    int getFrameIdx();

    int getViewerStatus();
    cv::Mat CurrentFrame;

private:
    void printfps(cv::Mat frame);

    int mFPSCount;
    int mFrameIdx;
    struct timeval mPreviewStartTime;
    struct timeval mPreviewStopTime;

    int viewer_status;
    //Hik
#if 0
    static cv::Mat dst;
//    HWND h = NULL;
//    LONG nPort=-1;
    static HWND h;
    static LONG nPort;
    static LONG lUserID;

    static pthread_mutex_t mutex;
    static std::list<cv::Mat> g_frameList;

//    FILE *g_pFile = NULL;
    static FILE *g_pFile;
#endif
    //cv::Mat CurrentFrame;

    static void PsDataCallBack(LONG lRealHandle, DWORD dwDataType,BYTE *pPacketBuffer,DWORD nPacketSize, void* pUser);
    static void CALLBACK DecCBFun(LONG nPort, char *pBuf, LONG nSize, FRAME_INFO *pFrameInfo, void* nReserved1, LONG nReserved2);
    static void /*CALLBACK */g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize,void* dwUser);
    static void /*CALLBACK */g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser);
    static void* RunIPCameraInfo(void *);
    //static void xxx(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser);
};

#endif // HIKGRAB_H
