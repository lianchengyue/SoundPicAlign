#include <sys/time.h>
#include "HikGrab.h"

#include "frontend/Resolution.h"

#define OPENCV_WIN
#define FPS_LOG_FREQ 3

using namespace cv;
//using namespace std;

    cv::Mat dst;
    HWND h;
    LONG nPort;
    LONG lUserID;

    pthread_mutex_t mutex;
    std::list<cv::Mat> g_frameList;

    FILE *g_pFile;

void HikGrab::PsDataCallBack(LONG lRealHandle, DWORD dwDataType,BYTE *pPacketBuffer,DWORD nPacketSize, void* pUser)
{

   if (dwDataType  == NET_DVR_SYSHEAD)
   {
       //写入头数据
       g_pFile = fopen("/home/montafan/ps.dat", "wb");

       if (g_pFile == NULL)
       {
           printf("CreateFileHead fail\n");
           return;
       }

       //写入头数据
       fwrite(pPacketBuffer, sizeof(unsigned char), nPacketSize, g_pFile);
       printf("write head len=%d\n", nPacketSize);
   }
   else
   {
       if(g_pFile != NULL)
       {
           fwrite(pPacketBuffer, sizeof(unsigned char), nPacketSize, g_pFile);
           printf("write data len=%d\n", nPacketSize);
       }
   }

}

//void CALLBACK DecCBFun(LONG nPort, char *pBuf, LONG nSize, FRAME_INFO *pFrameInfo, LONG nReserved1, LONG nReserved2)
void HikGrab::CALLBACK DecCBFun(LONG nPort, char *pBuf, LONG nSize, FRAME_INFO *pFrameInfo, void* nReserved1, LONG nReserved2)
{
   long lFrameType = pFrameInfo->nType;

     if (lFrameType == T_YV12)
     {
      //cv::Mat dst(pFrameInfo->nHeight, pFrameInfo->nWidth,
      //            CV_8UC3);  // 8UC3表示8bit uchar无符号类型,3通道值
           dst.create(pFrameInfo->nHeight, pFrameInfo->nWidth,
                 CV_8UC3);

           cv::Mat src(pFrameInfo->nHeight + pFrameInfo->nHeight / 2, pFrameInfo->nWidth, CV_8UC1, (uchar *)pBuf);
           cv::cvtColor(src, dst, CV_YUV2BGR_YV12);
           pthread_mutex_lock(&mutex);
           g_frameList.push_back(dst);
           pthread_mutex_unlock(&mutex);
     }
    usleep(1000);

   //cv::Mat src(pFrameInfo->nHeight + pFrameInfo->nHeight / 2, pFrameInfo->nWidth, CV_8UC1, (uchar *)pBuf);
   //cv::cvtColor(src, dst, CV_YUV2BGR_YV12);
   //cv::imshow("bgr", dst);
   //pthread_mutex_lock(&mutex);
   //g_frameList.push_back(dst);
   //pthread_mutex_unlock(&mutex);
   //vw << dst;
   //cv::waitKey(10);
}

void HikGrab::/*CALLBACK */g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize,void* dwUser)
{
   /*
   if (dwDataType == 1)
   {
       PlayM4_GetPort(&nPort);
       PlayM4_SetStreamOpenMode(nPort, STREAME_REALTIME);
       PlayM4_OpenStream(nPort, pBuffer, dwBufSize, 1024 * 1024);
       PlayM4_SetDecCallBackEx(nPort, DecCBFun, NULL, NULL);
       PlayM4_Play(nPort, h);
   }
   else
   {
       BOOL inData = PlayM4_InputData(nPort, pBuffer, dwBufSize);
   }*/
   DWORD dRet;
   switch (dwDataType)
   {
     case NET_DVR_SYSHEAD:           //系统头
       if (!PlayM4_GetPort(&nPort))  //获取播放库未使用的通道号
       {
         break;
       }
       if (dwBufSize > 0) {
         if (!PlayM4_SetStreamOpenMode(nPort, STREAME_REALTIME)) {
           dRet = PlayM4_GetLastError(nPort);
           break;
         }
         if (!PlayM4_OpenStream(nPort, pBuffer, dwBufSize, 1024 * 1024)) {
           dRet = PlayM4_GetLastError(nPort);
           break;
         }
         //设置解码回调函数 只解码不显示
        //  if (!PlayM4_SetDecCallBack(nPort, DecCBFun)) {
        //     dRet = PlayM4_GetLastError(nPort);
        //     break;
        //  }

         //设置解码回调函数 解码且显示
         if (!PlayM4_SetDecCallBackEx(nPort, DecCBFun, NULL, NULL))
         {
           dRet = PlayM4_GetLastError(nPort);
           break;
         }

         //打开视频解码
         if (!PlayM4_Play(nPort, h))
         {
           dRet = PlayM4_GetLastError(nPort);
           break;
         }

         //打开音频解码, 需要码流是复合流
         if (!PlayM4_PlaySound(nPort)) {
           dRet = PlayM4_GetLastError(nPort);
           break;
         }
       }
       break;
       //usleep(500);
     case NET_DVR_STREAMDATA:  //码流数据
       if (dwBufSize > 0 && nPort != -1) {
         BOOL inData = PlayM4_InputData(nPort, pBuffer, dwBufSize);
         while (!inData) {
           sleep(100);
           inData = PlayM4_InputData(nPort, pBuffer, dwBufSize);
///           std::cerr << "PlayM4_InputData failed \n" << std::endl;
           printf("PlayM4_InputData failed!\n");
         }
       }
       break;
   }
}

void HikGrab::/*CALLBACK */g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser)
{
   char tempbuf[256] = {0};
///   std::cout << "EXCEPTION_RECONNECT = " << EXCEPTION_RECONNECT << std::endl;
   printf("EXCEPTION_RECONNECT\n");
   switch(dwType)
   {
   case EXCEPTION_RECONNECT:	//预览时重连
       printf("pyd----------reconnect--------%d\n", time(NULL));
       break;
   default:
       break;
   }
}

void* HikGrab::RunIPCameraInfo(void *)
{
   char IP[]         = "192.168.1.64";   //海康威视网络摄像头的ip
   char UName[] = "admin";                 //海康威视网络摄像头的用户名
   char PSW[]      = "a1234567";           //海康威视网络摄像头的密码
   NET_DVR_Init();
   NET_DVR_SetConnectTime(2000, 1);
   NET_DVR_SetReconnect(1000, true);
   NET_DVR_SetLogToFile(3, "./sdkLog");
   NET_DVR_DEVICEINFO_V30 struDeviceInfo = {0};
   NET_DVR_SetRecvTimeOut(5000);
   lUserID = NET_DVR_Login_V30(IP, 8000, UName, PSW, &struDeviceInfo); //port 16
   //lUserID = NET_DVR_Login_V30("192.168.1.64", 8000, "admin", "a1234567", &struDeviceInfo); //port 1
   printf("lUserID=%d\n", lUserID);

   NET_DVR_SetExceptionCallBack_V30(0, NULL, HikGrab::g_ExceptionCallBack, NULL);
   ///NET_DVR_SetExceptionCallBack_V30(0, NULL, xxx, NULL);
   long lRealPlayHandle;
   NET_DVR_CLIENTINFO ClientInfo = {0};

   ClientInfo.lChannel       = 1;
   ClientInfo.lLinkMode     = 0;
   ClientInfo.hPlayWnd     = 0;
   ClientInfo.sMultiCastIP = NULL;


   //lRealPlayHandle = NET_DVR_RealPlay_V30(lUserID, &ClientInfo, PsDataCallBack, NULL, 0);
   lRealPlayHandle = NET_DVR_RealPlay_V30(lUserID, &ClientInfo, g_RealDataCallBack_V30, NULL, 0);
   //NET_DVR_SaveRealData(lRealPlayHandle, "/home/lds/source/yuntai.mp4");
   if (lRealPlayHandle < 0)
   {
       printf("pyd1---NET_DVR_RealPlay_V30 error\n");
   }
   sleep(-1);

   NET_DVR_Cleanup();
}


HikGrab::HikGrab()
{
    mFPSCount = 0;
    mFrameIdx = 0;
 ///flq   decompressionBuffer = new unsigned char [Resolution::get().numPixels() * 2];
    h = NULL;
    nPort = -1;
    g_pFile = NULL;

    //stat hik init
    pthread_t getframe;

    pthread_mutex_init(&mutex, NULL);
    int ret;

    ret = pthread_create(&getframe, NULL, HikGrab::RunIPCameraInfo, NULL);


    if(ret!=0)
    {
        printf("Create pthread error!\n");
    }
    //end hik init
}

HikGrab::~HikGrab()
{

}


int HikGrab::grab()
{
#if 0
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
#endif
    //HIKON
    pthread_t getframe;

    pthread_mutex_init(&mutex, NULL);
    int ret;

    ret = pthread_create(&getframe, NULL, HikGrab::RunIPCameraInfo, NULL);


    if(ret!=0)
    {
        printf("Create pthread error!\n");
    }

    cv::Mat image;
    while(1)
    {
        pthread_mutex_lock(&mutex);
        if(g_frameList.size())
        {
            std::list<cv::Mat>::iterator it;
            it = g_frameList.end();
            it--;
            image = (*(it));
            if (!image.empty())
            {
                imshow("frame from camera",image);
                cv::waitKey(1);
            }
            g_frameList.pop_front();
        }
        g_frameList.clear(); // 丢掉旧的帧
        pthread_mutex_unlock(&mutex);
    }
}

cv::Mat* HikGrab::getCurrentFrame()
{
    int width = Resolution::get().width();
    int height = Resolution::get().height();

    if(1 == viewer_status) //0:off 1:on
    {
        namedWindow("hik camera",WINDOW_AUTOSIZE);
    }


    cv::Mat CurrentFrame;
    while(1)
//    for(int i = 0; i<1000; i++)
    {
        pthread_mutex_lock(&mutex);
        if(g_frameList.size())
        {
            std::list<cv::Mat>::iterator it;
            it = g_frameList.end();
            it--;
            CurrentFrame = (*(it));
            if (!CurrentFrame.empty())
            {
                imshow("hik camera",CurrentFrame);
                cv::waitKey(1);
            }
            g_frameList.pop_front();
        }
        g_frameList.clear(); // 丢掉旧的帧
        pthread_mutex_unlock(&mutex);
    }


    return &CurrentFrame;

#if 0
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
#endif
}

//print fps
void HikGrab::printfps(cv::Mat frame)
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

int HikGrab::getViewerStatus()
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

int HikGrab::getFrameIdx()
{
    return mFrameIdx;
}
