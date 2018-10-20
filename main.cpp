#if 0
#include "opencv2/opencv.hpp"

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

static cv::Mat dst;
HWND h = NULL;
LONG nPort=-1;
LONG lUserID;

pthread_mutex_t mutex;
std::list<cv::Mat> g_frameList;


FILE *g_pFile = NULL;

void CALLBACK PsDataCallBack(LONG lRealHandle, DWORD dwDataType,BYTE *pPacketBuffer,DWORD nPacketSize, void* pUser)
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
void CALLBACK DecCBFun(LONG nPort, char *pBuf, LONG nSize, FRAME_INFO *pFrameInfo, void* nReserved1, LONG nReserved2)
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

void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize,void* dwUser)
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
           std::cerr << "PlayM4_InputData failed \n" << std::endl;
         }
       }
       break;
   }
}

void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser)
{
   char tempbuf[256] = {0};
   std::cout << "EXCEPTION_RECONNECT = " << EXCEPTION_RECONNECT << std::endl;
   switch(dwType)
   {
   case EXCEPTION_RECONNECT:	//预览时重连
       printf("pyd----------reconnect--------%d\n", time(NULL));
       break;
   default:
       break;
   }
}

void *RunIPCameraInfo(void *)
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

   NET_DVR_SetExceptionCallBack_V30(0, NULL, g_ExceptionCallBack, NULL);

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

int main(int argc, char *argv[])
{
#if 0 //logout
   NET_DVR_Logout_V30(0);
#else
   pthread_t getframe;

   pthread_mutex_init(&mutex, NULL);
   int ret;

   ret = pthread_create(&getframe, NULL, RunIPCameraInfo, NULL);


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
#endif
   return 0;
}

#else
#include "PangoVis.h"
#include "MainController.h"

int main(int argc, char *argv[])
{
//    PangoVis *pangoVis = new PangoVis();

    MainController controller(argc, argv);
    return controller.start();

}
#endif
















#if 0
#include "opencv2/opencv.hpp"

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

static cv::Mat dst;
HWND h = NULL;
LONG nPort=-1;
LONG lUserID;

pthread_mutex_t mutex;
std::list<cv::Mat> g_frameList;


FILE *g_pFile = NULL;

void CALLBACK PsDataCallBack(LONG lRealHandle, DWORD dwDataType,BYTE *pPacketBuffer,DWORD nPacketSize, void* pUser)
{

   if (dwDataType  == NET_DVR_SYSHEAD)
   {
       //写入头数据
       g_pFile = fopen("/home/lds/source/ps.dat", "wb");

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
void CALLBACK DecCBFun(LONG nPort, char *pBuf, LONG nSize, FRAME_INFO *pFrameInfo, void* nReserved1, LONG nReserved2)
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

void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize,void* dwUser)
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
           std::cerr << "PlayM4_InputData failed \n" << std::endl;
         }
       }
       break;
   }
}

void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser)
{
   char tempbuf[256] = {0};
   std::cout << "EXCEPTION_RECONNECT = " << EXCEPTION_RECONNECT << std::endl;
   switch(dwType)
   {
   case EXCEPTION_RECONNECT:	//预览时重连
       printf("pyd----------reconnect--------%d\n", time(NULL));
       break;
   default:
       break;
   }
}

void *RunIPCameraInfo(void *)
{
   char IP[]         = "192.168.**.***";   //海康威视网络摄像头的ip
   char UName[] = "****";                 //海康威视网络摄像头的用户名
   char PSW[]      = "*****";           //海康威视网络摄像头的密码
   NET_DVR_Init();
   NET_DVR_SetConnectTime(2000, 1);
   NET_DVR_SetReconnect(1000, true);
   NET_DVR_SetLogToFile(3, "./sdkLog");
   NET_DVR_DEVICEINFO_V30 struDeviceInfo = {0};
   NET_DVR_SetRecvTimeOut(5000);
   lUserID = NET_DVR_Login_V30(IP, 8000, UName, PSW, &struDeviceInfo);

   NET_DVR_SetExceptionCallBack_V30(0, NULL, g_ExceptionCallBack, NULL);

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

int main(int argc, char *argv[])
{
   pthread_t getframe;

   pthread_mutex_init(&mutex, NULL);
   int ret;

   ret = pthread_create(&getframe, NULL, RunIPCameraInfo, NULL);


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

   return 0;
}

#endif
