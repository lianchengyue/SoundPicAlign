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
   char IP[]         = "169.254.80.177";   //海康威视网络摄像头的ip
   char UName[] = "admin";                 //海康威视网络摄像头的用户名
   char PSW[]      = "12345";           //海康威视网络摄像头的密码
   NET_DVR_Init();
   NET_DVR_SetConnectTime(2000, 1);
   NET_DVR_SetReconnect(1000, true);
   NET_DVR_SetLogToFile(3, "./sdkLog");
   NET_DVR_DEVICEINFO_V30 struDeviceInfo = {0};
   NET_DVR_SetRecvTimeOut(5000);
   //lUserID = NET_DVR_Login_V30(IP, 16, UName, PSW, &struDeviceInfo); //port 16
   lUserID = NET_DVR_Login_V30("169.254.80.177", 8000, "admin", "12345", &struDeviceInfo); //port 16
   //("172.2.87.106", 8000, "admin", "12345", &struDeviceInfo);

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


#if 1
#include "PangoVis.h"
#include "MainController.h"

#if 1
int main(int argc, char *argv[])
{
//    PangoVis *pangoVis = new PangoVis();

    MainController controller(argc, argv);
    return controller.start();

}
#elif 0
#include "PangoVis.h"
#include <GL/glut.h>

/*    void GLGrid(coordpoint& pt1, coordpoint& pt2,int num)
    {

        const float _xLen = (pt2.x -pt1.x) / num;
        const float _yLen = (pt2.y - pt1.y) / num;
        const float _zLen = (pt2.z - pt1.z) / num;


        glLineWidth(1.0f);
        glLineStipple(1, 0x0303);//线条样式

        glBegin(GL_LINES);
        glEnable(GL_LINE_SMOOTH);

        //glColor3f(0.0f,0.0f, 1.0f); //白色线条

        int xi = 0;
        int yi = 0;
        int zi = 0;

        //绘制平行于X的直线

        for (zi = 0; zi <= num; zi++)
        {
            float z = _zLen * zi + pt1.z;

            for (yi = 0; yi <= num; yi++)
            {
                float y = _yLen * yi +pt1.y;

                glVertex3f(pt1.x, y, z);
                glVertex3f(pt2.x, y, z);
            }
        }

        //绘制平行于Y的直线
        for (zi = 0; zi <= num; zi++)
        {

            float z = _zLen * zi + pt1.z;

            for (xi = 0; xi <= num; xi++)
            {
                float x = _xLen * xi +pt1.x;

                glVertex3f(x, pt1.y, z);
                glVertex3f(x, pt2.y, z);
            }

        }

        //绘制平行于Z的直线

        for (yi = 0; yi <= num; yi++)
        {

            float y = _yLen * yi + pt1.y;

            for (xi = 0; xi <= num; xi++)
            {
                float x = _xLen * xi +pt1.x;

                glVertex3f(x, y, pt1.z);

                glVertex3f(x, y, pt2.z);
            }
        }

        glEnd();
    }
*/
    void GLDrawSpaceAxes(void)
    {

        GLUquadricObj *objCylinder =gluNewQuadric();
        glPushMatrix();

        glColor3f(1.0f,1.0f, 1.0f);
        glutSolidSphere(0.25,6,6);

        glColor3f(0.0f,0.0f, 1.0f);

        gluCylinder(objCylinder,0.1, 0.1, AXES_LEN, 10, 5);         //Z

        glTranslatef(0,0,AXES_LEN);

        gluCylinder(objCylinder,0.3, 0.0, 0.6, 10, 5);                 //Z

        glPopMatrix();
        glPushMatrix();

        glTranslatef(0,0.5,AXES_LEN);
        glRotatef(90,0.0,1.0,0.0);

        GLPrint("Z");                                               // Print GL Text ToThe Screen
        glPopMatrix();





        glPushMatrix();

        glColor3f(0.0f,1.0f, 0.0f);

        glRotatef(-90,1.0,0.0,0.0);

        gluCylinder(objCylinder,0.1, 0.1, AXES_LEN, 10, 5);         //Y

        glTranslatef(0,0,AXES_LEN);

        gluCylinder(objCylinder,0.3, 0.0, 0.6, 10, 5);                 //Y

        glPopMatrix();



        glPushMatrix();

        glTranslatef(0.5,AXES_LEN,0);

        GLPrint("Y");                                               // Print GL Text ToThe Screen

        glPopMatrix();
        glPushMatrix();

        glColor3f(1.0f,0.0f, 0.0f);
        glRotatef(90,0.0,1.0,0.0);

        gluCylinder(objCylinder,0.1, 0.1, AXES_LEN, 10, 5);         //X
        glTranslatef(0,0,AXES_LEN);

        gluCylinder(objCylinder,0.3, 0.0, 0.6, 10, 5);                 //X

        glPopMatrix();

        glPushMatrix();
        glTranslatef(AXES_LEN,0.5,0);

        GLPrint("X");                                               // Print GL Text ToThe Screen

        glPopMatrix();
    }



    #define AXES_LEN 10 //坐标轴长

    void GLDrawCubeCoordinates(void)

    {

    /*****网格绘制*****/

    /*****使用颜色混合来消除一些锯齿， 主要针对点和线

    以及不相互重叠的多边形的反锯齿。*****/

    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_BLEND);

    glEnable(GL_POINT_SMOOTH);                   //设置反走样

    glHint(GL_POINT_SMOOTH_HINT,GL_NICEST);       //设置反走样

    glEnable(GL_LINE_SMOOTH);

    glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);

    glEnable(GL_POLYGON_SMOOTH);

    glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST);

    //绘制立体坐标系

    GLUquadricObj *objCylinder =gluNewQuadric();

    glRotatef(-45,0.0,1.0,0.0);

    //确定坐标系原点

    glPushMatrix();

    glColor3f(1.0f,1.0f, 1.0f);

    glTranslatef(-5,0,-5);

    glutSolidSphere(0.2,20,20);

    //glutSolidTorus(0.2,1,10,10);圆环

    glPopMatrix();

    //画网格线

    coordpoint cpoint1 = {-5,0,-5};

    coordpoint cpoint2 = {5,0,5};

    glColor3f(0.9f,0.9f,0.9f);

    GLGrid(cpoint1,cpoint2,10);

    //画网格线





    glPushMatrix();

    {

        glRotatef(90,1.0,0.0,0.0);

        glTranslatef(0.0f,-5,-5);

        coordpoint cpoint3 ={-5,00,-5};

        coordpoint cpoint4 = {5,00,5};

        glColor3f(0.9f,0.9f,0.0f);

        GLGrid(cpoint3,cpoint4,10);



        //半透明绘制

        //glDepthMask(GL_FALSE);

        //glColor3f(0.3,0.3,0.0);

        //glTranslatef(0.0,5,0.0);

        //glutSolidSphere(2,20,20);

        //glDepthMask(GL_TRUE);

    }

    glPopMatrix();

    //画网格线



    glPushMatrix();

    glRotatef(90,0.0,0.0,1.0);

    glTranslatef(5,5,-0);

    coordpoint cpoint5 ={-5,0,-5};

    coordpoint cpoint6 = {5,0,5};

    glColor3f(0.0f,0.9f,0.0f);

    GLGrid(cpoint5,cpoint6,10);

    glPopMatrix();



    //画坐标轴

    glPushMatrix();

    glColor3f(0.0f, 1,0.0f);

    glTranslatef(-5,0,-5);

    gluCylinder(objCylinder,0.05, 0.05, AXES_LEN, 10, 5);           //X

    glTranslatef(0,0,AXES_LEN);

    gluCylinder(objCylinder,0.2, 0.0, 0.5, 10, 5);                 //X

    glPopMatrix();



    glPushMatrix();

    glTranslatef(-5,0,-5);

    glTranslatef(0,0.2,AXES_LEN);

    glRotatef(90,0.0,1.0,0.0);

    GLPrint("X");                                               // Print GL Text ToThe Screen

    glPopMatrix();



    //画坐标轴

    glPushMatrix();

    glColor3f(1, 0,0.0f);

    glTranslatef(-5,0,-5);

    glRotatef(90,0.0,1.0,0.0);

    gluCylinder(objCylinder,0.05, 0.05, AXES_LEN, 10, 5);           //Y

    glTranslatef(0,0,AXES_LEN);

    gluCylinder(objCylinder,0.2, 0.0, 0.5, 10, 5);                 //Y

    glPopMatrix();



    glPushMatrix();

    glTranslatef(-5,0,-5);

    glRotatef(90,0.0,1.0,0.0);

    glTranslatef(0,0.2,AXES_LEN);

    glRotatef(90,0.0,1.0,0.0);

    GLPrint("Y");                                               // Print GL Text ToThe Screen

    glPopMatrix();



    //画坐标轴

    glPushMatrix();

    glColor3f(1, 1,0.0f);

    glTranslatef(-5,0,-5);

    glRotatef(-90,1.0,0.0,0.0);

    gluCylinder(objCylinder,0.05, 0.05, AXES_LEN, 10, 5);           //Z

    glTranslatef(0,0,AXES_LEN);

    gluCylinder(objCylinder,0.2, 0.0, 0.5, 10, 5);                 //Z

    glPopMatrix();



    glPushMatrix();

    glTranslatef(-5,0,-5);

    glRotatef(-90,1.0,0.0,0.0);

    glTranslatef(0.0,0.6,AXES_LEN);

    glRotatef(90,0.0,1.0,0.0);

    glRotatef(90,0.0,0.0,1.0);

    GLPrint("Z");                                               // Print GL Text ToThe Screen

    glPopMatrix();





    /*****取消反锯齿*****/

    glDisable(GL_BLEND);

    glDisable(GL_LINE_SMOOTH);

    glDisable(GL_POINT_SMOOTH);

    glDisable(GL_POLYGON_SMOOTH);

     }
#elif 0
#include <iostream>
#include <pangolin/pangolin.h>
#include "PangoVis.h"
#include <GL/glut.h>

int main(int argc, char **argv)
{
    //创建一个窗口
    pangolin::CreateWindowAndBind("Main",640,480);
    //启动深度测试
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
            //对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
            pangolin::ModelViewLookAt(0,-10,0.1,0,0,0,pangolin::AxisNegY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    //setBounds 跟opengl的viewport 有关
    //看SimpleDisplay中边界的设置就知道
    pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0,1.0,0.0,1.0,-640.0f/480.0f)
                            .SetHandler(&handler);

    while(!pangolin::ShouldQuit())
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // Render OpenGL Cube
//        pangolin::glDrawColouredCube();\
        //坐标轴的创建
        pangolin::glDrawAxis(3);

        //点的创建
        glPointSize(10.0f);
        glBegin(GL_POINTS);
        glColor3f(1.0,1.0,1.0);
        glVertex3f(0.0f,0.0f,0.0f);
        glVertex3f(1,0,0);
        glVertex3f(0,2,0);
        glEnd();

        //把下面的点都做一次旋转变换
        glPushMatrix();
        //col major
        std::vector<GLfloat > Twc = {1,0,0,0, 0,1,0,0 , 0,0,1,0 ,0,0,5,1};
        glMultMatrixf(Twc.data());

        //直线的创建
        const float w = 0.2;
        const float h = w*0.75;
        const float z = w*0.6;
        glLineWidth(2);
        glColor3f(1.0,0,0);
        glBegin(GL_LINES);

        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);
        glVertex3f(-w,h,z);
        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);
        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glVertex3f(w,-h,z);
        glVertex3f(w,h,z);
        glEnd();

        glPopMatrix();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    return 0;

}
#else 0

#include "PangoVis.h"
#include <GL/glut.h>


void drawBackground()
{
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glColor4f(1.0, 1.0, 1.0, 0.5);
    // construct a strip of quads, with each pair being one of the corners.

    glBegin(GL_QUAD_STRIP);
        glVertex2f(0.0, 0.0);          glVertex2i(3, 3);
        glVertex2f(5, 0.0);            glVertex2i(3, 3);
        glVertex2f(5, 5);              glVertex2i(3, 3);
        glVertex2f(0.0, 5);            glVertex2i(3, 3);
        glVertex2f(0.0, 0.0);          glVertex2i(3, 3);
    glEnd();

    // draw lines around cropped area.
    // we want to invert the color to make it stand out.
    glBlendFunc(GL_ONE_MINUS_DST_COLOR, GL_ZERO);
    glColor3f(1.0, 0.0, 1.0);
    glBegin(GL_LINE_LOOP);
        glVertex2i(3, 3);
        glVertex2i(3, 3);
        glVertex2i(3, 3);
        glVertex2i(3, 3);
    glEnd();
    glDisable(GL_BLEND);
}

void drawRoad()
{
    glBegin(GL_QUADS);

    float width = 3.f;
    float length = 10.f;
    float height = 4.6f;

    glVertex3f(0, width, height);
    glVertex3f(length, width, height);
    glVertex3f(length,-width, height);
    glVertex3f(0,-width, height);

    glEnd();

    //点的创建
    /*
    glPointSize(10.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0,1.0,1.0);
    glVertex3f(length, width, height);
    glEnd();
    */
}

int main(int argc, char *argv[])
{
//    PangoVis *pangoVis = new PangoVis();

    //创建一个窗口
    pangolin::CreateWindowAndBind("Main",640,480);
    //启动深度测试
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
            //对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
            pangolin::ModelViewLookAt(0,-10,0.1,0,0,0,pangolin::AxisNegY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    //setBounds 跟opengl的viewport 有关
    //看SimpleDisplay中边界的设置就知道
    pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0,1.0,0.0,1.0,-640.0f/480.0f)
                            .SetHandler(&handler);

    //added by flq
    const int UI_WIDTH = 180;
    pangolin::CreatePanel("ui")
        .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
    pangolin::Var<bool> a_button("ui.A_Button",false,false);
    pangolin::Var<double> a_double("ui.A_Double",3,0,5);
    pangolin::Var<int> an_int("ui.An_Int",2,0,5);
    pangolin::Var<double> a_double_log("ui.Log_scale var",3,1,1E4, true);
    pangolin::Var<bool> a_checkbox("ui.A_Checkbox",false,true);
    pangolin::Var<int> an_int_no_input("ui.An_Int_No_Input",2);
    //pangolin::Var<CustomType> any_type("ui.Some_Type", CustomType(0,1.2f,"Hello") );

    pangolin::Var<bool> save_window("ui.Save_Window",false,false);
    pangolin::Var<bool> save_cube("ui.Save_Cube",false,false);

    pangolin::Var<bool> record_cube("ui.Record_Cube",false,false);

    // std::function objects can be used for Var's too. These work great with C++11 closures.
    //pangolin::Var<std::function<void(void)> > reset("ui.Reset", SampleMethod);

    // Demonstration of how we can register a keyboard hook to alter a Var
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'b', pangolin::SetVarFunctor<double>("ui.A_Double", 3.5));

    // Demonstration of how we can register a keyboard hook to trigger a method
    //pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r', SampleMethod);





    while(!pangolin::ShouldQuit())
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // Render OpenGL Cube
//        pangolin::glDrawColouredCube();
        //坐标轴的创建
#if 0
        //pangolin::glDrawAxis(3);
#else
        //draw the axes, to give a sense of orientation//GLRenderer.cpp
        double axis = 5;
        glBegin(GL_LINES);

        glColor3f(1,0,0);
        glVertex3f(0,0,0);
        glVertex3f(axis,0,0);
//        GLPrint("X");

        glColor3f(0,1,0);
        glVertex3f(0,0,0);
        glVertex3f(0,axis,0);

        glColor3f(0,0,1);
        glVertex3f(0,0,0);
        glVertex3f(0,0,axis);

        glEnd();
#endif
        //点的创建
        glPointSize(10.0f);
        glBegin(GL_POINTS);
        glColor3f(1.0,1.0,1.0);
        //glVertex3f(0.0f,0.0f,0.0f);
        //glVertex3f(1,0,0);
        glVertex3f(0,2,0);
        glEnd();

        glPointSize(10.0f);
        glBegin(GL_POINTS);
        glColor3f(0.0f,1.0f,1.0f);
        glVertex3f(0,2.2f,0);
        glEnd();


        //flq
        drawBackground();

        drawRoad();
//        constGLubyte* OpenGLVersion  = glGetString(GL_VERSION);//返回当前OpenGL实现的版本号
/*
        //把下面的点都做一次旋转变换
        glPushMatrix();
        //col major
        std::vector<GLfloat > Twc = {1,0,0,0, 0,1,0,0 , 0,0,1,0 ,0,0,5,1};
        glMultMatrixf(Twc.data());

        //直线的创建
        const float w = 2;
        const float h = w*0.75;
        const float z = w*0.6;
        glLineWidth(2);
        glColor3f(1.0,0,0);
        glBegin(GL_LINES);

        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);
        glVertex3f(-w,h,z);
        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);
        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glVertex3f(w,-h,z);
        glVertex3f(w,h,z);
        glEnd();

        glPopMatrix();
*/
        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    return 0;

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
#endif
