//#include "MainController.h"
#include "PangoVis.h"
#include "frontend/Resolution.h"
#include "MainController.h"

PangoVis::PangoVis(cv::Mat * Intrinsics)
 : ThreadObject("VisualisationThread"),
   numTriangles(0),
   SnapShotBtn("ui.SnapShot", false, false),
   PreviewDisplay("ui.PreviewDisplay", false, true),
   SnapCount("ui.SnapCount:", "0"),
   frontendFps("ui.Frontend:", "30fps"),
   pointX("ui.pointX", 1, -5, 5),
   pointY("ui.pointY", /*CAMERA_POSTION_Y*/2.2f, -5, 5),
   pointZ("ui.pointZ", 1, -5, 20)
   //intnum("ui.An_Int",2,0,5)
{
    printf("PangoVis,UI\n");
    pangolin::CreateWindowAndBind("FLQ", 1280 + 180, 960);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    rgbTex.Reinitialise(Resolution::get().width(), Resolution::get().height());
    rgbImg.Reinitialise(Resolution::get().width(), Resolution::get().height());

    rgbVideoTex.Reinitialise(Resolution::get().width(), Resolution::get().height());
    rgbVideo.Reinitialise(Resolution::get().width(), Resolution::get().height());

    glEnable(GL_DEPTH_TEST);

    s_cam = pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
                                        //pangolin::ModelViewLookAt(-0.35, -2.3, -6.4, 0, 0, 0, 0, -1, 0));
                                        pangolin::ModelViewLookAt(0, 0, -6.4, 0, 0, 0, 0, -1, 0));
                                        //pangolin::ModelViewLookAt(1, 1, 1, 0, 0, 0, 0, pangolin::AxisY, 0));

    pangolin::Display("cam").SetBounds(0, 1.0f, 0, 1.0f, -640 / 480.0)
                            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::Display("Img").SetAspect(640.0f / 480.0f);
    pangolin::Display("Video").SetAspect(640.0f / 480.0f);

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));

    pangolin::Display("multi").SetBounds(pangolin::Attach::Pix(0), 1 / 4.0f, pangolin::Attach::Pix(180), 1.0)
                              .SetLayout(pangolin::LayoutEqualHorizontal)
                              .AddDisplay(pangolin::Display("Img"))
                              .AddDisplay(pangolin::Display("Video"));

    K = Eigen::Matrix3f::Identity();
    K(0, 0) = Intrinsics->at<double>(0,0);
    K(1, 1) = Intrinsics->at<double>(1,1);
    K(0, 2) = Intrinsics->at<double>(0,2);
    K(1, 2) = Intrinsics->at<double>(1,2);

    //flq
    //printf("%f,%f,%f,%f\n", K(0, 0), K(1, 1), K(0, 2), K(1, 2));
    Kinv = K.inverse();

    //calcRMatrix();
//    threadPack.tracker->lastRgbImage

    //不再需要在主线程中循环，在Pangovis线程中循环,start
///    while(1)
///        process();
    //不再需要在主线程中循环，在Pangovis线程中循环,end
}

PangoVis::~PangoVis()
{
    reset();

    delete [] depthBuffer;
}

void PangoVis::removeAllClouds()
{
/*
    for(size_t i = 0; i < clouds.size(); i++)
    {
        delete clouds.at(i);
    }

    clouds.clear();
*/
}

void PangoVis::removeAllShapes()
{
    lines.clear();
    poses.clear();
}

void PangoVis::removeAllMeshes()
{
/*
    for(size_t i = 0; i < meshes.size(); i++)
    {
        delete meshes.at(i);
    }

    meshes.clear();
*/
}

void PangoVis::reset()
{
    latestDrawnMeshId = 0;
    numPoints = 0;
    numTriangles = 0;
    numTrianglePoints = 0;
    latestDrawnPoseCloudId = 0;

/*
    removeAllClouds();
    removeAllShapes();
    removeAllMeshes();

    if(liveTSDF)
    {
        delete liveTSDF;
        liveTSDF = 0;
    }

    if(incMesh)
    {
        delete incMesh;
        incMesh = 0;
    }
*/
}

void PangoVis::preCall()
{
    glClearColor(0.25, 0.25, 0.25, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    pangolin::Display("cam").Activate(s_cam);
}

bool inline PangoVis::process()
{
    TICK(threadIdentifier);

///    printf("PangoVisThread start!\n");
    if(pangolin::ShouldQuit())
    {
        //MainController::controller->shutdown();
        return false;
    }

    preCall();


    drawBackground();
    drawRoad();
    drawAxis();
    drawSonaCamera();
    drawXYZPointAndLine(pointX, pointY, pointZ);

    processTsdf();

    processClouds();

    processMeshes();

    processImages();

    render();

    postCall();

    handleInput();

    TOCK(threadIdentifier);

    //flq, delete temp
///    usleep(std::max(1, int(33000 - (Stopwatch::get().getTimings().at(threadIdentifier) * 1000))));

    return true;
}

void PangoVis::processClouds()
{
    //...
}

void PangoVis::processTsdf()
{
    //...
}

void PangoVis::processMeshes()
{
    //...
}

void PangoVis::render()
{
    glDisable(GL_DEPTH_TEST);

    pangolin::Display("Img").Activate();
    rgbTex.RenderToViewport(true);

    pangolin::Display("Video").Activate();
    rgbVideoTex.RenderToViewport(true);

    glEnable(GL_DEPTH_TEST);
}

void PangoVis::processImages()
{
    boost::mutex imageMutex;
    boost::mutex::scoped_lock imageLock(/*threadPack.tracker->*/imageMutex, boost::try_to_lock);

    if(imageLock/* && threadPack.tracker->imageAvailable*/)
    {
    //    memcpy(rgbImg.ptr, threadPack.tracker->getLiveImage()->rgbImage, Resolution::get().numPixels() * 3);
//        memcpy(rgbImg.ptr, threadPack.tracker->getRGBImage(), Resolution::get().numPixels() * 3);
///        printf("rgbImage:0x%x\n", threadPack.tracker->lastRgbImage);
///        printf(" strlen(threadPack.tracker->lastRgbImage)=%d\n", strlen((char*)threadPack.tracker->lastRgbImage));
        imageLock.unlock();
    }
    else if(imageLock)
    {
        imageLock.unlock();
    }

    //flq,pangovis图像显示部分
    //if(threadPack.tracker->lastRgbImage/* && threadPack.tracker->lastVideoImage*/)
    if(threadPack.tracker->lastVideoImage)
    {
        //声源定位显示
        rgbTex.Upload(threadPack.tracker->lastRgbImage, GL_RGB, GL_UNSIGNED_BYTE);
        //视频显示
        rgbVideoTex.Upload(threadPack.tracker->lastVideoImage, GL_RGB, GL_UNSIGNED_BYTE);
    }

}

void PangoVis::handleInput()
{
    if(pangolin::Pushed(SnapShotBtn))
    {
        double pointx, pointy, pointz;
        pointx = pointX;
        pointy = pointY;
        pointz = pointZ;
        //drawXYZPointAndLine(pointx, pointy, pointz);
        MainController::controller->SnapShot(pointx, pointy, pointz);
    }

    if(PreviewDisplay.GuiChanged())
    {
//        threadPack.pauseCapture.assignValue(pause);
        //pangolin::Display("Img").Activate();
        //rgbTex.RenderToViewport(false);
    }
#if 0
    //So this is some hilarious access/control!
    if(pangolin::Pushed(complete))
    {
//        MainController::controller->complete();
        followPose = false;
    }

    if(pause.GuiChanged())
    {
//        threadPack.pauseCapture.assignValue(pause);
    }

    if(pangolin::Pushed(save))
    {
//        MainController::controller->save();
    }

    if(pangolin::Pushed(resetAll))
    {
//        MainController::controller->reset();
//        MainController::controller->setPark(!volumeShifting);
//        threadPack.pauseCapture.assignValue(pause);
    }

    if(volumeShifting.GuiChanged())
    {
//        MainController::controller->setPark(!volumeShifting);
    }

    if(drawTSDF.GuiChanged())
    {
//        threadPack.tracker->tsdfRequest.assignValue(drawTSDF);
    }

    if(limitFrontend.GuiChanged())
    {
//        threadPack.limit.assignValue(limitFrontend);
    }

    if(pause)
    {
        status = "Paused";
    }
/*
    else if(threadPack.cloudSliceProcessorFinished.getValue() &&
            threadPack.meshGeneratorFinished.getValue() &&
            threadPack.placeRecognitionFinished.getValue() &&
            threadPack.deformationFinished.getValue())
    {
        status = "Finished";
    }
*/
    else
    {
        status = "Running";
    }

    if(followPose)
    {
        pangolin::OpenGlMatrix mv;

        Eigen::Matrix4f currPose = pose;
        Eigen::Matrix3f currRot = currPose.topLeftCorner(3, 3);

        Eigen::Quaternionf currQuat(currRot);
        Eigen::Vector3f forwardVector(0, 0, 1);
        Eigen::Vector3f upVector(0, -1, 0);

        Eigen::Vector3f forward = (currQuat * forwardVector).normalized();
        Eigen::Vector3f up = (currQuat * upVector).normalized();

        Eigen::Vector3f eye(currPose(0, 3), currPose(1, 3), currPose(2, 3));

        eye -= forward * 20;

        Eigen::Vector3f at = eye + forward;

        Eigen::Vector3f z = (eye - at).normalized();  // Forward
        Eigen::Vector3f x = up.cross(z).normalized(); // Right
        Eigen::Vector3f y = z.cross(x);

        Eigen::Matrix4d m;
        m << x(0),  x(1),  x(2),  -(x.dot(eye)),
             y(0),  y(1),  y(2),  -(y.dot(eye)),
             z(0),  z(1),  z(2),  -(z.dot(eye)),
                0,     0,     0,              1;

        memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));

        s_cam.SetModelViewMatrix(mv);
    }

    std::stringstream strs;
    strs << numPoints;
    totalPoints = strs.str();

    std::stringstream strst;
    strst << numTriangles;
    totalTriangles = strst.str();
#endif
}

void PangoVis::postCall()
{
    pangolin::FinishFrame();
}


void PangoVis::drawBackground()
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

void PangoVis::drawRoad()
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

void PangoVis::drawAxis()
{
    double axis = 3;
    glBegin(GL_LINES);

    glColor3f(1,0,0);
    glVertex3f(0,0,0);
    glVertex3f(axis,0,0);
//        GLPrint("X");

    glColor3f(0,1,0);
    glVertex3f(0,0,0);
    glVertex3f(0,axis,0);
//        GLPrint("Y");

    glColor3f(0,0,1);
    glVertex3f(0,0,0);
    glVertex3f(0,0,axis);
//        GLPrint("Z");

    glEnd();
}

void PangoVis::drawSonaCamera()
{
    glPointSize(10.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0,1.0,1.0);
    //glVertex3f(0.0f,0.0f,0.0f);
    //glVertex3f(1,0,0);
    glVertex3f(SONA_POSTION_X,SONA_POSTION_Y,SONA_POSTION_Z);
    glEnd();

    glPointSize(10.0f);
    glBegin(GL_POINTS);
    glColor3f(0.0f,1.0f,1.0f);
    glVertex3f(CAMERA_POSTION_X,-CAMERA_POSTION_Y,CAMERA_POSTION_Z);
    glEnd();

//added by flq
//    pangolin::glDrawAlignedBox(tsdfCube);

    //std::cout<<ThreadDataPack::get().RMatrix<<std::endl;
    //std::cout<<ThreadDataPack::get().TMatrix<<std::endl;

    Eigen::Matrix4f origin = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f pose = ThreadDataPack::get().finalpose;

//实时打印当前位姿
//    std::cout << pose << std::endl;
//    std::cout <<std::endl;

    glColor3f(1, 1, 1);
///    pangolin::glDrawFrustum(Kinv, Resolution::get().width(), Resolution::get().height(), origin, 0.1f);
    glColor3f(0, 1, 0);
    //绘制相机当前位姿态
    pangolin::glDrawFrustum(Kinv, Resolution::get().width(), Resolution::get().height(), pose, 0.1f);
    glColor3f(1, 1, 1);
}

void PangoVis::drawXYZPointAndLine(double x, double y, double z)
{
    glPointSize(10.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0,1.0,1.0);
    glVertex3f(x,y,z);
    glEnd();


    glBegin(GL_LINES);

    glColor3f(1,0,0);
    glVertex3f(CAMERA_POSTION_X, -CAMERA_POSTION_Y, CAMERA_POSTION_Z);
    glVertex3f(x,y,z);
    glEnd();
    glColor3f(1,1,1);
}
