//#include "MainController.h"
#include "PangoVis.h"
#include "frontend/Resolution.h"
#include "MainController.h"

PangoVis::PangoVis(cv::Mat * Intrinsics)
 : ThreadObject("VisualisationThread"),
   numTriangles(0),
   SnapShot("ui.SnapShot", false, false),
   PreviewDisplay("ui.PreviewDisplay", false, true),
   SnapCount("ui.SnapCount:", "0"),
   frontendFps("ui.Frontend:", "30fps"),
   pointX("ui.pointX", 1, -5, 5),
   pointY("ui.pointY", CAMERA_POSTION_Y, -5, 5),
   pointZ("ui.pointZ", 1, -5, 20)
   //intnum("ui.An_Int",2,0,5)
{
    printf("PangoVis,UI\n");
    pangolin::CreateWindowAndBind("FLQ", 1280 + 180, 960);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    rgbTex.Reinitialise(Resolution::get().width(), Resolution::get().height());
    rgbImg.Reinitialise(Resolution::get().width(), Resolution::get().height());

    glEnable(GL_DEPTH_TEST);

    s_cam = pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
                                        pangolin::ModelViewLookAt(-0.35, -2.3, -6.4, 0, 0, 0, 0, -1, 0));
                                        //pangolin::ModelViewLookAt(1, 1, 1, 0, 0, 0, 0, pangolin::AxisY, 0));

    pangolin::Display("cam").SetBounds(0, 1.0f, 0, 1.0f, -640 / 480.0)
                            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::Display("Img").SetAspect(640.0f / 480.0f);
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));

    pangolin::Display("multi").SetBounds(pangolin::Attach::Pix(0), 1 / 4.0f, pangolin::Attach::Pix(180), 1.0)
                              .SetLayout(pangolin::LayoutEqualHorizontal)
                              .AddDisplay(pangolin::Display("Img"));

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
    while(1)
        process();
}

#if 0
PangoVis::PangoVis()
{
#if 0
    pangolin::CreateWindowAndBind("Kintinuous", 1280 + 180, 960);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    glEnable(GL_DEPTH_TEST);

    s_cam = pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
                                        pangolin::ModelViewLookAt(-0.35, -2.3, -6.4, 0, 0, 0, 0, -1, 0));

    pangolin::Display("cam").SetBounds(0, 1.0f, 0, 1.0f, -640 / 480.0)
                            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::Display("Img").SetAspect(640.0f / 480.0f);
    pangolin::Display("Depth").SetAspect(640.0f / 480.0f);
    pangolin::Display("ModelImg").SetAspect(640.0f / 480.0f);
    pangolin::Display("Model").SetAspect(640.0f / 480.0f);

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));

    pangolin::Display("multi").SetBounds(pangolin::Attach::Pix(0), 1 / 4.0f, pangolin::Attach::Pix(180), 1.0)
                              .SetLayout(pangolin::LayoutEqualHorizontal)
                              .AddDisplay(pangolin::Display("Img"))
                              .AddDisplay(pangolin::Display("Depth"))
                              .AddDisplay(pangolin::Display("ModelImg"))
                              .AddDisplay(pangolin::Display("Model"));



    reset();
#endif
    //process();

    //创建一个窗口
    pangolin::CreateWindowAndBind("FLQ",1280 + 180, 960);
    rgbTex.Reinitialise(Resolution::get().width(), Resolution::get().height()),
    printf("%d,\n",Resolution::get().width());

    //启动深度测试
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
            //对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
            pangolin::ModelViewLookAt(0,-10,0.1,0,0,0,pangolin::AxisNegY)
            //pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
            //pangolin::ModelViewLookAt(-0.35, -2.3, -6.4, 0, 0, 0, 0, -1, 0)
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
//        pangolin::glDrawColouredCube();\
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

//GLPrint("Y");
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

}
#endif

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

    removeAllClouds();
    removeAllShapes();
    removeAllMeshes();
/*
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

    usleep(std::max(1, int(33000 - (Stopwatch::get().getTimings().at(threadIdentifier) * 1000))));

    return true;
}

void PangoVis::processClouds()
{
#if 0
    int latestDensePoseIdCopy = threadPack.tracker->latestDensePoseId.getValue();

    if(latestDensePoseIdCopy > 0)
    {
        const float3 & voxelSizeMeters = Volume::get().getVoxelSizeMeters();

        pose = threadPack.isamOffset.getValue() * threadPack.loopOffset.getValue() * threadPack.tracker->densePoseGraph.at(latestDensePoseIdCopy - 1).pose;

        Eigen::Vector3f translation = pose.topRightCorner(3, 1);

        Eigen::Vector3f initialTrans = Eigen::Vector3f::Constant(Volume::get().getVolumeSize() * 0.5) - threadPack.tracker->getVolumeOffset();

        Eigen::Vector3f currentCubeTranslation = initialTrans;
        currentCubeTranslation(0) += std::floor(translation(0) / voxelSizeMeters.x) * voxelSizeMeters.x;
        currentCubeTranslation(1) += std::floor(translation(1) / voxelSizeMeters.y) * voxelSizeMeters.y;
        currentCubeTranslation(2) += std::floor(translation(2) / voxelSizeMeters.z) * voxelSizeMeters.z;

        //Kinda hacky
        if(volumeShifting)
        {
            tsdfCube.setEmpty();
            tsdfCube.extend(currentCubeTranslation + Eigen::Vector3f::Constant(Volume::get().getVolumeSize() / 2.0f));
            tsdfCube.extend(currentCubeTranslation - Eigen::Vector3f::Constant(Volume::get().getVolumeSize() / 2.0f));
        }

        pangolin::glDrawAlignedBox(tsdfCube);

        glColor3f(0, 1, 0);
        pangolin::glDrawFrustum(Kinv, Resolution::get().width(), Resolution::get().height(), pose, 0.1f);
        glColor3f(1, 1, 1);
    }

    int latestPoseIdCopy = threadPack.latestPoseId.getValue();

    if(ConfigArgs::get().onlineDeformation)
    {
        boost::mutex::scoped_lock lock(threadPack.poolMutex, boost::try_to_lock);

        if(lock)
        {
            if(threadPack.poolLooped.getValue())
            {
                removeAllClouds();
                numPoints = 0;
                threadPack.poolLooped.assignValue(false);
            }

            if((int)threadPack.pointPool->size() != numPoints)
            {
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

                tempCloud->points.insert(tempCloud->end(), threadPack.pointPool->begin() + numPoints, threadPack.pointPool->end());

                lock.unlock();

                numPoints = threadPack.pointPool->size();

                static int i = 0;
                std::stringstream strs;
                strs << "pool" << i++;

                clouds.push_back(new PangoCloud(tempCloud.get()));
            }
            else
            {
                lock.unlock();
            }
        }

        removeAllShapes();

        for(int i = 2; i < latestPoseIdCopy; i++)
        {
            if(ConfigArgs::get().dynamicCube && !threadPack.cloudSlices.at(i)->poseIsam.getValue())
                break;

            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose.topLeftCorner(3, 3) = threadPack.cloudSlices.at(i)->cameraRotation;
            pose.topRightCorner(3, 1) = threadPack.cloudSlices.at(i)->cameraTranslation;
            poses.push_back(pose);

            if(i > 2)
            {
                lines.push_back(std::pair<Eigen::Vector3f, Eigen::Vector3f>(threadPack.cloudSlices.at(i)->cameraTranslation,
                                                                            threadPack.cloudSlices.at(i - 1)->cameraTranslation));
            }
        }
    }
    else
    {
        while(latestDrawnPoseCloudId < latestPoseIdCopy)
        {
            clouds.push_back(new PangoCloud(threadPack.cloudSlices.at(latestDrawnPoseCloudId)->processedCloud));

            numPoints += threadPack.cloudSlices.at(latestDrawnPoseCloudId)->processedCloud->size();

            if(latestDrawnPoseCloudId > 1)
            {
                Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
                pose.topLeftCorner(3, 3) = threadPack.cloudSlices.at(latestDrawnPoseCloudId)->cameraRotation;
                pose.topRightCorner(3, 1) = threadPack.cloudSlices.at(latestDrawnPoseCloudId)->cameraTranslation;
                poses.push_back(pose);

                if(latestDrawnPoseCloudId > 2)
                {
                    lines.push_back(std::pair<Eigen::Vector3f, Eigen::Vector3f>(threadPack.cloudSlices.at(latestDrawnPoseCloudId)->cameraTranslation,
                                                                                threadPack.cloudSlices.at(latestDrawnPoseCloudId - 1)->cameraTranslation));
                }
            }
            latestDrawnPoseCloudId++;
        }
    }
#endif
}

void PangoVis::processTsdf()
{
#if 0
    if(threadPack.finalised.getValue())
    {
        if(liveTSDF)
        {
            delete liveTSDF;
            liveTSDF = 0;
        }
    }
    else if(drawTSDF)
    {
        boost::mutex::scoped_lock tsdfLock(threadPack.tracker->tsdfMutex);

        if(threadPack.tracker->tsdfAvailable)
        {
            if(liveTSDF)
            {
                delete liveTSDF;
                liveTSDF = 0;
            }

            if(ConfigArgs::get().dynamicCube)
            {
                pcl::transformPointCloud(*threadPack.tracker->getLiveTsdf()->cloud, *threadPack.tracker->getLiveTsdf()->cloud, threadPack.isamOffset.getValue());
            }

            liveTSDF = new PangoCloud(threadPack.tracker->getLiveTsdf()->cloud);

            threadPack.tracker->tsdfAvailable = false;
        }
    }
#endif
}

void PangoVis::processMeshes()
{
#if 0
    int latestMeshIdCopy = threadPack.latestMeshId.getValue();

    if(ConfigArgs::get().incrementalMesh)
    {
        boost::mutex::scoped_lock lock(threadPack.incMeshMutex, boost::try_to_lock);

        if(lock)
        {
            int numIncPoints = threadPack.incrementalMesh->mesh->cloud.width * threadPack.incrementalMesh->mesh->cloud.height;

            bool looped = threadPack.incMeshLooped.getValue();

            if((int)threadPack.incrementalMesh->mesh->polygons.size() != numTriangles || numIncPoints != numTrianglePoints || looped)
            {
                removeAllMeshes();

                numTriangles = threadPack.incrementalMesh->mesh->polygons.size();
                numTrianglePoints = numIncPoints;

                if(incMesh)
                {
                    delete incMesh;
                    incMesh = 0;
                }

                incMesh = new PangoMesh(threadPack.incrementalMesh->mesh.get());

                if(looped)
                {
                    threadPack.incMeshLooped.assignValue(false);
                }
            }
        }
    }
    else
    {
        while(latestDrawnMeshId < latestMeshIdCopy)
        {
            if(threadPack.triangles.at(latestDrawnMeshId)->polygons.size() > 0)
            {
                meshes.push_back(new PangoMesh(threadPack.triangles.at(latestDrawnMeshId)));
                numTriangles += threadPack.triangles.at(latestDrawnMeshId)->polygons.size();
            }
            latestDrawnMeshId++;
        }
    }
#endif
}

void PangoVis::render()
{
    glDisable(GL_DEPTH_TEST);

    pangolin::Display("Img").Activate();
    rgbTex.RenderToViewport(true);

    glEnable(GL_DEPTH_TEST);

#if 0
    if((ConfigArgs::get().incrementalMesh || ConfigArgs::get().enableMeshGenerator) && drawMesh)
    {
        for(size_t i = 0; i < meshes.size(); i++)
        {
            meshes.at(i)->drawTriangles(drawMeshNormals);
        }

        if(incMesh)
        {
            incMesh->drawTriangles(drawMeshNormals);
        }
    }
    else if(drawCloud)
    {
        for(size_t i = 0; i < clouds.size(); i++)
        {
            clouds.at(i)->drawPoints();
        }

    }

    if(liveTSDF)
    {
        liveTSDF->drawPoints();
    }

    glColor3f(1, 0, 1);
    for(size_t i = 0; i < lines.size(); i++)
    {
        pangolin::glDrawLine(lines.at(i).first(0),
                             lines.at(i).first(1),
                             lines.at(i).first(2),
                             lines.at(i).second(0),
                             lines.at(i).second(1),
                             lines.at(i).second(2));
    }

    glColor3f(1, 1, 1);
    for(size_t i = 0; i < poses.size(); i++)
    {
        pangolin::glDrawFrustum(Kinv, Resolution::get().width(), Resolution::get().height(), poses.at(i), 0.05f);
    }

    glDisable(GL_DEPTH_TEST);

    pangolin::Display("Img").Activate();
    rgbTex.RenderToViewport(true);

    pangolin::Display("Depth").Activate();
    depthTex.RenderToViewport(true);

    pangolin::Display("ModelImg").Activate();
    tsdfRgbTex.RenderToViewport(true);

    pangolin::Display("Model").Activate();
    tsdfTex.RenderToViewport(true);

    glEnable(GL_DEPTH_TEST);
#endif
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

    rgbTex.Upload(threadPack.tracker->lastRgbImage, GL_RGB, GL_UNSIGNED_BYTE);





/*
    boost::mutex::scoped_lock imageLock(threadPack.tracker->imageMutex, boost::try_to_lock);

    if(imageLock && threadPack.tracker->imageAvailable)
    {
        threadPack.tracker->imageAvailable = false;

        memcpy(tsdfImg.ptr, threadPack.tracker->getLiveImage()->tsdfImage, Resolution::get().numPixels() * 3);
        memcpy(tsdfImgColor.ptr, threadPack.tracker->getLiveImage()->tsdfImageColor, Resolution::get().numPixels() * 3);
        memcpy(rgbImg.ptr, threadPack.tracker->getLiveImage()->rgbImage, Resolution::get().numPixels() * 3);
        memcpy(&depthBuffer[0], threadPack.tracker->getLiveImage()->depthData, Resolution::get().numPixels() * 2);

        imageLock.unlock();

        float max = 0;

        for(int i = 0; i < Resolution::get().numPixels(); i++)
        {
            if(depthBuffer[i] > max)
            {
                max = depthBuffer[i];
            }
        }

        for(size_t i = 0; i < depthImg.Area(); i++)
        {
            depthImg[i].x = ((float)depthBuffer[i] / max) * 255.0f;
            depthImg[i].y = ((float)depthBuffer[i] / max) * 255.0f;
            depthImg[i].z = ((float)depthBuffer[i] / max) * 255.0f;
        }

        rgbTex.Upload(rgbImg.ptr, GL_RGB, GL_UNSIGNED_BYTE);
        depthTex.Upload(depthImg.ptr, GL_RGB, GL_UNSIGNED_BYTE);
        tsdfRgbTex.Upload(tsdfImgColor.ptr, GL_RGB, GL_UNSIGNED_BYTE);
        tsdfTex.Upload(tsdfImg.ptr, GL_BGR, GL_UNSIGNED_BYTE);

        //For a minimal "TSDF" visualisation
        if(!drawTSDF)
        {
            if(liveTSDF)
            {
                delete liveTSDF;
                liveTSDF = 0;
            }

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);

            for(int i = 0; i < Resolution::get().width(); i++)
            {
                for(int j = 0; j < Resolution::get().height(); j++)
                {
                    if(depthBuffer[j * Resolution::get().width() + i])
                    {
                        pcl::PointXYZRGB pt;
                        pt.z = depthBuffer[j * Resolution::get().width() + i] * 0.001f;
                        pt.x = (static_cast<float>(i) - K(0, 2)) * pt.z * (1.0f / K(0, 0));
                        pt.y = (static_cast<float>(j) - K(1, 2)) * pt.z * (1.0f / K(1, 1));
                        pt.b = rgbImg(i, j).x;
                        pt.g = rgbImg(i, j).y;
                        pt.r = rgbImg(i, j).z;
                        cloud->push_back(pt);
                    }
                }
            }

            pcl::transformPointCloud(*cloud, *cloud, pose);

            liveTSDF = new PangoCloud(cloud.get());
        }
    }
    else if(imageLock)
    {
        imageLock.unlock();
    }
*/
}

void PangoVis::handleInput()
{
    if(pangolin::Pushed(SnapShot))
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
/*
    std::stringstream strsf;
    strsf << int(std::ceil(1.0f / (Stopwatch::get().getTimings().at("TrackerInterfaceThread") / 1000.0f))) << "Hz";
    frontendFps = strsf.str();

    std::stringstream strsb;
    strsb << int(float(MainController::controller->getMaxLag()) / 1000.0f) << "ms";
    backendLag = strsb.str();

    std::stringstream strsfr;
    strsfr << threadPack.trackerFrame.getValue();
    frame = strsfr.str();

*/
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
    glVertex3f(CAMERA_POSTION_X,CAMERA_POSTION_Y,CAMERA_POSTION_Z);
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
    glVertex3f(CAMERA_POSTION_X, CAMERA_POSTION_Y, CAMERA_POSTION_Z);
    glVertex3f(x,y,z);
    glEnd();
    glColor3f(1,1,1);
}
