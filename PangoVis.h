#ifndef PANGOVIS_H_
#define PANGOVIS_H_

#include <pangolin/pangolin.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

#include <Eigen/Core>

#include "utils/ThreadObject.h"
#include "utils/utils.h"

class PangoVis : public ThreadObject
{
    public:
        PangoVis(cv::Mat * Intrinsics);
        //PangoVis();

        virtual ~PangoVis();

        void reset();

    private:
        bool inline process();

        void preCall();

        void postCall();

        void render();

        void handleInput();

        void removeAllClouds();

        void removeAllShapes();

        void removeAllMeshes();

        void processClouds();

        void processTsdf();

        void processMeshes();

        void processImages();

        pangolin::OpenGlRenderState s_cam;

        int latestDrawnPoseCloudId;
        int latestDrawnMeshId;
        int numPoints;
        int numTriangles;
        int numTrianglePoints;

        Eigen::Matrix4f pose;
        Eigen::Matrix3f K, Kinv;
        Eigen::AlignedBox3f tsdfCube;

//        PangoCloud * liveTSDF;
//        PangoMesh * incMesh;
//        std::vector<PangoCloud*> clouds;
//        std::vector<PangoMesh*> meshes;
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> poses;
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>, Eigen::aligned_allocator<std::pair<Eigen::Vector3f, Eigen::Vector3f>>> lines;
/*
        pangolin::Var<bool> complete;
        pangolin::Var<bool> pause;
        pangolin::Var<bool> save;
        pangolin::Var<bool> resetAll;
        pangolin::Var<bool> volumeShifting;
        pangolin::Var<bool> limitFrontend;

        pangolin::Var<bool> followPose;
        pangolin::Var<bool> drawTSDF;
        pangolin::Var<bool> drawCloud;
        pangolin::Var<bool> drawMesh;
        pangolin::Var<bool> drawMeshNormals;

        pangolin::Var<std::string> totalPoints;
        pangolin::Var<std::string> totalTriangles;
        pangolin::Var<std::string> frame;
        pangolin::Var<std::string> frontendFps;
        pangolin::Var<std::string> backendLag;
        pangolin::Var<std::string> status;
*/
        pangolin::Var<bool> SnapShot;
        pangolin::Var<bool> PreviewDisplay;
        pangolin::Var<std::string> SnapCount;
        pangolin::Var<std::string> frontendFps;
        pangolin::Var<double> pointX;
        pangolin::Var<double> pointY;
        pangolin::Var<double> pointZ;
        //pangolin::Var<int> intnum;


        pangolin::GlTexture rgbTex,
                            depthTex,
                            tsdfRgbTex,
                            tsdfTex;

        pangolin::ManagedImage<unsigned char> rgbImg;
//        pangolin::ManagedImage<uchar3> tsdfImg;
//        pangolin::ManagedImage<uchar3> tsdfImgColor;
//        pangolin::ManagedImage<uchar3> depthImg;

        unsigned short * depthBuffer;

        void drawRoad();
        void drawBackground();
        void drawAxis();
        void drawSonaCamera();
        void drawXYZPointAndLine(double x, double y, double z);
};

#endif //PANGOVIS_H_
