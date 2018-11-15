#if 1
#include "PangoVis.h"
#include "MainController.h"

int main(int argc, char *argv[])
{
//    PangoVis *pangoVis = new PangoVis();

    MainController controller(argc, argv);
    return controller.start();

}
#else
#include <opencv2/opencv.hpp>
using namespace cv;
/*** 功能：  检查是否是旋转矩阵**/
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
    return  norm(I, shouldBeIdentity) < 1e-6;     }
/*** 功能： 通过给定的旋转矩阵计算对应的欧拉角**/
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If
    float x, y, z;     if (!singular) {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }

    std::cout<<"Vec3f(x, y, z):"<<Vec3f(x, y, z)<<" du"<<std::endl;

    double euler_X;
    double euler_Y;
    double euler_Z;

#define PI 3.141592f
    euler_X = x * 360.f / 2 / PI;
    euler_Y = y * 360.f / 2 / PI;
    euler_Z = z * 360.f / 2 / PI;
    std::cout<<"Vec3f(euler_X, euler_Y, euler_Z):"<<Vec3f(euler_X, euler_Y, euler_Z)<<" du"<<std::endl;

    return Vec3f(x, y, z);
}

int main()
{
/*
    float vals[] = {525., 0., 3.1950000000000000e+02,
                    0., 525., 2.3950000000000000e+02,
                    0., 0., 1.};
    Mat RMatrix = Mat(3,3,CV_64F,vals);//CV_32FC1
*/
//    cv::Mat RMatrix(3, 3, cv::DataType<float>::type);

    /*
    Mat RMatrix = (Mat_<double>(3,3) <<
                 0.8216967021625539, -0.5616453878807308, -0.09679353246723993,
                 0.5697289121595741, 0.8139357830574649, 0.113655214170312,
                 0.01494979279626867, -0.1485361886239672, 0.9887939645671457
               );
    */
    Mat RMatrix = (Mat_<double>(3,3) <<
                 1, 0, 5,
                 2, 1, 6,
                 3, 4, 0);

    std::cout << "原矩阵:" << RMatrix << std::endl;
    //逆矩阵
    RMatrix = RMatrix.inv();
    std::cout << "矩阵的逆:" << RMatrix << std::endl;
    //矩阵的转置
    transpose(RMatrix, RMatrix);
    std::cout << "矩阵的转置:" << RMatrix << std::endl;

    rotationMatrixToEulerAngles(RMatrix);
    return 0;
}

#endif
