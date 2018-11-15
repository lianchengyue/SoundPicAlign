#ifndef UTILS_H
#define UTILS_H

//角度制,弧度制.    360° = 2π
#define PI 3.141592f

#define CAMERA_POSTION_X 0
#define CAMERA_POSTION_Y 2.2f
#define CAMERA_POSTION_Z 0

#define CAMERA_EULER_X 0
#define CAMERA_EULER_Y (360 - 45)//调节俯仰角，面向地面为360-0,面向正前方为360-90
#define CAMERA_EULER_Z (360 - 90)//调节偏航角,旋转到横屏


#define SONA_POSTION_X 0
#define SONA_POSTION_Y 2.0f
#define SONA_POSTION_Z 0

//#define RESOLUTION_WIDTH 1920
//#define RESOLUTION_HEIGHT 1080

#define RESOLUTION_WIDTH 640
#define RESOLUTION_HEIGHT 480

#endif // UTILS_H
