#ifndef UTILS_H
#define UTILS_H

#if 1
//角度制,弧度制.    360° = 2π
#define PI 3.141592f

#define CAMERA_POSTION_X 0
#define CAMERA_POSTION_Y 2.2f
#define CAMERA_POSTION_Z 0

#define CAMERA_EULER_X 0         //翻滚角Roll
#define CAMERA_EULER_Y (360 - 45)//调节俯仰角Pitch，面向地面为360-0,面向正前方为360-90
#define CAMERA_EULER_Z (360 - 90)//调节偏航角Yaw,旋转到横屏

#define SONA_POSTION_X 0
#define SONA_POSTION_Y 2.0f
#define SONA_POSTION_Z 0

#define RESOLUTION_WIDTH 1920
#define RESOLUTION_HEIGHT 1080

//#define RESOLUTION_WIDTH 640
//#define RESOLUTION_HEIGHT 480
#else
//角度制,弧度制.    360° = 2π
#define PI 3.141592f

#define CAMERA_POSTION_X 0
#define CAMERA_POSTION_Y 0
#define CAMERA_POSTION_Z 0

#define CAMERA_EULER_X 0         //翻滚角Roll
#define CAMERA_EULER_Y (360 - 60)//调节俯仰角Pitch，面向地面为360-0,面向正前方为360-90
#define CAMERA_EULER_Z (360 - 90)//调节偏航角Yaw,旋转到横屏

#define SONA_POSTION_X 0
#define SONA_POSTION_Y 2.0f
#define SONA_POSTION_Z 0

//#define RESOLUTION_WIDTH 1920
//#define RESOLUTION_HEIGHT 1080
#define RESOLUTION_WIDTH 4096
#define RESOLUTION_HEIGHT 2160

//#define RESOLUTION_WIDTH 640
//#define RESOLUTION_HEIGHT 480
#endif

#endif // UTILS_H
