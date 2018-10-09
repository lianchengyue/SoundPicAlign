#ifndef LOGREADER_H_
#define LOGREADER_H_

#include <string>
#include <zlib.h>
#include <poll.h>
#include <opencv2/opencv.hpp>

//#include "ConfigArgs.h"
#include "../frontend/Resolution.h"
//#include "ThreadDataPack.h"

class LogReader
{
    public:
        LogReader()
         : compressedImage(0),
           decompressionBuffer(0),
           TrigerSavedBuffer(0)
//           deCompImage(0)
        {}

        virtual ~LogReader()
        {}

        virtual bool grabNext(bool& returnVal, int& frame_idx) = 0;
        virtual bool grabTrigerdNext(bool& returnVal/*, int& frame_idx*/) = 0;

        unsigned char * decompressedImage;
        //定位触发的同一时间的图片
        unsigned char * TrigerSavedImage;

        unsigned char * compressedImage;
        int32_t compressedDepthSize;
        int32_t compressedImageSize;

        int64_t timestamp;
        bool isCompressed;

        Bytef * decompressionBuffer;
        //定位触发的同一时间图片的buffer
        Bytef * TrigerSavedBuffer;
        //IplImage * deCompImage;
        cv::Mat * deCompImage;
};

#endif /* LOGREADER_H_ */
