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
           decompressionBuffer(0)
//           deCompImage(0)
        {}

        virtual ~LogReader()
        {}

        virtual bool grabNext(bool & returnVal, int & currentFrame) = 0;

        unsigned char * decompressedImage;

        unsigned char * compressedImage;
        int32_t compressedDepthSize;
        int32_t compressedImageSize;

        int64_t timestamp;
        bool isCompressed;

        Bytef * decompressionBuffer;
        //IplImage * deCompImage;
        cv::Mat * deCompImage;
};

#endif /* LOGREADER_H_ */
