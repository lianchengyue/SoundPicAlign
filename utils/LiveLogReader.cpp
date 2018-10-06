#include "LiveLogReader.h"

LiveLogReader::LiveLogReader()
 : lastFrameTime(0),
   lastGot(-1)
{
    printf("LiveLogReader\n");
#ifdef HIKVISION_GRAB_FUNC
    m_hikgrab = new HikGrab();
#else
    m_usbgrab = new usbGrab();
    //m_usbgrab->grab();
    //imwrite("../SoundPicAlign/1.jpg", *m_usbgrab->getCurrentFrame());
    //deCompImage = m_usbgrab->getCurrentFrame();
#endif



#if 0
    decompressionBuffer = new Bytef[Resolution::get().numPixels() * 2];
    deCompImage = cvCreateImage(cvSize(Resolution::get().width(), Resolution::get().height()), IPL_DEPTH_8U, 3);

    std::cout << "Creating live capture... "; std::cout.flush();

//    asus = new OpenNI2Interface(Resolution::get().width(), Resolution::get().height());

    if(!asus->ok())
    {
        std::cout << "failed!" << std::endl;
        std::cout << asus->error();
        exit(0);
    }
    else
    {
        std::cout << "success!" << std::endl;

        std::cout << "Waiting for first frame"; std::cout.flush();

        int lastDepth = asus->latestDepthIndex.getValue();

        do
        {
            usleep(33333);
            std::cout << "."; std::cout.flush();
            lastDepth = asus->latestDepthIndex.getValue();
        } while(lastDepth == -1);

        std::cout << " got it!" << std::endl;
    }
#endif
}

LiveLogReader::~LiveLogReader()
{
//    delete asus;
    delete [] decompressionBuffer;
//    cvReleaseImage(&deCompImage);
}

bool LiveLogReader::grabNext(bool & returnVal, int & currentFrame)
{
#ifdef HIKVISION_GRAB_FUNC
    int bufferIndex = m_hikgrab->getFrameIdx() % 10;

    deCompImage = m_hikgrab->getCurrentFrame();

    decompressedImage = (unsigned char *)deCompImage->data;
    printf("decompressedImage:0x%x, strlen(decompressedImage)=%d\n", decompressedImage, strlen((char*)decompressedImage));
#else
    int bufferIndex = m_usbgrab->getFrameIdx() % 10;

//    memcpy(m_usbgrab->frameBuffers[0], deCompImage->data, Resolution::get().numPixels() * 3);
    deCompImage = m_usbgrab->getCurrentFrame();

    decompressedImage = (unsigned char *)deCompImage->data;
    printf("decompressedImage:0x%x, strlen(decompressedImage)=%d\n", decompressedImage, strlen((char*)decompressedImage));
#endif

#if 0
    int lastDepth = asus->latestDepthIndex.getValue();

    if(lastDepth == -1)
    {
        return true;
    }

    int bufferIndex = lastDepth % 10;

    if(bufferIndex == lastGot)
    {
        return true;
    }

    if(lastFrameTime == asus->frameBuffers[bufferIndex].second)
    {
        return true;
    }

    memcpy(&decompressionBuffer[0], asus->frameBuffers[bufferIndex].first.first, Resolution::get().numPixels() * 2);
    memcpy(deCompImage->imageData, asus->frameBuffers[bufferIndex].first.second, Resolution::get().numPixels() * 3);

    lastFrameTime = asus->frameBuffers[bufferIndex].second;

    timestamp = lastFrameTime;
    isCompressed = false;

    decompressedImage = (unsigned char *)deCompImage->imageData;
    decompressedDepth = (unsigned short *)&decompressionBuffer[0];

    compressedImage = 0;
    compressedDepth = 0;

    compressedImageSize = Resolution::get().numPixels() * 3;
    compressedDepthSize = Resolution::get().numPixels() * 2;
/*
    if(ConfigArgs::get().flipColors)
    {
        cv::Mat3b rgb(Resolution::get().rows(),
                      Resolution::get().cols(),
                      (cv::Vec<unsigned char, 3> *)decompressedImage,
                      Resolution::get().width() * 3);

        cv::cvtColor(rgb, rgb, CV_RGB2BGR);
    }

    ThreadDataPack::get().trackerFrame.assignAndNotifyAll(currentFrame);
*/
#endif
    return true;
}
