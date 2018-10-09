#include "LiveLogReader.h"

LiveLogReader::LiveLogReader()
 : lastFrameTime(0),
   lastGot(-1)
{
    printf("LiveLogReader\n");

    decompressionBuffer = new unsigned char[Resolution::get().numPixels() * 3];
    TrigerSavedBuffer = new unsigned char[Resolution::get().numPixels() * 3];

#ifdef HIKVISION_GRAB_FUNC
    m_hikgrab = new HikGrab();
#else
    m_usbgrab = new usbGrab();
#endif
}

LiveLogReader::~LiveLogReader()
{
//    delete asus;
    delete [] decompressionBuffer;
    delete [] TrigerSavedBuffer;
//    cvReleaseImage(&deCompImage);
}

bool LiveLogReader::grabNext(bool& returnVal, int& frame_idx)
{
#if 0
#ifdef HIKVISION_GRAB_FUNC
    int bufferIndex = m_hikgrab->getFrameIdx() % 10;

    deCompImage = m_hikgrab->getCurrentFrame();

    if(NULL == deCompImage)
        return false;

    decompressedImage = (unsigned char *)deCompImage->data;
    printf("decompressedImage:0x%x, strlen(decompressedImage)=%d\n", decompressedImage, strlen((char*)decompressedImage));
#else
    int bufferIndex = m_usbgrab->getFrameIdx() % 10;

    deCompImage = m_usbgrab->getCurrentFrame();

    decompressedImage = (unsigned char *)deCompImage->data;
    printf("decompressedImage:0x%x, strlen(decompressedImage)=%d\n", decompressedImage, strlen((char*)decompressedImage));
#endif
#else
#ifdef HIKVISION_GRAB_FUNC
    int bufferIndex = m_hikgrab->getFrameIdx() % 10;

    deCompImage = m_hikgrab->getCurrentFrame();
    memcpy(&decompressionBuffer[0], m_hikgrab->frameBuffers[bufferIndex], Resolution::get().numPixels() * 3);

    decompressedImage = (unsigned char*)&decompressionBuffer[0];//decompressedImage 会显示到rgbVideo
    printf("decompressedImage:0x%x, strlen(decompressedImage)=%d\n", decompressedImage, strlen((char*)decompressedImage));
#else
    int bufferIndex = m_usbgrab->getFrameIdx() % 10;

    deCompImage = m_usbgrab->getCurrentFrame();
    memcpy(&decompressionBuffer[0], m_usbgrab->frameBuffers[bufferIndex], Resolution::get().numPixels() * 3);

    decompressedImage = (unsigned char*)&decompressionBuffer[0];//decompressedImage 会显示到rgbVideo
    printf("decompressedImage:0x%x, strlen(decompressedImage)=%d\n", decompressedImage, strlen((char*)decompressedImage));
#endif
#endif
    return true;
}


bool LiveLogReader::grabTrigerdNext(bool& returnVal/*, int& frame_idx*/)
{
#ifdef HIKVISION_GRAB_FUNC
    int bufferIndex = m_hikgrab->getFrameIdx() % 10;

    deCompImage = m_hikgrab->getCurrentFrame(); //can delete
    memcpy(&TrigerSavedBuffer[0], m_hikgrab->frameBuffers[bufferIndex], Resolution::get().numPixels() * 3);

    TrigerSavedImage = (unsigned char*)&TrigerSavedBuffer[0];//TrigerSavedImage 会显示到rgbVideo
    printf("TrigerSavedImage:0x%x, strlen(TrigerSavedImage)=%d\n", TrigerSavedImage, strlen((char*)TrigerSavedImage));
#else
    int bufferIndex = m_usbgrab->getFrameIdx() % 10;

    deCompImage = m_usbgrab->getCurrentFrame();
    memcpy(&TrigerSavedBuffer[0], m_usbgrab->frameBuffers[bufferIndex], Resolution::get().numPixels() * 3);

    TrigerSavedImage = (unsigned char*)&TrigerSavedBuffer[0];//TrigerSavedImage 会显示到rgbVideo
    printf("TrigerSavedImage:0x%x, strlen(TrigerSavedImage)=%d\n", TrigerSavedImage, strlen((char*)TrigerSavedImage));
#endif
    return true;
}
