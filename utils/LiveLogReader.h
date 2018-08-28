#ifndef LIVELOGREADER_H_
#define LIVELOGREADER_H_

#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <signal.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include "LogReader.h"
#include "utils/usbGrab.h"
//#include "OpenNI2Interface.h"

class LiveLogReader : public LogReader
{
    public:
        LiveLogReader();
        virtual ~LiveLogReader();

        bool grabNext(bool & returnVal, int & currentFrame);

    private:
//        OpenNI2Interface * asus;
        usbGrab *m_usbgrab;
        int64_t lastFrameTime;
        int lastGot;
};

#endif /* LIVELOGREADER_H_ */
