#ifndef LIVELOGREADER_H_
#define LIVELOGREADER_H_

#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <signal.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include "LogReader.h"
//#ifdef HIKVISION_GRAB_FUNC
#include "utils/HikGrab.h"
//#else
#include "utils/usbGrab.h"
//#endif
#include "utils/Macros.h"
//#include "OpenNI2Interface.h"

class LiveLogReader : public LogReader
{
    public:
        LiveLogReader();
        virtual ~LiveLogReader();

        bool grabNext(bool& returnVal, int& frame_idx);
        bool grabTrigerdNext(bool& returnVal/*, int& frame_idx*/);

    private:
#ifdef HIKVISION_GRAB_FUNC
        HikGrab *m_hikgrab;
#else
        usbGrab *m_usbgrab;
#endif
        int64_t lastFrameTime;
        int lastGot;
};

#endif /* LIVELOGREADER_H_ */
