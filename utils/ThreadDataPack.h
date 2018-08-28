#ifndef THREADDATAPACK_H_
#define THREADDATAPACK_H_

#include "ThreadMutexObject.h"

#include "../frontend/FrontProcessor.h"
#include <vector>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>

#include "frontend/FrontProcessor.h"

class ThreadDataPack
{
    public:
        static ThreadDataPack & get()
        {
            static ThreadDataPack instance;
            return instance;
        }

        virtual ~ThreadDataPack()
        {

        }

        void assignFrontend(FrontProcessor * frontend)
        {
            assert(!tracker);
            tracker = frontend;
        }

        void reset()
        {
            //We only delete this because the first item is the initial pose slice
            //created by the CloudSliceProcessor, the rest of the pointers are owned
            //by the KintinuousTracker and dealt with there



/*
            for(unsigned int i = 0; i < loopClosureConstraints.size(); i++)
            {
                delete loopClosureConstraints.at(i);
            }
            loopClosureConstraints.clear();
*/


            pauseCapture.assignValue(false);
            latestLoopId.assignValue(0);
            latestPoseId.assignValue(0);
            latestMeshId.assignValue(0);
            trackerFinished.assignValue(false);
            cloudSliceProcessorFinished.assignValue(false);
            meshGeneratorFinished.assignValue(false);
            placeRecognitionFinished.assignValue(false);
            deformationFinished.assignValue(false);
            trackerFrame.assignValue(0);
            finalised.assignValue(false);
            poolLooped.assignValue(false);
            limit.assignValue(true);
            incMeshLooped.assignValue(false);
            lastLoopTime.assignValue(0);
            readyForLoop.assignValue(true);

            boost::mutex::scoped_lock lock(poolMutex);

        }
        
        void notifyVariables()
        {
            latestLoopId.notifyAll();
            latestMeshId.notifyAll();
            latestPoseId.notifyAll();
        }

        boost::mutex poolMutex;
        ThreadMutexObject<bool> poolLooped;

        boost::mutex incMeshMutex;
        ThreadMutexObject<bool> incMeshLooped;

        ThreadMutexObject<bool> finalised;

        ThreadMutexObject<bool> limit;


        ThreadMutexObject<uint64_t> lastLoopTime;
        ThreadMutexObject<bool> readyForLoop;
//        std::vector<LoopClosureConstraint *> loopClosureConstraints;

        ThreadMutexObject<int> latestLoopId;
        ThreadMutexObject<int> latestMeshId;
        ThreadMutexObject<int> latestPoseId;
        ThreadMutexObject<bool> trackerFinished;
        ThreadMutexObject<bool> cloudSliceProcessorFinished;
        ThreadMutexObject<bool> meshGeneratorFinished;
        ThreadMutexObject<bool> placeRecognitionFinished;
        ThreadMutexObject<bool> deformationFinished;
        ThreadMutexObject<int> trackerFrame;
        ThreadMutexObject<bool> pauseCapture;

        FrontProcessor * tracker;

        //flq
        cv::Mat RMatrix;
        cv::Mat TMatrix;
        Eigen::Matrix4f finalpose;

    private:
        ThreadDataPack()
           : tracker(0)
        {
            reset();
        }
};

#endif /* THREADDATAPACK_H_ */
