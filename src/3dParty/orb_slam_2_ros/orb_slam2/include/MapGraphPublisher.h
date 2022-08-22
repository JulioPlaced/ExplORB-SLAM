/**
* Created by jjgomez on 30/8/21.
*/

#ifndef ORB_SLAM2_ROS_MAPGRAPHPUBLISHER_H
#define ORB_SLAM2_ROS_MAPGRAPHPUBLISHER_H

#include "System.h"
#include "Map.h"
#include <stdio.h>

namespace ORB_SLAM2 {
    class System;
    class MapGraphPublisher {
        public:
            MapGraphPublisher() = delete;
            MapGraphPublisher(Map* pMap, System* pSystem);

            /*
             * Infinity loop to run in the separated thread
             */
            void run();

        private:
            Map* pMap_;
            System* pSystem_;

            bool bRun_;
    };
}

#endif //ORB_SLAM2_ROS_MAPGRAPHPUBLISHER_H
