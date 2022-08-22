//
// Created by jjgomez on 30/8/21.
//

//
// Created by jjgomez on 30/8/21.
//

#include "MapGraphPublisher.h"
#include "HessianComputation.h"

#include <thread>

using namespace std;

namespace ORB_SLAM2 {
    MapGraphPublisher::MapGraphPublisher(Map* pMap, System* pSystem) {
        bRun_ = true;

        pMap_ = pMap;
        pSystem_ = pSystem;
    }

    void MapGraphPublisher::run() {
        while(bRun_){
            // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            computeFactorGraph(pMap_,50,pSystem_);
            // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}
