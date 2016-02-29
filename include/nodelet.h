#ifndef TRAJ_GEN_NODELET_H
#define TRAJ_GEN_NODELET_H


#include <nodelet/nodelet.h>
#include "TestTrajectory.h"


    class PipsTrajectoryNodelet : public nodelet::Nodelet
    {
        public:
            PipsTrajectoryNodelet();
            ~PipsTrajectoryNodelet();
            virtual void onInit();
        private:
            TestTrajectory* tester;
            
    };


#endif //TRAJ_GEN_NODELET_H
