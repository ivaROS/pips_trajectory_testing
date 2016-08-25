#ifndef PIPS_DEMO_NODELET_H
#define PIPS_DEMO_NODELET_H

#include "pips_demo.h"
#include <nodelet/nodelet.h>



    class PipsTrajectoryNodelet : public nodelet::Nodelet
    {
        public:
            PipsTrajectoryNodelet();
            ~PipsTrajectoryNodelet();
            virtual void onInit();
        private:
            TestTrajectory* tester;
            
    };


#endif //PIPS_DEMO_NODELET_H
