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



