#include <pips_trajectory_testing/stand_alone_collision_checker.h>
#include <pips_trajectory_testing/depth_image_cc_wrapper.h>

int main(int argc, char** argv)
{
  pips_trajectory_testing::StandAloneCollisionChecker<pips_trajectory_testing::DepthImageCCWrapper>(argc, argv);
}
