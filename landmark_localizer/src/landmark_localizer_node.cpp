/*
 * node.cpp
 *
 * Created on: 02 02, 2023
 * Author: yuying jin
 */

#include "monocular_pose_estimator.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "landmark_localizer");

  monocular_pose_estimator::MPENode mpe_node;

  ros::spin();

  return 0;
}
