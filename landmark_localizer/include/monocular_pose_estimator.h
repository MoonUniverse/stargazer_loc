// This file is part of RPG-MPE - the RPG Monocular Pose Estimator
//
// RPG-MPE is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RPG-MPE is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with RPG-MPE.  If not, see <http://www.gnu.org/licenses/>.
#ifndef MONOCULAR_POSE_ESTIMATOR_NODE_H_
#define MONOCULAR_POSE_ESTIMATOR_NODE_H_

#include "ros/ros.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// #include <opencv/cvwimage.h>
// #include <opencv/highgui.h>

#include <dynamic_reconfigure/server.h>
// #include <monocular_pose_estimator/MonocularPoseEstimatorConfig.h>

#include "pose_estimator.h"

namespace monocular_pose_estimator
{

class MPENode
{
private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher pose_pub_; //!< The ROS publisher that publishes the estimated pose.

  ros::Subscriber landmark_info_sub_; //!< The ROS subscriber to the raw landmark 
  ros::Subscriber camera_info_sub_; //!< The ROS subscriber to the camera info

  geometry_msgs::PoseWithCovarianceStamped predicted_pose_; //!< The ROS message variable for the estimated pose and covariance of the object

  bool have_camera_info_; //!< The boolean variable that indicates whether the camera calibration parameters have been obtained from the camera
  sensor_msgs::CameraInfo cam_info_; //!< Variable to store the camera calibration parameters

  PoseEstimator trackable_object_; //!< Declaration of the object whose pose will be estimated

  std::string landmark_info_topic_;
  std::string camera_info_topic_;

public:

  MPENode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  MPENode() : MPENode( ros::NodeHandle(), ros::NodeHandle("~") ){}
  ~MPENode();

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void landmarkCallback(const landmark_localizer::LandmarkArray::ConstPtr& image_msg);

};

} // monocular_pose_estimator namespace

#endif 
