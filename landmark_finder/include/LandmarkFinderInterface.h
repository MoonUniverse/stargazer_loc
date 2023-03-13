#ifndef LANDMARKFINDERINTERFACE_H_
#define LANDMARKFINDERINTERFACE_H_

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <DebugVisualizer.h>
#include <LandmarkFinder.h>
#include <landmark_finder/LandmarkFinderConfig.h>
#include <sensor_msgs/CameraInfo.h>
#include "LandmarkFinderInterfaceParameters.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
namespace stargazer_ros_tool {

typedef union {
    unsigned char cv[4];
    float fv;
} float_union;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

class LandmarkFinderInterface {

public:
    LandmarkFinderInterface(ros::NodeHandle = ros::NodeHandle(),
                            ros::NodeHandle = ros::NodeHandle("~"));
    PM::ICP icp;

private:
    void imgCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthimgCallback(const sensor_msgs::ImageConstPtr& msg);
    void reconfigureCallback(LandmarkFinderConfig& config, const uint32_t& level = 0);
    void camerainfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

    image_transport::Subscriber img_sub;
    image_transport::Subscriber depth_img_sub;
    image_transport::ImageTransport img_trans;
    ros::Publisher lm_pub;
    ros::Publisher laser_filtered_point_pub;
    ros::Publisher ref_filtered_point_pub;
    ros::Publisher compute_point_pub;
    ros::Subscriber camera_info_sub;
    dynamic_reconfigure::Server<LandmarkFinderConfig> server;
    LandmarkFinderInterfaceParameters params_;
    stargazer::DebugVisualizer debugVisualizer_;
    std::unique_ptr<stargazer::LandmarkFinder> landmarkFinder;
    bool have_camera_info_;

    cv::Mat camera_matrix_K_;
    cv::Mat camera_distortion_coeffs_;
    sensor_msgs::CameraInfo cam_info_; //!< Variable to store the camera calibration parameters

    tf::TransformBroadcaster broadcaster;

    cv_bridge::CvImagePtr depthcvPtr;

    std::string icp_file_;
};
}

#endif