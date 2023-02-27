#include "LandmarkFinderInterface.h"

#include "StargazerConversionMethods.h"
#include "ros_utils.h"

using namespace stargazer_ros_tool;

LandmarkFinderInterface::LandmarkFinderInterface(ros::NodeHandle nh_public,
                                                 ros::NodeHandle nh_private)
        : img_trans{nh_public}, server{nh_private},have_camera_info_(false) {

    params_.fromNodeHandle(nh_private);
    landmarkFinder = std::make_unique<stargazer::LandmarkFinder>(params_.stargazer_config);
    server.setCallback(boost::bind(&LandmarkFinderInterface::reconfigureCallback, this, _1, _2));
    lm_pub = nh_private.advertise<landmark_finder::LandmarkArray>(params_.landmark_topic, 1);
    img_sub = img_trans.subscribe(
        params_.undistorted_image_topic, 1, &LandmarkFinderInterface::imgCallback, this);
    camera_info_sub = nh_private.subscribe(
        params_.camera_info_topic, 1, &LandmarkFinderInterface::camerainfoCallback, this);    
    debugVisualizer_.SetWaitTime(10);

    if (params_.cfg.debug_mode)
        showNodeInfo();
}

void LandmarkFinderInterface::camerainfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
  if (!have_camera_info_)
  {
    cam_info_ = *msg;

    // Calibrated camera
    camera_matrix_K_ = cv::Mat(3, 3, CV_64F);
    camera_matrix_K_.at<double>(0, 0) = cam_info_.K[0];
    camera_matrix_K_.at<double>(0, 1) = cam_info_.K[1];
    camera_matrix_K_.at<double>(0, 2) = cam_info_.K[2];
    camera_matrix_K_.at<double>(1, 0) = cam_info_.K[3];
    camera_matrix_K_.at<double>(1, 1) = cam_info_.K[4];
    camera_matrix_K_.at<double>(1, 2) = cam_info_.K[5];
    camera_matrix_K_.at<double>(2, 0) = cam_info_.K[6];
    camera_matrix_K_.at<double>(2, 1) = cam_info_.K[7];
    camera_matrix_K_.at<double>(2, 2) = cam_info_.K[8];
    camera_distortion_coeffs_ = (cv::Mat_<double>(5, 1) << cam_info_.D[0],cam_info_.D[1],
                                                              cam_info_.D[2],cam_info_.D[3],cam_info_.D[5]);
    have_camera_info_ = true;
    ROS_INFO("Camera calibration information obtained.");
  }
}

void LandmarkFinderInterface::imgCallback(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cvPtr;
    if (msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = msg->header;
        img.height = msg->height;
        img.width = msg->width;
        img.is_bigendian = msg->is_bigendian;
        img.step = msg->step;
        img.data = msg->data;
        img.encoding = "mono8";
        cvPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }else
    {
        cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }

    std::vector<stargazer::ImgLandmark> detected_img_landmarks;
    landmarkFinder->DetectLandmarks(cvPtr->image, detected_img_landmarks);

    // Convert
    landmark_finder::LandmarkArray landmarksMessage =
        convert2LandmarkMsg(detected_img_landmarks, msg->header);
    lm_pub.publish(landmarksMessage);

    //  Visualize
    if (!params_.cfg.debug_mode)
        return;

    // Invert images
    // cv::bitwise_not(landmarkFinder->grayImage_, landmarkFinder->grayImage_);
    // cv::bitwise_not(landmarkFinder->filteredImage_, landmarkFinder->filteredImage_);
    // cv::bitwise_not(landmarkFinder->binaryImage_, landmarkFinder->binaryImage_);
    // Show images
    // debugVisualizer_.ShowImage(landmarkFinder->grayImage_, "Gray Image");
    // debugVisualizer_.ShowImage(landmarkFinder->filteredImage_, "Filtered Image");
    debugVisualizer_.ShowImage(landmarkFinder->binaryImage_, "Binary Image");

    // // Show detections
    // auto point_img = debugVisualizer_.ShowPoints(landmarkFinder->filteredImage_,
    //                                              landmarkFinder->clusteredPixels_);
    auto cluster_img = debugVisualizer_.ShowClusters(landmarkFinder->filteredImage_,
                                                     landmarkFinder->clusteredPoints_);

    // Show landmarks
    cv::Mat temp;
    cvtColor(landmarkFinder->grayImage_, temp, CV_GRAY2BGR);
    debugVisualizer_.DrawLandmarks(temp, detected_img_landmarks);
    debugVisualizer_.ShowImage(temp, "Detected Landmarks");

    if(!have_camera_info_){
      return;
    }

    if(detected_img_landmarks.size() == 0) {
      return;
    }
    // cal pose
    stargazer::ImgLandmark minId_landmarks;
    if (detected_img_landmarks.size() == 1) {
      minId_landmarks = detected_img_landmarks[0];
    } else {
      unsigned int min_nID = detected_img_landmarks[0].nID;
      minId_landmarks = detected_img_landmarks[0];
      for (int i = 1; i < detected_img_landmarks.size(); i++) {
        if (min_nID > detected_img_landmarks[i].nID) {
          min_nID = detected_img_landmarks[i].nID;
          minId_landmarks = detected_img_landmarks[i];
        }
      }
    }

    std::vector<cv::Point2f> landmark_point_array;
    cv::Point2f landmark_point;
    for (int i = 0; i < 3; i++) {
      landmark_point.x = minId_landmarks.voCorners[i].x;
      landmark_point.y = minId_landmarks.voCorners[i].y;
      landmark_point_array.push_back(landmark_point);
    }
    for (int i = 0; i < minId_landmarks.voIDPoints.size(); i++) {
      landmark_point.x = minId_landmarks.voIDPoints[i].x;
      landmark_point.y = minId_landmarks.voIDPoints[i].y;
      landmark_point_array.push_back(landmark_point);
    }
    // std::cout << "landmark_point_array:" << landmark_point_array << std::endl;
    std::vector<stargazer::Point> world_point_array;
    std::vector<cv::Point3d> world_point;
    world_point_array = stargazer::getLandmarkPoints(minId_landmarks.nID);

    for (int i = 0; i < world_point_array.size(); i++) {
      cv::Point3d temp_point;
      temp_point.x = world_point_array[i][0];
      temp_point.y = world_point_array[i][1];
      temp_point.z = 0;
      world_point.push_back(temp_point);
    }
    // std::cout << "world_point" << world_point << std::endl;
    cv::Mat raux, taux;
    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
    cv::solvePnP(world_point, landmark_point_array, camera_matrix_K_, camera_distortion_coeffs_, raux, taux ,false, cv::SOLVEPNP_ITERATIVE);


    raux.convertTo(Rvec, CV_32F);    
    taux.convertTo(Tvec, CV_32F);   

    cv::Mat_<float> rotMat(3, 3);
    cv::Rodrigues(Rvec, rotMat);  //由于solvePnP返回的是旋转向量，故用罗德里格斯变换变成旋转矩阵


    //格式转换
    Eigen::Matrix3d R_n;
    Eigen::Matrix3d T_n;
    cv::cv2eigen(rotMat, R_n);
    cv::cv2eigen(Tvec, T_n);

    Eigen::Quaterniond eigen_quat(R_n);
    // tf::Quaternion tf_quat;
    // tf::quaternionEigenToTF(eigen_quat, tf_quat);
    
    // std::cout << "R_n" << R_n << std::endl;
    // std::cout << "T_n" << T_n << std::endl;
    broadcaster.sendTransform(tf::StampedTransform(
        tf::Transform(tf::Quaternion(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w()),
                      tf::Vector3(T_n(0), T_n(1), T_n(2))),
        ros::Time::now(), "ir_camera_link", "landmark_link"));

}

void LandmarkFinderInterface::reconfigureCallback(LandmarkFinderConfig& config,
                                                  const uint32_t& level) {

    params_.fromConfig(config, level);

    landmarkFinder->tight_filter_size = static_cast<uint32_t>(params_.cfg.tight_filter_size);
    landmarkFinder->wide_filter_size = static_cast<uint32_t>(params_.cfg.wide_filter_size);
    landmarkFinder->threshold = static_cast<uint8_t>(params_.cfg.threshold);
    landmarkFinder->maxRadiusForCluster = params_.cfg.maxRadiusForCluster;
    landmarkFinder->maxRadiusForPixelCluster = params_.cfg.maxRadiusForPixelCluster;
    landmarkFinder->maxPixelForCluster = static_cast<uint16_t>(params_.cfg.maxPixelForCluster);
    landmarkFinder->minPixelForCluster = static_cast<uint16_t>(params_.cfg.minPixelForCluster);
    landmarkFinder->maxPointsPerLandmark = static_cast<uint16_t>(params_.cfg.maxPointsPerLandmark);
    landmarkFinder->minPointsPerLandmark = static_cast<uint16_t>(params_.cfg.minPointsPerLandmark);
}
