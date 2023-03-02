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
    depth_img_sub = img_trans.subscribe(
        params_.depth_image_topic, 1, &LandmarkFinderInterface::depthimgCallback, this);
    camera_info_sub = nh_private.subscribe(
        params_.camera_info_topic, 1, &LandmarkFinderInterface::camerainfoCallback, this); 

    laser_filtered_point_pub = nh_private.advertise<sensor_msgs::PointCloud2>("laser_filtered_point", 1);
    ref_filtered_point_pub = nh_private.advertise<sensor_msgs::PointCloud2>("ref_filtered_point", 1);
    compute_point_pub = nh_private.advertise<sensor_msgs::PointCloud2>("icp_filtered_point", 1);
    // See the implementation of setDefault() to create a custom ICP algorithm
    // icp.setDefault();
    std::ifstream icp_file("/home/tmirobot/Documents/github/stargazer_loc/src/landmark_finder/launch/icp.yaml");
    icp.loadFromYaml(icp_file);

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
    std::cout <<"camera_matrix_K_: " << camera_matrix_K_ << std::endl;

    std::cout <<"camera_distortion_coeffs_: " << camera_distortion_coeffs_ << std::endl;

    ROS_INFO("Camera calibration information obtained.");
  }
}

void LandmarkFinderInterface::depthimgCallback(const sensor_msgs::ImageConstPtr& msg) {
    depthcvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
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
    debugVisualizer_.ShowImage(depthcvPtr->image, "depth Image");

    debugVisualizer_.ShowImage(landmarkFinder->binaryImage_, "Binary Image");

    // // Show detections
    // auto point_img = debugVisualizer_.ShowPoints(landmarkFinder->filteredImage_,
    //                                              landmarkFinder->clusteredPixels_);
    auto cluster_img = debugVisualizer_.ShowClusters(landmarkFinder->filteredImage_,
                                                     landmarkFinder->clusteredPoints_);
    debugVisualizer_.ShowImage(depthcvPtr->image, "depth Image");
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
  Eigen::Matrix3d R_n;
  Eigen::Matrix3d T_n;
  Eigen::Matrix4d R_T_n;
// cv solve pnp
#if 1
  std::vector<cv::Point2f> landmark_point_array_pnp;
  cv::Point2f landmark_point_pnp;
  for (int i = 0; i < 3; i++) {
    landmark_point_pnp.x = minId_landmarks.voCorners[i].x;
    landmark_point_pnp.y = minId_landmarks.voCorners[i].y;
    landmark_point_array_pnp.push_back(landmark_point_pnp);
  }
  for (int i = 0; i < minId_landmarks.voIDPoints.size(); i++) {
    landmark_point_pnp.x = minId_landmarks.voIDPoints[i].x;
    landmark_point_pnp.y = minId_landmarks.voIDPoints[i].y;
    landmark_point_array_pnp.push_back(landmark_point_pnp);
  }
  std::cout << "landmark_point_array_pnp:" << landmark_point_array_pnp << std::endl;
  std::vector<stargazer::Point> world_point_array_pnp;
  std::vector<cv::Point3d> world_point;
  world_point_array_pnp = stargazer::getLandmarkPoints(minId_landmarks.nID);

  for (int i = 0; i < world_point_array_pnp.size(); i++) {
    cv::Point3d temp_point;
    temp_point.x = world_point_array_pnp[i][0];
    temp_point.y = world_point_array_pnp[i][1];
    temp_point.z = 0;
    world_point.push_back(temp_point);
  }
  std::cout << "world_point" << world_point << std::endl;
  cv::Mat raux, taux;
  cv::Mat Rvec;
  cv::Mat_<float> Tvec;
  cv::solvePnP(world_point, landmark_point_array_pnp, camera_matrix_K_, camera_distortion_coeffs_, raux, taux ,false, cv::SOLVEPNP_ITERATIVE);


  raux.convertTo(Rvec, CV_32F);    
  taux.convertTo(Tvec, CV_32F);   

  cv::Mat_<float> rotMat(3, 3);
  cv::Rodrigues(Rvec, rotMat);  //由于solvePnP返回的是旋转向量，故用罗德里格斯变换变成旋转矩阵


  //格式转换

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
#endif

// icp
#if 1
    std::vector<cv::Point> landmark_point_in_camera;
    cv::Point landmark_point;

    for (int i = 0; i < 3; i++) {
      landmark_point.x = minId_landmarks.voCorners[i].x;
      landmark_point.y = minId_landmarks.voCorners[i].y;
      // for(const auto& iter : landmarkFinder->clusteredPoints_map[landmark_point]) {
      //   landmark_point_in_camera.push_back(iter);
      // }
      landmark_point_in_camera.push_back(landmark_point);
    }
    for (int i = 0; i < minId_landmarks.voIDPoints.size(); i++) {
      landmark_point.x = minId_landmarks.voIDPoints[i].x;
      landmark_point.y = minId_landmarks.voIDPoints[i].y;
      // for(const auto& iter : landmarkFinder->clusteredPoints_map) {
      //   double distance = hypot((iter.first.x - landmark_point.x) , (iter.first.y - landmark_point.y));
      //   if(distance < 3){
      //     for(const auto& iter : landmarkFinder->clusteredPoints_map[iter.first]) {
      //       landmark_point_in_camera.push_back(iter);
      //     }
      //   }
      // }
      landmark_point_in_camera.push_back(landmark_point);
    }   
    sensor_msgs::PointCloud2 filtered_point;
    std::vector<unsigned> tmp_data; 
    for(const auto& iter : landmark_point_in_camera) {
      uint16_t depth = depthcvPtr->image.at<uint16_t>(iter.y, iter.x);
      float_union cloud_point_x, cloud_point_y, cloud_point_z, cloud_intensity;      
      // if(depth == 0) continue;
      cloud_point_z.fv = (float)depth / 1000;
      cloud_point_x.fv = (iter.x - camera_matrix_K_.at<double>(0, 2)) 
                            * cloud_point_z.fv / camera_matrix_K_.at<double>(0, 0);
      cloud_point_y.fv = (iter.y - camera_matrix_K_.at<double>(1, 2)) 
                            * cloud_point_z.fv / camera_matrix_K_.at<double>(1, 1);
      cloud_intensity.fv = 0;

      for (int j = 0; j < 4; j++) {
          tmp_data.push_back(cloud_point_x.cv[j]);
      }
      for (int j = 0; j < 4; j++) {
          tmp_data.push_back(cloud_point_y.cv[j]);
      }
      for (int j = 0; j < 4; j++) {
          tmp_data.push_back(cloud_point_z.cv[j]);
      }
      for (int j = 0; j < 4; j++) {
          tmp_data.push_back(cloud_intensity.cv[j]);
      }

    }

    const int numChannels = 4;
    // debug
    filtered_point.header.frame_id = msg->header.frame_id;
    filtered_point.header.stamp = msg->header.stamp;
    filtered_point.height = 1;
    filtered_point.width = landmark_point_in_camera.size();

    filtered_point.is_bigendian = false;
    filtered_point.is_dense = true;
    filtered_point.point_step = numChannels * sizeof(float);
    filtered_point.row_step = filtered_point.point_step * filtered_point.width;

    filtered_point.fields.resize(numChannels);
    for (int i = 0; i < numChannels; i++) {
        std::string channelId[] = {"x", "y", "z", "intensity"};
        filtered_point.fields[i].name = channelId[i];
        filtered_point.fields[i].offset = i * sizeof(float);
        filtered_point.fields[i].count = 1;
        filtered_point.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }
    filtered_point.data.resize(filtered_point.row_step * filtered_point.height);

    for (int i = 0; i < filtered_point.row_step; i++) {
        filtered_point.data[i] = tmp_data[i];
    }
    tmp_data.clear();
    laser_filtered_point_pub.publish(filtered_point);


    std::vector<stargazer::Point> world_point_array;
    std::vector<stargazer::Point> intensity_world_point_array;
    world_point_array = stargazer::getLandmarkPoints(minId_landmarks.nID);
    
    for(int i = 0; i < world_point_array.size(); i++){
      stargazer::Point intensity_point;
      intensity_point[0] = world_point_array[i][0];
      intensity_point[1] = world_point_array[i][1];
      intensity_point[2] = 0;
      intensity_world_point_array.push_back(intensity_point);
      // for(int j = 0; j < 20; j ++){
      //   stargazer::Point intensity_point;
      //   intensity_point[0] = world_point_array[i][0] + 0.005 * cosf(2*M_PI/20*j);
      //   intensity_point[1] = world_point_array[i][1] + 0.005 * sinf(2*M_PI/20*j);
      //   intensity_point[2] = 0;
      //   intensity_world_point_array.push_back(intensity_point);

      // }
      // for(int j = 0; j < 40; j ++){
      //   stargazer::Point intensity_point;
      //   intensity_point[0] = world_point_array[i][0] + 0.01 * cosf(2*M_PI/40*j);
      //   intensity_point[1] = world_point_array[i][1] + 0.01 * sinf(2*M_PI/40*j);
      //   intensity_point[2] = 0;
      //   intensity_world_point_array.push_back(intensity_point);

      // }

    }

    sensor_msgs::PointCloud2 ref_point;
    std::vector<unsigned> ref_tmp_data; 
    for(int i = 0; i < intensity_world_point_array.size(); i++) {
      float_union cloud_point_x, cloud_point_y, cloud_point_z, cloud_intensity;      
      cloud_point_z.fv = 0;
      cloud_point_x.fv = intensity_world_point_array[i][0];
      cloud_point_y.fv = intensity_world_point_array[i][1];
      cloud_intensity.fv = 0;

      for (int j = 0; j < 4; j++) {
          ref_tmp_data.push_back(cloud_point_x.cv[j]);
      }
      for (int j = 0; j < 4; j++) {
          ref_tmp_data.push_back(cloud_point_y.cv[j]);
      }
      for (int j = 0; j < 4; j++) {
          ref_tmp_data.push_back(cloud_point_z.cv[j]);
      }
      for (int j = 0; j < 4; j++) {
          ref_tmp_data.push_back(cloud_intensity.cv[j]);
      }

    }

    // debug
    ref_point.header.frame_id = msg->header.frame_id;
    ref_point.header.stamp = msg->header.stamp;
    ref_point.height = 1;
    ref_point.width = intensity_world_point_array.size();

    ref_point.is_bigendian = false;
    ref_point.is_dense = true;
    ref_point.point_step = numChannels * sizeof(float);
    ref_point.row_step = ref_point.point_step * ref_point.width;

    ref_point.fields.resize(numChannels);
    for (int i = 0; i < numChannels; i++) {
        std::string channelId[] = {"x", "y", "z", "intensity"};
        ref_point.fields[i].name = channelId[i];
        ref_point.fields[i].offset = i * sizeof(float);
        ref_point.fields[i].count = 1;
        ref_point.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }
    ref_point.data.resize(ref_point.row_step * ref_point.height);

    for (int i = 0; i < ref_point.row_step; i++) {
        ref_point.data[i] = ref_tmp_data[i];
    }
    ref_tmp_data.clear();
    ref_filtered_point_pub.publish(ref_point);


    DP ref_cloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(ref_point);//ref_point
    DP input_cloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(filtered_point);//filtered_point

    PM::TransformationParameters T;

    T = icp(input_cloud, ref_cloud);

    std::cout << "T:" << std::endl << T << std::endl;

    DP data_out(input_cloud);
    icp.transformations.apply(data_out, T);

    sensor_msgs::PointCloud2 compute_point;
    
    compute_point = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(data_out, "ir_camera_link", ros::Time::now());

    compute_point_pub.publish(compute_point);


    // DP ref_cloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(ref_point);//ref_point
    // DP input_cloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(filtered_point);//filtered_point



    // PM::TransformationParameters T;
    // PM::TransformationParameters iterateTransfo;
    // iterateTransfo = PM::TransformationParameters::Identity(4, 4);

    // std::shared_ptr<PM::Transformation> rigidTrans;
    // rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");


    //格式转换
    // Eigen::Matrix3d R_n;
    // Eigen::Matrix3d T_n;
    // cv::cv2eigen(rotMat, R_n);
    // cv::cv2eigen(Tvec, T_n);


    // iterateTransfo(0,0) = R_n(0,0);
    // iterateTransfo(0,1) = R_n(0,1);
    // iterateTransfo(0,2) = R_n(0,2);
    // iterateTransfo(0,3) = T_n(0,0);

    // iterateTransfo(1,0) = R_n(1,0);
    // iterateTransfo(1,1) = R_n(1,1);
    // iterateTransfo(1,2) = R_n(1,2);
    // iterateTransfo(1,3) = T_n(1,0);

    // iterateTransfo(2,0) = R_n(2,0);
    // iterateTransfo(2,1) = R_n(2,1);
    // iterateTransfo(2,2) = R_n(2,2);
    // iterateTransfo(2,3) = T_n(2,0);

    // iterateTransfo(3,0) = 0;
    // iterateTransfo(3,1) = 0;
    // iterateTransfo(3,2) = 0;
    // iterateTransfo(3,3) = 1;

    // const DP initializedData = rigidTrans->compute(input_cloud, iterateTransfo);
    // T = icp(initializedData, ref_cloud);

    // std::cout << "T:" << std::endl << T << std::endl;

    // DP data_out(initializedData);
    // icp.transformations.apply(data_out, T);

    // sensor_msgs::PointCloud2 compute_point;
    
    // compute_point = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(data_out, "ir_camera_link", ros::Time::now());

    // compute_point_pub.publish(compute_point);

    landmarkFinder->clusteredPoints_map.clear();

#endif 



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
