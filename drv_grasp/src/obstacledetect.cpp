#include <tf2/LinearMath/Quaternion.h>
#include "obstacledetect.h"


// Normal threshold, |z_norm_| > th_z_norm_ means the point is from plane
float th_z_norm_ = 0.7;

// Region growing threshold
float th_smooth_ = 8;

// Voxel grid threshold
float th_leaf_ = 0.015;
// th_deltaz_ must 2 times bigger than th_leaf_
float th_deltaz_ = 2 * th_leaf_;
float th_ratio_ = 5 * th_leaf_; // flatness ratio max value of plane

// Depth threshold
float th_max_depth_ = 1.3;

ObstacleDetect::ObstacleDetect(bool use_od, string base_frame, float base_to_ground, 
                               float table_height, float table_area) :
  use_od_(use_od),
  fi_(new FetchRGBD),
  pub_it_(nh_),
  src_cloud_(new PointCloudMono),
  src_z_inliers_(new pcl::PointIndices),
  m_tf_(new Transform),
  base_frame_(base_frame),
  base_link_above_ground_(base_to_ground),
  table_height_(table_height),
  th_height_(0.2),
  th_area_(table_area)
{
  param_running_mode_ = "/status/running_mode";
  
  // For store max hull id and area
  global_area_temp_ = 0;
  
  // Regist the callback if you want to use octomap
  // Obstacle info from point cloud
  sub_pointcloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/vision/depth_registered/points", 1, 
                                                            &ObstacleDetect::cloudCallback, this);

  // Detect table obstacle
  pub_table_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/ctrl/vision/detect/table", 1);
  pub_table_points_ = nh_.advertise<sensor_msgs::PointCloud2>("/vision/table/points", 1);
  
  // Detect obstacle by octomap from point cloud
  pub_exp_obj_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/vision/points/except_object", 1);
  
  // Detect obstacle by octomap from depth image
  pub_depth_cam_info_ = nh_.advertise<sensor_msgs::CameraInfo>("/vision/depth/camera_info", 3);
  pub_exp_obj_depth_ = pub_it_.advertise("/vision/depth/except_object", 1);
}

ObstacleDetect::ObstacleDetect(bool use_od, string base_frame, 
                               float base_to_ground, float table_height, float table_area,
                               float grasp_area_x, float grasp_area_y, float tolerance) :
  use_od_(use_od),
  fi_(new FetchRGBD),
  pub_it_(nh_),
  src_cloud_(new PointCloudMono),
  src_z_inliers_(new pcl::PointIndices), 
  m_tf_(new Transform),
  base_frame_(base_frame),
  base_link_above_ground_(base_to_ground),
  table_height_(table_height),
  th_height_(0.2),
  th_area_(table_area),
  grasp_area_x_(grasp_area_x),
  grasp_area_y_(grasp_area_y), 
  tolerance_(tolerance)
{
  param_running_mode_ = "/status/running_mode";
  
  // Store max hull id and area
  global_area_temp_ = 0;
  
  // Get table info from point cloud
  sub_pointcloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/vision/depth_registered/points", 1, 
                                                            &ObstacleDetect::cloudCallback, this);
  
  pub_table_points_ = nh_.advertise<sensor_msgs::PointCloud2>("/vision/table/points", 1);
  pub_exp_obj_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/vision/points_except_object", 1);
  pub_exp_obj_depth_ = pub_it_.advertise("/vision/depth/except_object", 1);
}

void ObstacleDetect::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  if (!use_od_)
    return;
  
  if (ros::param::has(param_running_mode_)) {
    int mode_type;
    ros::param::get(param_running_mode_, mode_type);
    // 2 for tracking, 3 for putting
    if (mode_type == 2 || mode_type == 3) {
      if (msg->data.empty()) {
        ROS_WARN_THROTTLE(31, "ObstacleDetect: PointCloud is empty.");
        return;
      }
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(*msg, pcl_pc2);
      PointCloudMono::Ptr temp(new PointCloudMono);
      pcl::fromPCLPointCloud2(pcl_pc2, *temp);
      
      PointCloudMono::Ptr temp_filtered(new PointCloudMono);
      Utilities::getCloudByZ(temp, src_z_inliers_, temp_filtered, 0.0, th_max_depth_);
      
      m_tf_->getTransform(base_frame_, msg->header.frame_id);
      m_tf_->doTransform(temp_filtered, src_cloud_);
    }
  }
}

bool ObstacleDetect::detectPutTable(geometry_msgs::PoseStamped &put_pose,
                                    geometry_msgs::PoseStamped &ref_pose, 
                                    bool &need_move)
{
  getSourceCloud();
  findMaxPlane();
  if (plane_max_hull_ == NULL) {
    ROS_INFO_THROTTLE(11, "ObstacleDetect: No put place detected.");
    return false;
  }
  else {
    need_move = analysePutPose(put_pose, ref_pose);
    return true;
  }
}

bool ObstacleDetect::getSourceCloud()
{
  while (ros::ok()) {
    if (!src_cloud_->points.empty())
      return true;
    
    // Handle callbacks and sleep for a small amount of time
    // before looping again
    ros::spinOnce();
    ros::Duration(0.005).sleep();
  }
}

void ObstacleDetect::detectObstacleTable()
{
  getSourceCloud();
  findMaxPlane();
  
  // Get table geometry from hull and publish the plane with max area
  analyseObstacle();
}

void ObstacleDetect::detectObstacleInCloud(int min_x, int min_y, 
                                           int max_x, int max_y)
{
  if (src_cloud_->points.empty())
    return;
  
  assert(src_cloud_->points.size() == src_z_inliers_->indices.size());
  
  PointCloudMono::Ptr cloud_except_obj(new PointCloudMono);
  pcl::PointIndices::Ptr idx_obj(new pcl::PointIndices);
  
  for (size_t i = 0; i < src_z_inliers_->indices.size(); ++i) {
    int c = src_z_inliers_->indices[i] % 640;
    int r = src_z_inliers_->indices[i] / 640;
    if (c > min_x && c < max_x && r > min_y && r < max_y)
      idx_obj->indices.push_back(i);
  }
  // Set negtive=true to get cloud id not equal to idx_obj
  Utilities::getCloudByInliers(src_cloud_, cloud_except_obj, idx_obj, 
                               true, false);
  publishCloud(cloud_except_obj, pub_exp_obj_cloud_);
}

void ObstacleDetect::detectObstacleInDepth(int min_x, int min_y, 
                                           int max_x, int max_y)
{
  cv_bridge::CvImagePtr rgb, depth;
  sensor_msgs::CameraInfo info;
  fi_->fetchRGBD(rgb, depth, info);
  Mat depth_except_obj = depth->image;
  for (size_t r = 0; r < depth_except_obj.rows; ++r) {
    for (size_t c = 0; c < depth_except_obj.cols; ++c) {
      if (c > min_x && c < max_x && r > min_y && r < max_y) {
        // The type of depth image is CV_32F
        depth_except_obj.at<float>(r, c) = 0.0;
      }
    }
  }
  cv_bridge::CvImage cv_img;
  cv_img.image = depth_except_obj;
  cv_img.header = info.header;
  cv_img.encoding = depth->encoding;
  
  pub_depth_cam_info_.publish(info);
  pub_exp_obj_depth_.publish(cv_img.toImageMsg());
}

void ObstacleDetect::findMaxPlane()
{
  if (src_cloud_->points.empty())
    return;
  
  // Clear temp
  planeZVector_.clear();
  plane_coeff_.clear();
  plane_hull_.clear();
  global_area_temp_ = 0.0;
  global_height_temp_ = 0.0;
  
  // Calculate the normal of source cloud
  PointCloudRGBN::Ptr src_norm(new PointCloudRGBN);
  Utilities::estimateNormCurv(src_cloud_, src_norm, 2*th_leaf_, th_leaf_, true);
  
  // Extract all points whose norm indicates that the point belongs to plane
  pcl::PointIndices::Ptr idx_norm_ok(new pcl::PointIndices);
  Utilities::getCloudByNormZ(src_norm, idx_norm_ok, th_z_norm_);
  
  if (idx_norm_ok->indices.empty()) {
    ROS_DEBUG("ObstacleDetect: No point have the right normal of plane.");
    return;
  }
  
  PointCloudRGBN::Ptr cloud_norm_ok(new PointCloudRGBN);
  Utilities::getCloudByInliers(src_norm, cloud_norm_ok, idx_norm_ok, false, false);
  
  ROS_DEBUG("Points may from plane: %d", cloud_norm_ok->points.size());
  
  // Prepare curv data for clustering
  pcl::PointCloud<pcl::Normal>::Ptr norm_plane_curv(new pcl::PointCloud<pcl::Normal>);
  norm_plane_curv->resize(cloud_norm_ok->size());
  
  size_t i = 0;
  for (PointCloudRGBN::const_iterator pit = cloud_norm_ok->begin();
       pit != cloud_norm_ok->end(); ++pit) {
    norm_plane_curv->points[i].normal_x = pit->normal_x;
    norm_plane_curv->points[i].normal_y = pit->normal_y;
    norm_plane_curv->points[i].normal_z = pit->normal_z;
    ++i;
  }
  // Perform clustering, cause the scene may contain multiple planes
  calRegionGrowing(cloud_norm_ok, norm_plane_curv);
  
  // Extract each plane from the points having similar z value,
  // the planes are stored in vector plane_hull_
  extractPlaneForEachZ(cloud_norm_ok);
}

template <typename PointTPtr>
void ObstacleDetect::publishCloud(PointTPtr cloud, ros::Publisher pub)
{
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*cloud, ros_cloud);
  ros_cloud.header.frame_id = base_frame_;
  ros_cloud.header.stamp = ros::Time(0);
  pub.publish(ros_cloud);
}

bool ObstacleDetect::analysePutPose(geometry_msgs::PoseStamped &put_pose,
                                    geometry_msgs::PoseStamped &ref_pose)
{
  PointCloudMono::Ptr cloud = plane_max_hull_;
  
  pcl::PointXY p;
  // Cause the camera view field is limited, we consider a hull is in reach
  // when the center point plus a toletance in x direction is in hull
  p.x = grasp_area_x_ + tolerance_;
  p.y = grasp_area_y_;
  
  put_pose.header.frame_id = base_frame_;
  put_pose.header.stamp = ros::Time(0);
  ref_pose.header.frame_id = base_frame_;
  ref_pose.header.stamp = ros::Time(0);
  
  PointCloudMono::Ptr cloud_sk(new PointCloudMono);
  //  Utilities::shrinkHull(cloud, cloud_sk, 0.06);
  publishCloud(cloud, pub_table_points_); // For reference
  
  pcl::PointXY p_dis;
  pcl::PointXY p_closest;
  if (Utilities::isInHull(cloud, p, p_dis, p_closest)) {
    put_pose.pose.position.x = grasp_area_x_;
    put_pose.pose.position.y = grasp_area_y_;
    // All points in cloud have same z, so we directly use the first point
    put_pose.pose.position.z = cloud->points[0].z + z_offset_;
    put_pose.pose.orientation.x = 0;
    put_pose.pose.orientation.y = 0;
    put_pose.pose.orientation.z = 0;
    put_pose.pose.orientation.w = 1;
    
    ROS_INFO_THROTTLE(11, "ObstacleDetect: Put pose detected.");
    return false; // Return value means whether need move
  }
  else {
    put_pose.pose.position.x = p_dis.x;
    put_pose.pose.position.y = p_dis.y;
    // All points in cloud have same z
    put_pose.pose.position.z = cloud->points[0].z + z_offset_;
    put_pose.pose.orientation.x = 0;
    put_pose.pose.orientation.y = 0;
    put_pose.pose.orientation.z = 0;
    put_pose.pose.orientation.w = 1;
    
    ref_pose.pose.position.x = p_closest.x;
    ref_pose.pose.position.y = p_closest.y;
    ref_pose.pose.position.z = cloud->points[0].z + z_offset_;
    ref_pose.pose.orientation.x = 0;
    ref_pose.pose.orientation.y = 0;
    ref_pose.pose.orientation.z = 0;
    ref_pose.pose.orientation.w = 1;
    ROS_INFO_THROTTLE(11, "ObstacleDetect: Need move to put object.");
    return true;
  }
}

void ObstacleDetect::analyseObstacle()
{
  PointCloudMono::Ptr cloud = plane_max_hull_;
  if (cloud == NULL) {
    ROS_WARN("ObstacleDetect: No table detected.");
    return;
  }
  
  // For now, we only publish the largest plane
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cloud, minPt, maxPt);
  
  // Calculate table centroid position
  geometry_msgs::PoseStamped table_pose;
  table_pose.header.frame_id = base_frame_;
  table_pose.header.stamp = ros::Time(0);
  table_pose.pose.position.x = (maxPt.x + minPt.x)/2;
  table_pose.pose.position.y = (maxPt.y + minPt.y)/2;
  table_pose.pose.position.z = (maxPt.z - base_link_above_ground_)/2;
  
  // Try find the longest line between neighbor points to be the edge
  float dis_temp = 0.0;
  float xs_, ys_, xe_, ye_;
  float mean_x = (maxPt.x + minPt.x)/2;
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    float xs = cloud->points[i].x;
    float ys = cloud->points[i].y;
    int ide = (i + 1) < cloud->points.size() ? (i + 1) : 0;
    float xe = cloud->points[ide].x;
    float ye = cloud->points[ide].y;
    // Only consider the far edge
    if (xs >= mean_x && xe >= mean_x) {
      float dis = pow(xe - xs, 2) + pow(ye - ys, 2);
      if (dis > dis_temp) {
        dis_temp = dis;
        xs_ = xs; ys_ = ys; xe_ = xe; ye_ = ye;
      }
    }
  }
  float yaw = 0.0;
  // atan2 is better than atan in 2 ways:
  // atan2 can handle the condition that the denominator=0
  // atan2 in range -Pi~PI while atan in -PI/2~PI/2
  // here the denominator plus -1 cause the coordinate of atan2 is
  // different with that of ROS (with rotation=PI/2)
  yaw = atan2(xe_ - xs_, ys_ - ye_);
  
  tf2::Quaternion q;
  q.setEuler(0, 0, yaw); // Notice the last angle is around Z axis
  table_pose.pose.orientation.x = q.x();
  table_pose.pose.orientation.y = q.y();
  table_pose.pose.orientation.z = q.z();
  table_pose.pose.orientation.w = q.w();
  
  pub_table_pose_.publish(table_pose);
  
  publishCloud(cloud, pub_table_points_);
  ROS_INFO_THROTTLE(11, "ObstacleDetect: Table detected.");
}

void ObstacleDetect::projectCloud(pcl::ModelCoefficients::Ptr coeff_in, 
                                  PointCloudMono::Ptr cloud_in, 
                                  PointCloudMono::Ptr &cloud_out)
{
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud_in);
  proj.setModelCoefficients(coeff_in);
  proj.filter(*cloud_out);
}

void ObstacleDetect::extractPlaneForEachZ(PointCloudRGBN::Ptr cloud_in)
{
  for (vector<float>::iterator cit = planeZVector_.begin(); 
       cit != planeZVector_.end(); cit++) {
    extractPlane(*cit, cloud_in);
  }
}

void ObstacleDetect::extractPlane(float z_in, PointCloudRGBN::Ptr cloud_in)
{
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
  // Plane function: ax + by + cz + d = 0, here coeff[3] = d = -cz
  coeff->values.push_back(0.0);
  coeff->values.push_back(0.0);
  coeff->values.push_back(1.0);
  coeff->values.push_back(-z_in);
  
  // Use original cloud as input, get projected cloud for clustering
  PointCloudMono::Ptr cloud_projected_surface(new PointCloudMono);
  Utilities::cutCloud(coeff, th_deltaz_, cloud_in, cloud_projected_surface);
  
  // Since there may be multiple plane around z, we need do clustering
  vector<pcl::PointIndices> cluster_indices;
  Utilities::clusterExtract(cloud_projected_surface, cluster_indices, th_ratio_, 0, 307200);
  
  ROS_DEBUG("Plane cluster number: %d", cluster_indices.size());
  
  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); 
       it != cluster_indices.end (); ++it) {
    PointCloudMono::Ptr cloud_near_z(new PointCloudMono);
    for (vector<int>::const_iterator pit = it->indices.begin(); 
         pit != it->indices.end(); ++pit)
      cloud_near_z->points.push_back(cloud_projected_surface->points[*pit]);
    
    cloud_near_z->width = cloud_near_z->points.size();
    cloud_near_z->height = 1;
    cloud_near_z->is_dense = true;
    
    if (cloud_near_z->points.size() < 3)
      continue;
    
    PointCloudMono::Ptr cloud_hull(new PointCloudMono);
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud_near_z);
    hull.setComputeAreaVolume(true);
    hull.reconstruct(*cloud_hull);
    float area_hull = hull.getTotalArea();
    
    if (cloud_hull->points.size() > 3 && area_hull > th_area_) {
      /* Select the plane which has similar th_height and
       * largest plane to be the output, notice that z_in is in base_link frame
       * th_height_ is the height of table, base_link is 0.4m above the ground */
      if (area_hull > global_area_temp_ && z_in > global_height_temp_ &&
          fabs(z_in + base_link_above_ground_ - table_height_) < th_height_) {
        plane_max_hull_ = cloud_hull;
        plane_max_coeff_ = coeff;
        // Update temp
        global_height_temp_ = z_in;
        global_area_temp_ = area_hull;
      }
      ROS_DEBUG("Found plane with area %f.", area_hull);
      plane_coeff_.push_back(coeff);
      plane_hull_.push_back(cloud_hull);
    }
  }
}

float ObstacleDetect::getCloudZMean(PointCloudMono::Ptr cloud_in)
{
  // Remove the fariest point in each loop
  PointCloudMono::Ptr cloud_local_temp (new PointCloudMono);
  PointCloudMono::Ptr cloud_temp (new PointCloudMono);
  cloud_temp = cloud_in;
  
  float mid, zrange;
  while (cloud_temp->points.size() > 2) {
    float dis_high = 0.0;
    float dis_low = 0.0;
    int max_high = -1;
    int max_low = -1;
    pcl::PointIndices::Ptr pointToRemove(new pcl::PointIndices);
    Utilities::getAverage(cloud_temp, mid, zrange);
    if (zrange <= th_deltaz_)
      break;
    
    // remove both upper and bottom points
    size_t ct = 0;
    for (PointCloudMono::const_iterator pit = cloud_temp->begin();
         pit != cloud_temp->end();++pit) {
      float dis = pit->z - mid;
      if (dis - dis_high >= th_leaf_) {
        dis_high = dis;
        max_high = ct;
      }
      if (dis - dis_low <= - th_leaf_) {
        dis_low = dis;
        max_low = ct;
      }
      ct++;
    }
    if (max_low < 0 && max_high < 0)
      break;
    if (max_high >= 0)
      pointToRemove->indices.push_back(max_high);
    if (max_low >= 0)
      pointToRemove->indices.push_back(max_low);
    
    Utilities::getCloudByInliers(cloud_temp, cloud_local_temp, 
                                 pointToRemove, true, false);
    cloud_temp = cloud_local_temp;
  }
  return mid;
}

void ObstacleDetect::calRegionGrowing(PointCloudRGBN::Ptr cloud, 
                                      pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  if (cloud->points.empty()) {
    ROS_DEBUG("ObstacleDetect: Norm cloud contains nothing.");
    return;
  }
  
  pcl::RegionGrowing<pcl::PointXYZRGBNormal, pcl::Normal> reg;
  pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBNormal> > 
      (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  
  reg.setMinClusterSize(30);
  reg.setMaxClusterSize(307200);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(20);
  reg.setInputCloud(cloud);
  //reg.setIndices (indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(th_smooth_ / 180.0 * M_PI);
  //reg.setCurvatureThreshold (0);
  
  vector<pcl::PointIndices> clusters;
  reg.extract(clusters);
  
  // Region grow tire the whole cloud apart, process each part 
  // to see if they come from the same plane
  getMeanZofEachCluster(clusters, cloud);
}

void ObstacleDetect::getMeanZofEachCluster(vector<pcl::PointIndices> indices_in, 
                                           PointCloudRGBN::Ptr cloud_in)
{
  if (indices_in.empty())
    ROS_DEBUG("ObstacleDetect: Region growing get nothing.");
  
  else {
    size_t k = 0;
    for (vector<pcl::PointIndices>::const_iterator it = indices_in.begin(); 
         it != indices_in.end(); ++it) {
      PointCloudRGBN::Ptr cloud_fit_part(new PointCloudRGBN);
      int count = 0;
      for (vector<int>::const_iterator pit = it->indices.begin(); 
           pit != it->indices.end(); ++pit) {
        cloud_fit_part->points.push_back(cloud_in->points[*pit]);
        count++;
      }
      // Reassemble the cloud part
      cloud_fit_part->width = cloud_fit_part->points.size ();
      cloud_fit_part->height = 1;
      cloud_fit_part->is_dense = true;
      
      // Search those part which may come from the same plane
      PointCloudMono::Ptr cloud_fit_part_t(new PointCloudMono);
      Utilities::pointTypeTransfer(cloud_fit_part, cloud_fit_part_t);
      
      float z_value_of_plane_part = getCloudZMean(cloud_fit_part_t);
      planeZVector_.push_back(z_value_of_plane_part);
      k++;
    }
    
    if (planeZVector_.empty()) {
      ROS_DEBUG("processEachInliers: No cloud part pass the z judgment.");
    }
    else {
      ROS_DEBUG("Probability plane number: %d", planeZVector_.size());
      // Small to large
      sort(planeZVector_.begin(), planeZVector_.end());
    }
  }
}
