#include <tf2/LinearMath/Quaternion.h>
#include "obstacledetect.h"


// Normal threshold
float th_norm_ = 0.7;
float th_smooth_ = 8;

// Length threshold
float th_leaf_ = 0.01;
// th_deltaz_ must 2 times bigger than th_leaf_
float th_deltaz_ = 2 * th_leaf_;
float th_ratio_ = 5 * th_leaf_; // flatness ratio max value of plane

ObstacleDetect::ObstacleDetect(string map_frame, float table_height, float table_area) :
  src_cloud_(new PointCloudMono),
  m_tf_(new Transform),
  map_frame_(map_frame)
{
  param_running_mode_ = "/status/running_mode";
  
  th_height_ = table_height;
  th_area_ = table_area;
  
  sub_pointcloud_ = nh.subscribe<sensor_msgs::PointCloud2>("depth_registered/points", 1, 
                                                           &ObstacleDetect::cloudCallback, this);
  
  pub_table_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/ctrl/vision/detect/table", 1);
  pub_table_points_ = nh.advertise<sensor_msgs::PointCloud2>("table/points", 1);
}

void ObstacleDetect::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  if (ros::param::has(param_running_mode_)) {
    int mode_type;
    ros::param::get(param_running_mode_, mode_type);
    // 2 for tracking
    if (mode_type == 2) {
      if (msg->data.empty()) {
        ROS_WARN_THROTTLE(31, "ObstacleDetect: PointCloud is empty.");
        return;
      }
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(*msg, pcl_pc2);
      PointCloudMono::Ptr temp(new PointCloudMono);
      pcl::fromPCLPointCloud2(pcl_pc2, *temp);
      m_tf_->getTransform(map_frame_, msg->header.frame_id);
      m_tf_->doTransform(temp, src_cloud_);
    }
  }
}

void ObstacleDetect::detectTableInCloud()
{
  // Calculate the normal of source cloud
  PointCloudRGBN::Ptr src_norm(new PointCloudRGBN);
  Utilities::estimateNormCurv(src_cloud_, src_norm, 0.02, 0.001, true);
  
  // Extact all points whose norm indicates that the point belongs to plane
  pcl::PointIndices::Ptr idx_near_plane(new pcl::PointIndices);
  Utilities::getCloudByNormZ(src_norm, idx_near_plane, th_norm_);
  
  if (idx_near_plane->indices.empty()) {
    ROS_INFO("ObstacleDetect: No point near plane.");
    return;
  }
  
  PointCloudRGBN::Ptr cloud_near_plane(new PointCloudRGBN);
  Utilities::getCloudByInliers(src_norm, cloud_near_plane, idx_near_plane, false, false);
  
  ROS_DEBUG("Points may from plane: %d", cloud_near_plane->points.size());
  
  // Prepare curv data for clustering
  pcl::PointCloud<pcl::Normal>::Ptr norm_plane_curv(new pcl::PointCloud<pcl::Normal>);
  norm_plane_curv->resize(cloud_near_plane->size());
  
  size_t i = 0;
  for (PointCloudRGBN::const_iterator pit = cloud_near_plane->begin();
       pit != cloud_near_plane->end(); ++pit) {
    norm_plane_curv->points[i].normal_x = pit->normal_x;
    norm_plane_curv->points[i].normal_y = pit->normal_y;
    norm_plane_curv->points[i].normal_z = pit->normal_z;
    ++i;
  }
  // Perform clustering, cause the scene may contain multiple planes
  calRegionGrowing(cloud_near_plane, norm_plane_curv);
  
  // Extact each plane from the points having similiar z value,
  // the planes are stored in vector plane_hull_
  extractPlaneForEachZ(cloud_near_plane);
  
  // Get table geometry from hull and publish the plane with max area
  analyseHull();
}

void ObstacleDetect::analyseHull()
{
  if (max_hull_id_ >= 0 && max_hull_id_ < plane_hull_.size()) {
    PointCloudMono::Ptr cloud = plane_hull_[max_hull_id_];
    
    // For now, we only publish the largest plane
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    
    geometry_msgs::PoseStamped table_pose;
    table_pose.header.frame_id = map_frame_;
    table_pose.header.stamp = ros::Time(0);
    table_pose.pose.position.x = (maxPt.x - minPt.x)/2;
    table_pose.pose.position.y = (maxPt.y - minPt.y)/2;
    table_pose.pose.position.z = maxPt.z/2;

    vector<pcl::PointXYZ> vertex_points;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      if (cloud->points[i].x == maxPt.x) {
        vertex_points.push_back(cloud->points[i]);
      }
      else if (cloud->points[i].y == maxPt.y) {
        vertex_points.push_back(cloud->points[i]);
      }
      else if (cloud->points[i].x == minPt.x) {
        vertex_points.push_back(cloud->points[i]);
      }
      else if (cloud->points[i].y == minPt.y) {
        vertex_points.push_back(cloud->points[i]);
      }
    }
    // Try find right angle to calculate yaw of table
    float yaw = 0.0;
    for (size_t i = 0; i < vertex_points.size() - 2; ++i) {
      size_t id_s = i;
      size_t id_m = i + 1;
      size_t id_e = i + 2 < vertex_points.size()?(i+2):0;
      tf2::Vector3 ms(vertex_points[id_m].x - vertex_points[id_s].x,
                      vertex_points[id_m].y - vertex_points[id_s].y, 0);
      tf2::Vector3 em(vertex_points[id_e].x - vertex_points[id_m].x,
                      vertex_points[id_e].y - vertex_points[id_m].y, 0);
      float angle = tf2::tf2Angle(ms, em);
      if (fabs(angle - M_PI/2 < 0.1) || fabs(angle - 3*M_PI/2) < 0.1) {
        ROS_INFO("ObstacleDetect: Found right angle.");
        
        if (ms.length() > em.length()) {
          yaw = atan(ms.x()/ms.y());
        }
        else {
          yaw = atan(em.x()/em.y());
        }
        break;
      }
    }
    
    tf2::Quaternion q;
    q.setEuler(yaw, 0, 0);
    table_pose.pose.orientation.x = q.x();
    table_pose.pose.orientation.y = q.y();
    table_pose.pose.orientation.z = q.z();
    table_pose.pose.orientation.w = q.w();

    pub_table_pose_.publish(table_pose);
    
    sensor_msgs::PointCloud2 table_points;
    pcl::toROSMsg(*cloud, table_points);
    table_points.header = table_pose.header;
    pub_table_points_.publish(table_points);
  }
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
  
  // Since there may be multipe plane around z, we need do clustering
  vector<pcl::PointIndices> cluster_indices;
  Utilities::clusterExtract(cloud_projected_surface, cluster_indices, th_ratio_, 2500, 307200);
  
  ROS_DEBUG("Plane cluster number: %d", cluster_indices.size());
  
  size_t hull_id = 0;
  float area_temp = 0.0;
  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); 
       it != cluster_indices.end (); ++it) {
    PointCloudMono::Ptr cloud_near_z(new PointCloudMono);
    for (vector<int>::const_iterator pit = it->indices.begin(); 
         pit != it->indices.end(); ++pit)
      cloud_near_z->points.push_back(cloud_projected_surface->points[*pit]);
    
    cloud_near_z->width = cloud_near_z->points.size();
    cloud_near_z->height = 1;
    cloud_near_z->is_dense = true;
    
    PointCloudMono::Ptr cloud_hull(new PointCloudMono);
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud_near_z);
    hull.setComputeAreaVolume(true);
    hull.reconstruct(*cloud_hull);
    float area_hull = hull.getTotalArea();

    if (cloud_hull->points.size() > 3 && area_hull > th_area_) {
      // Select the nearest and largest plane to be the output
      if (area_hull > area_temp && fabs(z_in - th_height_) < 0.15) {
        max_hull_id_ = hull_id;
        area_temp = area_hull;
      }
      ROS_INFO("Found plane with area %f", area_hull);
      plane_coeff_.push_back(coeff);
      plane_hull_.push_back(cloud_hull);
    }
    ++hull_id;
  }
}

float ObstacleDetect::getCloudByZ(PointCloudMono::Ptr cloud_in)
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
  pcl::RegionGrowing<pcl::PointXYZRGBNormal, pcl::Normal> reg;
  pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBNormal> > 
      (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  
  reg.setMinClusterSize(1000);
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
    ROS_INFO("ObstacleDetect: Region growing get nothing.");
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
      
      cerr << "processEachInliers: Number of points in cluster " << k << " :" << count << endl;
      
      // Search those part which may come from the same plane
      PointCloudMono::Ptr cloud_fit_part_t(new PointCloudMono);
      Utilities::pointTypeTransfer(cloud_fit_part, cloud_fit_part_t);
      
      float z_value_of_plane_part = getCloudByZ(cloud_fit_part_t);
      planeZVector_.push_back(z_value_of_plane_part);
      k++;
    }
    
    if (planeZVector_.empty()) {
      cerr << "processEachInliers: No cloud part pass the z judgment." << endl;
    }
    else {
      cerr << "Probability plane number: " << planeZVector_.size() << endl;
      sort(planeZVector_.begin(), planeZVector_.end());
    }
  }
}
