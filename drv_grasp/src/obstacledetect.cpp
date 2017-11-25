#include "obstacledetect.h"

//globe parameter

float axisLength_ = 0.1;
float br_ = 0.2;
float bg_ = 0.4;
float bb_ = 0.4;

// Normal threshold
float th_norm_ = 0.7;
float th_smooth_ = 8;

// Length threshold
float th_leaf_ = 0.01;
float th_noise_ = th_leaf_; // the measure error of sensor
float th_ec_ = 3 * th_leaf_;
// th_deltaz_ must 2 times bigger than th_leaf_
float th_deltaz_ = 2 * th_leaf_;
float th_ratio_ = 5 * th_leaf_; // flatness ratio max value of plane

float globalMaxZ, globalMinZ;
float globeMid;

ObstacleDetect::ObstacleDetect(float table_height, float table_area) :
  src_cloud_(new PointCloudMono)
{
  th_height_ = table_height;
  th_area_ = table_area;
  
  sub_pointcloud_ = nh.subscribe<sensor_msgs::PointCloud2>("depth_registered/points", 1, 
                                                           &ObstacleDetect::cloudCallback, this);
  
  pub_table_ = nh.advertise<geometry_msgs::TwistStamped>("/ctrl/vision/detect/table")
}

void ObstacleDetect::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  if (msg->data.empty()) {
    ROS_WARN_THROTTLE(31, "ObstacleDetect: PointCloud is empty.");
    return;
  }
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *src_cloud_);
}

/**
 * @brief ObstacleDetect::detectTableInCloud
 * Find out whether the source cloud contains table,
 * if ture, publish table geometry for obstacle avoiding.
 * The msg type is geometry_msgs/PolygonStamped, which
 * contains 2 Point32, the first is the centroid of the table,
 * the second is the size of the table (assuming a box)
 */
void ObstacleDetect::detectTableInCloud()
{
  // Calculate the normal of source cloud
  PointCloudRGBN::Ptr src_norm(new PointCloudRGBN);
  Utilities::estimateNormCurv(src_cloud_, src_norm, 0.02, 0.001, true);
  
  // Extact all points whose norm indicates that the point belongs to plane
  pcl::PointIndices::Ptr idx_near_plane(new pcl::PointIndices);
  getCloudByConditions(src_norm, idx_near_plane, th_norm_);
  
  if (idx_near_plane->indices.empty()) {
    ROS_INFO("ObstacleDetect: No point near plane.");
    return;
  }
  
  PointCloudRGBN::Ptr cloud_near_plane(new PointCloudRGBN);
  getCloudByInliers(src_norm, cloud_near_plane, idx_near_plane, false, false);
  
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
  //  Utilities::estimateNormCurv(cloud_msg, source_n);
  //  Utilities::rotateCloudXY(source_n, cloud_t, roll_, pitch_, transform_inv_);
  //  // std::cout << "trans_inv: "<< transform_inv_ << std::endl;
  
  //  PointCloudMono::Ptr plane_max (new PointCloudMono());
  //  PointCloudMono::Ptr ground_max (new PointCloudMono());
  //  pcl::ModelCoefficients::Ptr mcp (new pcl::ModelCoefficients);
  //  pcl::ModelCoefficients::Ptr mcg (new pcl::ModelCoefficients);
  
  //  processHullVector(FP.plane_hull, FP.plane_coeff, plane_max, mcp);
  
  //  if (!plane_max->empty() && (cloudPubPlaneMax_.getNumSubscribers() || posePubPlaneMax_.getNumSubscribers()))
  //  {
  //    sensor_msgs::PointCloud2 plane_max_msg;
  //    pcl::toROSMsg(*plane_max, plane_max_msg);
  //    plane_max_msg.header.frame_id = source_msg->header.frame_id;
  //    plane_max_msg.header.stamp = pcl_conversions::fromPCL(source_msg->header.stamp);
  
  //    geometry_msgs::PoseStamped pose_plane_max_msg;
  //    pose_plane_max_msg.header = plane_max_msg.header;
  //    pose_plane_max_msg.pose.orientation.x = mcp->values[0];
  //    pose_plane_max_msg.pose.orientation.y = mcp->values[1];
  //    pose_plane_max_msg.pose.orientation.z = mcp->values[2];
  //    pose_plane_max_msg.pose.orientation.w = mcp->values[3];
  
  //    cloudPubPlaneMax_.publish(plane_max_msg);
  //    posePubPlaneMax_.publish(pose_plane_max_msg);
  //  }
}

void ObstacleDetect::analyseHull()
{
  
}

void ObstacleDetect::getAverage(PointCloudMono::Ptr cloud_in, 
                                float &avr, float &deltaz)
{
  avr = 0.0;
  deltaz = 0.0;
  size_t sz = cloud_in->points.size();
  for (PointCloudMono::const_iterator pit = cloud_in->begin();
       pit != cloud_in->end(); ++pit)
    avr += pit->z;
  
  avr = avr / sz;
  
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cloud_in, minPt, maxPt);
  deltaz = maxPt.z - minPt.z;
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
  
  vector<pcl::PointIndices> cluster_indices;
  Utilities::clusterExtract(cloud_projected_surface, cluster_indices, th_ratio_, 2500, 307200);
  
  //cout << "plane cluster number:" << cluster_indices.size() << endl;
  
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
    /*pcl::ConcaveHull< pcl::PointXYZ > chull;
                                chull.setAlpha(0.5);
                                chull.setInputCloud(cloud_2d_divided);
                                chull.reconstruct(*cloud_hull);*/
    
    if (cloud_hull->points.size() > 3 && area_hull > th_area_) {
      ROS_INFO("Found plane with area %f", area_hull);
      plane_coeff_.push_back(coeff);
      plane_hull_.push_back(cloud_hull);
    }
  }
}

void ObstacleDetect::getCloudByInliers(PointCloudMono::Ptr cloud_in, 
                                       PointCloudMono::Ptr &cloud_out,
                                       pcl::PointIndices::Ptr inliers, 
                                       bool negative, bool organized)
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setNegative(negative);
  extract.setInputCloud(cloud_in);
  extract.setIndices(inliers);
  extract.setKeepOrganized(organized);
  extract.filter(*cloud_out);
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
    getAverage(cloud_temp, mid, zrange);
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
    
    getCloudByInliers(cloud_temp,cloud_local_temp, pointToRemove, true, false);
    cloud_temp = cloud_local_temp;
  }
  return mid;
}

void ObstacleDetect::getCloudByConditions(PointCloudRGBN::Ptr cloud_source, 
                                          pcl::PointIndices::Ptr &inliers_plane, float thn)
{
  for (int i = 0; i < cloud_source->points.size(); ++i)
  {
    float nz = cloud_source->points[i].normal_z;
    
    /// THRESHOLD
    //if point normal fulfill this criterion, consider it from plane
    if (nz > thn)
    {
      inliers_plane->indices.push_back(i);
    }
  }
}

void ObstacleDetect::getCloudByInliers(PointCloudRGBN::Ptr cloud_in, 
                                       PointCloudRGBN::Ptr &cloud_out,
                                       pcl::PointIndices::Ptr inliers, 
                                       bool negative, bool organized)
{
  pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
  extract.setNegative(negative);
  extract.setInputCloud(cloud_in);
  extract.setIndices(inliers);
  extract.setKeepOrganized(organized);
  extract.filter (*cloud_out);
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
