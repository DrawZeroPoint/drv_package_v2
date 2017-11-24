#include "obstacledetect.h"

//globe parameter

double axisLength_ = 0.1;
double br_ = 0.2;
double bg_ = 0.4;
double bb_ = 0.4;

//normal threshold
double th_norm_ = 0.7;
double th_smooth_ = 8;

//length threshold
double th_leaf_ = 0.01;
double th_noise_ = th_leaf_;// the measure error of sensor
double th_ec_ = 3 * th_leaf_;
double th_deltaz_ = 2 * th_leaf_;
double th_ratio_ = 5 * th_leaf_;// flatness ratio max value of plane

//area threshold
double th_area_ = 0.1;

double globalMaxZ, globalMinZ;
double globeMid;

ObstacleDetect::ObstacleDetect(float table_height)
{
  table_high_ = table_height;
  
  sub_pointcloud_ = nh.subscribe<PointCloud>("depth_registered/points", 1, 
                                             &ObstacleDetect::cloudCallback, this);
}

void ObstacleDetect::cloudCallback(const PointCloud::ConstPtr &cloud_msg)
{
  if (source_msg->empty())
  {
    ROS_ERROR("Can't get source cloud message.\n");
    fType_ = t_null;
    return;
  }
  PointCloudWN::Ptr source_n (new  PointCloudWN());
  PointCloudWN::Ptr cloud_t (new PointCloudWN ());
  
  Utilities::estimateNorCurv(source_msg, source_n);
  Utilities::rotateCloudXY(source_n, cloud_t, roll_, pitch_, transform_inv_);
  // std::cout << "trans_inv: "<< transform_inv_ << std::endl;
  
  FindPlane FP;
  FP.getParameters(GH);
  FP.findPlaneInCloud(cloud_t);
  
  PointCloudMono::Ptr plane_max (new PointCloudMono());
  PointCloudMono::Ptr ground_max (new PointCloudMono());
  pcl::ModelCoefficients::Ptr mcp (new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr mcg (new pcl::ModelCoefficients);
  
  processHullVector(FP.plane_hull, FP.plane_coeff, plane_max, mcp);
  processHullVector(FP.ground_hull, FP.ground_coeff, ground_max, mcg);
  
  if (!plane_max->empty() && (cloudPubPlaneMax_.getNumSubscribers() || posePubPlaneMax_.getNumSubscribers()))
  {
    sensor_msgs::PointCloud2 plane_max_msg;
    pcl::toROSMsg(*plane_max, plane_max_msg);
    plane_max_msg.header.frame_id = source_msg->header.frame_id;
    plane_max_msg.header.stamp = pcl_conversions::fromPCL(source_msg->header.stamp);
    
    geometry_msgs::PoseStamped pose_plane_max_msg;
    pose_plane_max_msg.header = plane_max_msg.header;
    pose_plane_max_msg.pose.orientation.x = mcp->values[0];
    pose_plane_max_msg.pose.orientation.y = mcp->values[1];
    pose_plane_max_msg.pose.orientation.z = mcp->values[2];
    pose_plane_max_msg.pose.orientation.w = mcp->values[3];
    
    cloudPubPlaneMax_.publish(plane_max_msg);
    posePubPlaneMax_.publish(pose_plane_max_msg);
  }
  
  if (!ground_max->empty() && (cloudPubGround_.getNumSubscribers() || posePubGround_.getNumSubscribers()))
  {
    sensor_msgs::PointCloud2 ground_max_msg;
    pcl::toROSMsg(*ground_max, ground_max_msg);
    ground_max_msg.header.frame_id = source_msg->header.frame_id;
    ground_max_msg.header.stamp = pcl_conversions::fromPCL(source_msg->header.stamp);
    
    geometry_msgs::PoseStamped pose_ground_msg;
    pose_ground_msg.header = ground_max_msg.header;
    pose_ground_msg.pose.orientation.x = mcg->values[0];
    pose_ground_msg.pose.orientation.y = mcg->values[1];
    pose_ground_msg.pose.orientation.z = mcg->values[2];
    pose_ground_msg.pose.orientation.w = mcg->values[3];
    
    cloudPubGround_.publish(ground_max_msg);
    posePubGround_.publish(pose_ground_msg);
  }
  
  if (!plane_max->empty() && !ground_max->empty())
  {
    fType_ = t_both;
  }
  else if (plane_max->empty() && !ground_max->empty())
  {
    fType_ = t_ground;
  }
  else if (!plane_max->empty() && ground_max->empty())
  {
    fType_ = t_table;
  }
  else
  {
    fType_ = t_null;
  }
}


void ObstacleDetect::preProcess(PointCloudRGBN::Ptr cloud_in, PointCloudRGBN::Ptr cloud_out)
{
  pcl::VoxelGrid<pcl::PointXYZRGBNormal> vg;
  vg.setInputCloud (cloud_in);
  vg.setLeafSize (0.05, 0.05, 0.05);
  vg.filter (*cloud_out);
}

void ObstacleDetect::getAverage(PointCloudMono::Ptr cloud_in, double &avr, double &deltaz)
{
  avr = 0.0;
  deltaz = 0.0;
  size_t sz = cloud_in->points.size();
  for (PointCloudMono::const_iterator pit = cloud_in->begin();pit != cloud_in->end(); ++pit)
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

void ObstacleDetect::extractPlane(double z_in, PointCloudRGBN::Ptr cloud_fit_plane, 
                                  PointCloudMono::Ptr cloud_out_mono)
{
  pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients() );
  coeff->values.push_back(0.0);
  coeff->values.push_back(0.0);
  coeff->values.push_back(1.0);
  coeff->values.push_back(-z_in);
  
  // use original cloud as input, get projected cloud to next cluster step
  PointCloudMono::Ptr cloud_projected_surface (new PointCloudMono());
  Utilities::cutCloud(coeff, th_deltaz_, cloud_fit_plane, cloud_projected_surface);//th_deltaz_ must 2 times bigger than th_leaf_
  
  std::vector<pcl::PointIndices> cluster_indices;
  Utilities::ecExtraction(cloud_projected_surface, cluster_indices, th_ratio_, 2500, 307200);
  
  //cout << "plane cluster number:" << cluster_indices.size() << endl;
  
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    PointCloudMono::Ptr cloud_2d_divided (new PointCloudMono);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_2d_divided->points.push_back (cloud_projected_surface->points[*pit]);
    
    cloud_2d_divided->width = cloud_2d_divided->points.size ();
    cloud_2d_divided->height = 1;
    cloud_2d_divided->is_dense = true;
    
    PointCloudMono::Ptr cloud_hull (new PointCloudMono);
    pcl::ConvexHull< pcl::PointXYZ > chull;
    chull.setInputCloud(cloud_2d_divided);
    chull.setComputeAreaVolume(true);
    chull.reconstruct(*cloud_hull);
    double area_hull = chull.getTotalArea();
    /*pcl::ConcaveHull< pcl::PointXYZ > chull;
                                chull.setAlpha(0.5);
                                chull.setInputCloud(cloud_2d_divided);
                                chull.reconstruct(*cloud_hull);*/
    
    if (cloud_hull->points.size() > 3 && area_hull > th_area_)
    {
      cerr << "Plane area is " << area_hull << ", consider to be support surface." << endl;
      plane_coeff.push_back(coeff);
      plane_hull.push_back(cloud_hull);
    }
    else
    {
      cerr << "Plane area is " << area_hull << ", smaller than " << th_area_ << ", ignored." << endl;
    }
  }
}


void ObstacleDetect::getCloudByInliers(PointCloudMono::Ptr cloud_in, PointCloudMono::Ptr &cloud_out,
                                       pcl::PointIndices::Ptr inliers, bool negative, bool organized)
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setNegative(negative);
  extract.setInputCloud(cloud_in);
  extract.setIndices(inliers);
  extract.setKeepOrganized(organized);
  extract.filter(*cloud_out);
}

double ObstacleDetect::filtCloudByZ(PointCloudMono::Ptr cloud_in)
{
  // remove the far point from mid each loop
  PointCloudMono::Ptr cloud_local_temp (new PointCloudMono ());
  PointCloudMono::Ptr cloud_temp (new PointCloudMono ());
  cloud_temp = cloud_in;
  
  double mid, zrange;
  while (cloud_temp->points.size() > 2)
  {
    double dis_high = 0.0;
    double dis_low = 0.0;
    int max_high = -1;
    int max_low = -1;
    pcl::PointIndices::Ptr pointToRemove (new pcl::PointIndices);
    getAverage(cloud_temp, mid, zrange);
    if (zrange <= th_deltaz_)
      break;
    
    // remove both upper and bottom points
    size_t ct = 0;
    for (PointCloudMono::const_iterator pit = cloud_temp->begin();pit != cloud_temp->end();++pit)
    {
      double dis = pit->z - mid;
      if (dis - dis_high >= th_leaf_)
      {
        dis_high = dis;
        max_high = ct;
      }
      if (dis - dis_low <= - th_leaf_)
      {
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
    
    getCloudByInliers(cloud_temp,cloud_local_temp,pointToRemove,true,false);
    cloud_temp = cloud_local_temp;
  }
  return mid;
}

void ObstacleDetect::processEachInliers(std::vector<pcl::PointIndices> indices_in, 
                                        PointCloudRGBN::Ptr cloud_in)
{
  if (indices_in.empty())
  {
    cerr << "processEachInliers: Region growing get nothing.\n" << endl;
  }
  else
  {
    size_t k = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = indices_in.begin (); it != indices_in.end (); ++it)
    {
      PointCloudRGBN::Ptr cloud_fit_part (new PointCloudRGBN ());
      int count = 0;
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        cloud_fit_part->points.push_back (cloud_in->points[*pit]);
        count++;
      }
      // reassemble the cloud part
      cloud_fit_part->width = cloud_fit_part->points.size ();
      cloud_fit_part->height = 1;
      cloud_fit_part->is_dense = true;
      
      cerr << "processEachInliers: Number of points in cluster " << k << " :" << count << endl;
      
      //search those part which may come from same plane
      PointCloudMono::Ptr cloud_fit_part_t (new PointCloudMono ());
      Utilities::pointTypeTransfer(cloud_fit_part,cloud_fit_part_t);
      
      double z_value_of_plane_part = filtCloudByZ(cloud_fit_part_t);
      planeZVector_.push_back(z_value_of_plane_part);
      k++;
    }
    
    if (planeZVector_.empty())
    {
      cerr << "processEachInliers: No cloud part pass the z judgment." << endl;
      return;
    }
    
    sort(planeZVector_.begin(),planeZVector_.end());
  }
}


void ObstacleDetect::getCloudByConditions(PointCloudRGBN::Ptr cloud_source, 
                                          pcl::PointIndices::Ptr &inliers_plane, double thn)
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
                                       pcl::PointIndices::Ptr inliers, bool negative, bool organized)
{
  pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
  extract.setNegative (negative);
  extract.setInputCloud (cloud_in);
  extract.setIndices (inliers);
  extract.setKeepOrganized(organized);
  extract.filter (*cloud_out);
}


void ObstacleDetect::calRegionGrowing(PointCloudRGBN::Ptr cloud, 
                                      pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  pcl::RegionGrowing<pcl::PointXYZRGBNormal, pcl::Normal> reg;
  pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr tree = 
      boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBNormal> > (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  
  // this affect both ground and table detection, small value make detect ground easier
  reg.setMinClusterSize(1000);
  reg.setMaxClusterSize(307200);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(20);
  reg.setInputCloud(cloud);
  //reg.setIndices (indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(th_smooth_ / 180.0 * M_PI);
  //reg.setCurvatureThreshold (0);
  
  vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  
  /// region grow tire the whole cloud apart, process each part to see if they come from the same plane
  processEachInliers(clusters, cloud);
}

// merge cloud which from the same plane
void ObstacleDetect::mergeCloud(PointCloudRGBN::Ptr cloud_fit_plane, 
                                PointCloudMono::Ptr cloud_out_mono)
{
  cerr << "Probability plane number: " << planeZVector_.size() << endl;
  for (vector <double>::iterator cit = planeZVector_.begin(); cit != planeZVector_.end(); cit++)
  {
    extractPlane(*cit, cloud_fit_plane, cloud_out_mono);
  }
}

void ObstacleDetect::detectTableInCloud(PointCloudRGBN::Ptr &cloud_in)
{
  PointCloudRGBN::Ptr cloud_out (new  PointCloudRGBN());
  PointCloudMono::Ptr cloud_out_mono (new  PointCloudMono());
  PointCloudRGBN::Ptr cloud_fit_plane (new  PointCloudRGBN());
  //    preProcess(cloud_in, cloud_out);
  cloud_out = cloud_in;
  
  pcl::PointIndices::Ptr indices_plane_n(new pcl::PointIndices);
  getCloudByConditions(cloud_out, indices_plane_n, th_norm_);
  if (indices_plane_n->indices.empty())
  {
    cerr << "ObstacleDetect: No cloud satisfy normal condition.\n" << endl;
    return;
  }
  getCloudByInliers(cloud_out, cloud_fit_plane, indices_plane_n, false, false);// here cloud_fit_plane get normals
  
  cerr << "Points may from plane: " << cloud_fit_plane->points.size() << endl;
  
  pcl::PointCloud<pcl::Normal>::Ptr normals_plane_curv (new pcl::PointCloud<pcl::Normal> ());
  normals_plane_curv->resize(cloud_fit_plane->size());
  
  for (size_t i = 0; i < cloud_fit_plane->points.size(); i++)
  {
    normals_plane_curv->points[i].normal_x = cloud_fit_plane->points[i].normal_x;
    normals_plane_curv->points[i].normal_y = cloud_fit_plane->points[i].normal_y;
    normals_plane_curv->points[i].normal_z = cloud_fit_plane->points[i].normal_z;
    // std::cerr << normals_plane_curv->points[i].normal_x << std::endl;
  }
  calRegionGrowing(cloud_fit_plane, normals_plane_curv);
  
  Utilities::pointTypeTransfer(cloud_out, cloud_out_mono);
  mergeCloud(cloud_fit_plane, cloud_out_mono);
}
