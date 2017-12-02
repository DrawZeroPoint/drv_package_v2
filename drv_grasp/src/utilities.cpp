#include "utilities.h"

using namespace std;

Utilities::Utilities()
{
}

void Utilities::generateName(int count, string pref, string surf, string &name)
{
  std::ostringstream ost;
  ost << count;
  std::string temp(ost.str());
  name = pref + temp + surf;
}

void Utilities::msgToCloud(const PointCloud::ConstPtr msg,
                           PointCloudMono::Ptr cloud)
{
  cloud->height = msg->height;
  cloud->width  = msg->width;
  cloud->is_dense = false;
  cloud->resize(cloud->height * cloud->width);
  
  size_t i = 0;
  for (PointCloud::const_iterator pit = msg->begin(); 
       pit != msg->end(); ++pit) {
    cloud->points[i].x = pit->x;
    cloud->points[i].y = pit->y;
    cloud->points[i].z = pit->z;
    ++i;
  }
}

void Utilities::estimateNormCurv(PointCloudMono::Ptr cloud_in, 
                                 PointCloudRGBN::Ptr &cloud_out,
                                 float norm_r, float grid_sz, bool down_sp)
{
  PointCloudMono::Ptr cloud_fit(new PointCloudMono);
  if (down_sp)
    preProcess(cloud_in, cloud_fit, grid_sz);
  else
    cloud_fit = cloud_in;
  
  cloud_out->height = cloud_fit->height;
  cloud_out->width  = cloud_fit->width;
  cloud_out->is_dense = false;
  cloud_out->resize(cloud_out->height * cloud_out->width);
  
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud_fit);
  
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset 
  // (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(norm_r); // mm
  
  // Compute the features
  pcl::PointCloud<pcl::Normal>::Ptr cloud_norm(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*cloud_norm);
  
  for (size_t i = 0; i < cloud_out->size(); ++i) {
    cloud_out->points[i].x = cloud_fit->points[i].x;
    cloud_out->points[i].y = cloud_fit->points[i].y;
    cloud_out->points[i].z = cloud_fit->points[i].z;
    cloud_out->points[i].r = 1;
    cloud_out->points[i].g = 1;
    cloud_out->points[i].b = 1;
    cloud_out->points[i].normal_x = cloud_norm->points[i].normal_x;
    cloud_out->points[i].normal_y = cloud_norm->points[i].normal_y;
    cloud_out->points[i].normal_z = cloud_norm->points[i].normal_z;
  }
}

void Utilities::preProcess(PointCloudMono::Ptr cloud_in, 
                           PointCloudMono::Ptr &cloud_out,
                           float gird_sz)
{
  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud_in);
  vg.setLeafSize(gird_sz, gird_sz, gird_sz);
  vg.filter(*cloud_out);
}

void Utilities::pointTypeTransfer(PointCloudRGBN::Ptr cloud_in, 
                                  PointCloudMono::Ptr &cloud_out)
{
  cloud_out->resize(cloud_in->size());
  
  for (size_t i = 0; i < cloud_in->points.size(); i++) {
    cloud_out->points[i].x = cloud_in->points[i].x;
    cloud_out->points[i].y = cloud_in->points[i].y;
    cloud_out->points[i].z = cloud_in->points[i].z;
  }
}

void Utilities::cutCloud(pcl::ModelCoefficients::Ptr coeff_in, double th_distance,
                         PointCloudRGBN::Ptr cloud_in, 
                         PointCloudMono::Ptr &cloud_out)
{
  vector<int> inliers_cut;
  Eigen::Vector4f coeffs(coeff_in->values[0], coeff_in->values[1],
      coeff_in->values[2], coeff_in->values[3]);
  
  PointCloudMono::Ptr cloudSourceFiltered_t(new PointCloudMono);
  pointTypeTransfer(cloud_in, cloudSourceFiltered_t);
  pcl::SampleConsensusModelPlane<pcl::PointXYZ> scmp(cloudSourceFiltered_t);
  scmp.selectWithinDistance(coeffs, th_distance, inliers_cut);
  scmp.projectPoints(inliers_cut, coeffs, *cloud_out, false);
}

void Utilities::clusterExtract(PointCloudMono::Ptr cloud_in, 
                               vector<pcl::PointIndices> &cluster_indices,
                               float th_cluster, int minsize, int maxsize)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(th_cluster);
  ec.setMinClusterSize(minsize); // should be small, let area judge
  ec.setMaxClusterSize(maxsize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_in);
  ec.extract(cluster_indices);
}

void Utilities::rotateCloudXY(PointCloudRGBN::Ptr cloud_in, PointCloudRGBN::Ptr &cloud_out,
                              float rx, float ry, Eigen::Matrix4f &transform_inv)
{
  Eigen::Matrix4f transform_x = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f transform_y = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f transform_ = Eigen::Matrix4f::Identity();
  
  //the math function cos etc. operate angle in radius
  transform_x(1,1) = cos(rx);
  transform_x(2,1) = -sin(rx);
  transform_x(1,2) = sin(rx);
  transform_x(2,2) = cos(rx);
  //		std::cout << "trans x: "<< transform_x << std::endl;
  
  transform_y(0,0) = cos(ry);
  transform_y(0,2) = -sin(ry);
  transform_y(2,0) = sin(ry);
  transform_y(2,2) = cos(ry);
  //		std::cout << "trans y: "<< transform_y << std::endl;
  
  transform_ = transform_y * transform_x;
  //		std::cout << "total trans: "<< transform_ << std::endl;
  transform_inv = transform_.inverse();
  //		std::cout << "trans_inv: "<< transform_inv << std::endl;
  
  // Executing the transformation
  pcl::transformPointCloudWithNormals(*cloud_in, *cloud_out, transform_);
}

void Utilities::rotateBack(PointCloudMono::Ptr cloud_in, PointCloudMono::Ptr &cloud_out,
                           Eigen::Matrix4f transform_inv)
{
  // Executing the transformation
  pcl::transformPointCloud(*cloud_in, *cloud_out, transform_inv);
}

void Utilities::getAverage(PointCloudMono::Ptr cloud_in, 
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

void Utilities::getCloudByNormZ(PointCloudRGBN::Ptr cloud_in, 
                                pcl::PointIndices::Ptr &inliers, 
                                float th_norm)
{
  size_t i = 0;
  for (PointCloudRGBN::const_iterator pit = cloud_in->begin();
       pit != cloud_in->end();++pit) {
    float n_z = pit->normal_z;
    // If point normal fulfill this criterion, consider it from plane
    // Here we use absolute value cause
    if (fabs(n_z) > th_norm)
      inliers->indices.push_back(i);
    ++i;
  }
}

void Utilities::getCloudByZ(PointCloudMono::Ptr cloud_in, 
                            pcl::PointIndices::Ptr &inliers, 
                            PointCloudMono::Ptr &cloud_out, 
                            float z_min, float z_max)
{
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_max);
  pass.setIndices(inliers);
  //pass.setFilterLimitsNegative (true);
  pass.filter(*cloud_out);
  
}

void Utilities::getCloudByInliers(PointCloudMono::Ptr cloud_in, 
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

void Utilities::getCloudByInliers(PointCloudRGBN::Ptr cloud_in, 
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

void Utilities::shrinkHull(PointCloudMono::Ptr cloud, 
                           PointCloudMono::Ptr &cloud_sk, float dis)
{
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cloud, minPt, maxPt);
  
  size_t i = 0;
  float center_x = (maxPt.x + minPt.x) / 2;
  float center_y = (maxPt.y + minPt.y) / 2;
  for (PointCloudMono::const_iterator pit = cloud->begin(); 
       pit != cloud->end(); ++pit) {
    if (pit->x == center_x) {
      if (pit->y > center_y) {
        cloud_sk->points[i].y = (pit->y - dis)>center_y?(pit->y - dis):pit->y;
      }
      else {
        cloud_sk->points[i].y = (pit->y + dis)<center_y?(pit->y + dis):pit->y;
      }
      cloud_sk->points[i].x = pit->x;
    }
    else {
      float d_x = pit->x - center_x;
      float d_y = pit->y - center_y;
      float theta = atan(d_y/d_x);
      if (d_x > 0 && d_y >= 0) {
        cloud_sk->points[i].x = (pit->x - fabs(dis*sin(theta)))>center_x?
                                (pit->x - fabs(dis*sin(theta))):pit->x;
        cloud_sk->points[i].y = (pit->y - fabs(dis*cos(theta)))>center_y?
                                (pit->y - fabs(dis*cos(theta))):pit->y;
      }
      else if (d_x < 0 && d_y >= 0) {
        cloud_sk->points[i].x = (pit->x + fabs(dis*sin(theta)))<center_x?
                                (pit->x + fabs(dis*sin(theta))):pit->x;
        cloud_sk->points[i].y = (pit->y - fabs(dis*cos(theta)))>center_y?
                                (pit->y - fabs(dis*cos(theta))):pit->y;
      }
      else if (d_x < 0 && d_y < 0) {
        cloud_sk->points[i].x = (pit->x + fabs(dis*sin(theta)))<center_x?
                                (pit->x + fabs(dis*sin(theta))):pit->x;
        cloud_sk->points[i].y = (pit->y + fabs(dis*cos(theta)))<center_y?
                                (pit->y + fabs(dis*cos(theta))):pit->y;
      }
      else {
        cloud_sk->points[i].x = (pit->x - fabs(dis*sin(theta)))>center_x?
                                (pit->x - fabs(dis*sin(theta))):pit->x;
        cloud_sk->points[i].y = (pit->y + fabs(dis*cos(theta)))<center_y?
                                (pit->y + fabs(dis*cos(theta))):pit->y;
      }
    }
  }
}

bool Utilities::isInHull(PointCloudMono::Ptr hull, pcl::PointXY p_in, 
                         pcl::PointXY &p_dis, pcl::PointXY &p_closest)
{
  float x_temp = 1.0;
  float y_temp = 1.0;
  float dis_temp = 30.0;
  
  size_t i = 0;
  for (PointCloudMono::const_iterator pit = hull->begin(); 
       pit != hull->end(); ++pit) {
    float delta_x = pit->x - p_in.x;
    x_temp *= delta_x;
    float delta_y = pit->y - p_in.y;
    y_temp *= delta_y;
    
    // Use Manhattan distance
    float dis = fabs(delta_x) + fabs(delta_y);
    if (dis < dis_temp) {
      p_dis.x = delta_x;
      p_dis.y = delta_y;
      p_closest.x = pit->x;
      p_closest.y = pit->y;
      dis_temp = dis;
    }
    ++i;
  }
  if (x_temp <= 0 && y_temp <= 0) {
    // The p_in is surrounded by hull
    p_dis.x = 0;
    p_dis.y = 0;
    return true;
  }
  else {
    return false;
  }
}

bool Utilities::tryExpandROI(int &minx, int &miny, int &maxx, int &maxy, 
                             int width, int height, int pad)
{
  if (minx >= maxx || miny >= maxy) {
    return false;
  }
  minx -= pad;
  maxx += pad;
  miny -= pad;
  maxy += pad;
  if (minx < 0) minx = 0;
  if (maxx > width) maxx = width - 1;
  if (miny < 0) miny = 0;
  if (maxy > height) maxy = height - 1;
}
