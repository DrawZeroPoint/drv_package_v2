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
  std::vector <int> inliers_cut;
  Eigen::Vector4f coeffs(coeff_in->values[0],coeff_in->values[1],coeff_in->values[2],coeff_in->values[3]);
  
  PointCloudMono::Ptr cloudSourceFiltered_t (new PointCloudMono);
  pointTypeTransfer(cloud_in, cloudSourceFiltered_t);
  pcl::SampleConsensusModelPlane<pcl::PointXYZ> scmp(cloudSourceFiltered_t);
  scmp.selectWithinDistance (coeffs, th_distance, inliers_cut);
  scmp.projectPoints(inliers_cut, coeffs, *cloud_out, false);
}

void Utilities::ecExtraction(PointCloudMono::Ptr cloud_in, std::vector<pcl::PointIndices> &cluster_indices,
                             double th_cluster, int minsize, int maxsize)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (th_cluster);
  ec.setMinClusterSize (minsize);//should be small, let area judge
  ec.setMaxClusterSize (maxsize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_in);
  ec.extract (cluster_indices);
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

void Utilities::estimateNorCurv(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_in, 
                                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_out)
{
  PointCloudMono::Ptr cloud_mono (new PointCloudMono);
  cloud_mono->height = cloud_in->height;
  cloud_mono->width  = cloud_in->width;
  cloud_mono->is_dense = false;
  cloud_mono->resize(cloud_mono->height * cloud_mono->width);
  
  for (size_t i = 0; i < cloud_in->size(); ++i)
  {
    cloud_mono->points[i].x = cloud_in->points[i].x;
    cloud_mono->points[i].y = cloud_in->points[i].y;
    cloud_mono->points[i].z = cloud_in->points[i].z;
  }
  
  PointCloudMono::Ptr cloud_mono_fit (new PointCloudMono);
  preProcess(cloud_mono, cloud_mono_fit);
  
  cloud_out->height = cloud_mono_fit->height;
  cloud_out->width  = cloud_mono_fit->width;
  cloud_out->is_dense = false;
  cloud_out->resize(cloud_out->height * cloud_out->width);
  
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_mono_fit);
  
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.02);//in mm
  
  // Compute the features
  pcl::PointCloud<pcl::Normal >::Ptr cloud_nor(new pcl::PointCloud<pcl::Normal >());
  ne.compute (*cloud_nor);
  
  for (size_t i = 0; i < cloud_out->size(); ++i)
  {
    cloud_out->points[i].x = cloud_mono_fit->points[i].x;
    cloud_out->points[i].y = cloud_mono_fit->points[i].y;
    cloud_out->points[i].z = cloud_mono_fit->points[i].z;
    cloud_out->points[i].r = 1;
    cloud_out->points[i].g = 1;
    cloud_out->points[i].b = 1;
    cloud_out->points[i].normal_x = cloud_nor->points[i].normal_x;
    cloud_out->points[i].normal_y = cloud_nor->points[i].normal_y;
    cloud_out->points[i].normal_z = cloud_nor->points[i].normal_z;
  }
}

void Utilities::preProcess(PointCloudMono::Ptr cloud_in, PointCloudMono::Ptr cloud_out)
{
  cerr <<  endl;
  cerr << "-------- Get surface processing loop started --------"  << endl;
  
  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (cloud_in);
  vg.setLeafSize (0.011, 0.011, 0.011);
  vg.filter (*cloud_out);
}
