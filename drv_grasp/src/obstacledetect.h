#ifndef OBSTACLEDETECT_H
#define OBSTACLEDETECT_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_conversions/pcl_conversions.h>

//PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_cloud.h>

#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/mls.h>

#include <math.h>
#include <vector>
#include <string>

#include "utilities.h"

using namespace std;

class ObstacleDetect
{
public:
  ObstacleDetect(float table_height);
  
  void detectTableInCloud(PointCloudRGBN::Ptr &cloud_in);
  
  vector < pcl::ModelCoefficients::Ptr > plane_coeff;
  vector < pcl::PointCloud< pcl::PointXYZ >::Ptr> plane_hull;
  
private:
  float table_high_;
  vector <double> planeZVector_;
  vector <double> planeCollectVector_;
  
  
  void preProcess(PointCloudRGBN::Ptr cloud_in, PointCloudRGBN::Ptr cloud_out);
  void getAverage(PointCloudMono::Ptr cloud_in, double &avr, double &deltaz);
  void projectCloud(pcl::ModelCoefficients::Ptr coeff_in, PointCloudMono::Ptr cloud_in, 
                    PointCloudMono::Ptr &cloud_out);
  void extractPlane(double z_in, PointCloudRGBN::Ptr cloud_fit_plane, 
                    PointCloudMono::Ptr cloud_out_mono);
  
  void getCloudByInliers(PointCloudMono::Ptr cloud_in, PointCloudMono::Ptr &cloud_out, 
                         pcl::PointIndices::Ptr inliers, bool negative, bool organized);
  void getCloudByInliers(PointCloudRGBN::Ptr cloud_in, 
                         PointCloudRGBN::Ptr &cloud_out,
                         pcl::PointIndices::Ptr inliers, bool negative, bool organized);
  
  double filtCloudByZ(PointCloudMono::Ptr cloud_in);
  void processEachInliers(std::vector<pcl::PointIndices> indices_in, 
                          PointCloudRGBN::Ptr cloud_in);
  void getCloudByConditions(PointCloudRGBN::Ptr cloud_source, 
                            pcl::PointIndices::Ptr &inliers_plane, double thn);
  void calRegionGrowing(PointCloudRGBN::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
  void mergeCloud(PointCloudRGBN::Ptr cloud_fit_plane, PointCloudMono::Ptr cloud_out_mono);
};

#endif // OBSTACLEDETECT_H
