#ifndef UTILITIES_H
#define UTILITIES_H

#include <ros/ros.h>

//STL
#include <string>
#include <vector>
#include <math.h>

//PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

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

#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudMono;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudRGBN;

class Utilities
{
public:
  Utilities();
  
  static void msgToCloud(const PointCloud::ConstPtr msg, 
                         PointCloudMono::Ptr cloud);
  
  static void estimateNormCurv(PointCloudMono::Ptr cloud_in, 
                               PointCloudRGBN::Ptr &cloud_out,
                               float norm_r, float grid_sz, bool down_sp);
  
  static void generateName(int count, string pref, string surf, string &name);
  
  static void getAverage(PointCloudMono::Ptr cloud_in, float &avr, float &deltaz);
  
  static void pointTypeTransfer(PointCloudRGBN::Ptr cloud_in, 
                                PointCloudMono::Ptr &cloud_out);
  
  static void clusterExtract(PointCloudMono::Ptr cloud_in, 
                             vector<pcl::PointIndices> &cluster_indices,
                             float th_cluster, int minsize, int maxsize);
  
  static void cutCloud(pcl::ModelCoefficients::Ptr coeff_in, double th_distance, 
                       PointCloudRGBN::Ptr cloud_in,
                       PointCloudMono::Ptr &cloud_out);
  
  static void rotateCloudXY(PointCloudRGBN::Ptr cloud_in, 
                            PointCloudRGBN::Ptr &cloud_out,
                            float rx, float ry, Eigen::Matrix4f &transform_inv);
  
  static void rotateBack(PointCloudMono::Ptr cloud_in, 
                         PointCloudMono::Ptr &cloud_out,
                         Eigen::Matrix4f transform_inv);
  
  static void preProcess(PointCloudMono::Ptr cloud_in, 
                         PointCloudMono::Ptr &cloud_out, float gird_sz);
  
  static void getCloudByNormZ(PointCloudRGBN::Ptr cloud_in, pcl::PointIndices::Ptr &inliers, 
                              float th_norm);
  
  static void getCloudByZ(PointCloudMono::Ptr cloud_in, pcl::PointIndices::Ptr &inliers, 
                          PointCloudMono::Ptr &cloud_out, float z_min, float z_max);
  
  static void getCloudByInliers(PointCloudMono::Ptr cloud_in, PointCloudMono::Ptr &cloud_out, 
                                pcl::PointIndices::Ptr inliers, bool negative, bool organized);
  static void getCloudByInliers(PointCloudRGBN::Ptr cloud_in, 
                                PointCloudRGBN::Ptr &cloud_out,
                                pcl::PointIndices::Ptr inliers, bool negative, bool organized);
  
  /**
   * @brief shrinkHull Shrink the 2D hull by distance dis according to the center
   * @param cloud Hull cloud in XY plane
   * @param cloud_sk Shrinked cloud in XY plane
   * @param dis in meter
   */
  static void shrinkHull(PointCloudMono::Ptr cloud, PointCloudMono::Ptr &cloud_sk, float dis);
  
  /**
   * @brief isInHull
   * Judge whether a given point p_in is in 2D hull, if so, return true, else return false
   * @param hull Hull cloud in XY plane
   * @param p_in Point to be judged in XY plane
   * @param p_dis Distance between p_in and its nearest point on hull
   * @return 
   */
  static bool isInHull(PointCloudMono::Ptr hull, pcl::PointXY p_in, 
                       pcl::PointXY &p_dis, pcl::PointXY &p_closest);
  
};

#endif // UTILITIES_H
