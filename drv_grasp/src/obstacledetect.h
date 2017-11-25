#ifndef OBSTACLEDETECT_H
#define OBSTACLEDETECT_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>

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

#include "transform.h"
#include "utilities.h"

using namespace std;

class ObstacleDetect
{
public:
  ObstacleDetect(string map_frame, float table_height, float table_area);
  
  /**
   * @brief ObstacleDetect::detectTableInCloud
   * Find out whether the source cloud contains table,
   * if ture, publish table geometry for obstacle avoiding.
   * The msg type is geometry_msgs/PoseStamped, which
   * contains pose of the table centroid, assuming the table is a box,
   * the size of table can be pre-defined
   */
  void detectTableInCloud();
  
  PointCloudMono::Ptr src_cloud_;
  
  size_t max_hull_id_;
  vector<pcl::ModelCoefficients::Ptr> plane_coeff_;
  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_hull_;
  
private:
  ros::NodeHandle nh;
  
  string param_running_mode_;
  
  // The transform object can't be shared between Classes
  Transform *m_tf_;
  // Frame for pointcloud to trasfer
  string map_frame_;
  
  ros::Subscriber sub_pointcloud_;
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  
  ros::Publisher pub_table_pose_;
  ros::Publisher pub_table_points_;
  
  float th_height_;
  // Tabel area threshold
  float th_area_;
  
  vector<float> planeZVector_;
  vector<float> planeCollectVector_;
  
  void projectCloud(pcl::ModelCoefficients::Ptr coeff_in, PointCloudMono::Ptr cloud_in, 
                    PointCloudMono::Ptr &cloud_out);
  
  float getCloudByZ(PointCloudMono::Ptr cloud_in);
  void getMeanZofEachCluster(std::vector<pcl::PointIndices> indices_in, 
                          PointCloudRGBN::Ptr cloud_in);

  void calRegionGrowing(PointCloudRGBN::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
  
  /**
   * @brief extractPlaneForEachZ
   * Merge clouds which from the same plane with equal z value
   * @param cloud_in source cloud
   */
  void extractPlaneForEachZ(PointCloudRGBN::Ptr cloud_in);
  
  /**
   * @brief extractPlane extract points which have similiar z value
   * @param z_in target z value
   * @param cloud_in source point cloud
   */
  void extractPlane(float z_in, PointCloudRGBN::Ptr cloud_in);
  void analyseHull();
};

#endif // OBSTACLEDETECT_H
