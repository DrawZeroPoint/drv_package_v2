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
  ObstacleDetect(bool use_od, string base_frame, float base_to_ground, 
                 float table_height, float table_area);
  ObstacleDetect(bool use_od, string base_frame, float base_to_ground, 
                 float table_height, float table_area, float grasp_area_x, float grasp_area_y);
  /**
   * @brief ObstacleDetect::detectTableInCloud
   * Find out whether the source cloud contains table,
   * if true, publish table geometry for obstacle avoiding.
   * The msg type is geometry_msgs/PoseStamped, which
   * contains pose of the table centroid, assuming the table is a box,
   * the size of table can be pre-defined
   */
  void detectObstacleTable();
  
  void detectPutTable(geometry_msgs::PoseStamped &put_pose, 
                      geometry_msgs::PoseStamped &ref_pose, bool &need_move);
  
  inline void setZOffset(float z_offset) {z_offset_ = z_offset;}
  
private:
  ros::NodeHandle nh;
  string param_running_mode_;
  
  // Whether perform obstacle detection
  bool use_od_;
  
  PointCloudMono::Ptr src_cloud_;
  // The transform object can't be shared between Classes
  Transform *m_tf_;
  // Frame for point cloud to transfer
  string base_frame_;
  
  ros::Subscriber sub_pointcloud_;
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  
  ros::Publisher pub_table_pose_;
  ros::Publisher pub_table_points_;
  
  // Target table approximate height
  float table_height_;
  
  // height max error 
  float th_height_;
  // Table area threshold
  float th_area_;
  
  // Distance between base_frame and ground
  float base_link_above_ground_;
  
  // Graspable area center xy in base_link frame
  float grasp_area_x_;
  float grasp_area_y_;
  // Distance between object center and base
  float z_offset_;
  
  float global_area_temp_;
  float global_height_temp_;
  
  vector<pcl::ModelCoefficients::Ptr> plane_coeff_;
  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_hull_;
  
  pcl::ModelCoefficients::Ptr plane_max_coeff_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_max_hull_;
  
  vector<float> planeZVector_;
  vector<float> planeCollectVector_;
  
  void findMaxPlane();
  void analyseObstacle();
  bool analysePutPose(geometry_msgs::PoseStamped &put_pose, 
                      geometry_msgs::PoseStamped &ref_pose);
  
  void projectCloud(pcl::ModelCoefficients::Ptr coeff_in, PointCloudMono::Ptr cloud_in, 
                    PointCloudMono::Ptr &cloud_out);
  
  float getCloudZMean(PointCloudMono::Ptr cloud_in);
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
   * @brief extractPlane extract points which have similar z value
   * @param z_in target z value
   * @param cloud_in source point cloud
   */
  void extractPlane(float z_in, PointCloudRGBN::Ptr cloud_in);
  
  
  template <typename PointTPtr>
  void publishCloud(PointTPtr cloud);
};

#endif // OBSTACLEDETECT_H
