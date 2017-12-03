#ifndef OBSTACLEDETECT_H
#define OBSTACLEDETECT_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

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
using namespace cv;

class ObstacleDetect
{
public:
  ObstacleDetect(bool use_od, string base_frame, float base_to_ground, 
                 float table_height, float table_area);
  ObstacleDetect(bool use_od, string base_frame, float base_to_ground, float table_height,
                 float table_area, float grasp_area_x, float grasp_area_y, float tolerance);
  /**
   * @brief ObstacleDetect::detectTableInCloud
   * Find out whether the source cloud contains table,
   * if true, publish table geometry for obstacle avoiding.
   * The msg type is geometry_msgs/PoseStamped, which
   * contains pose of the table centroid, assuming the table is a box,
   * the size of table can be pre-defined
   */
  void detectObstacleTable();
  
  /**
   * @brief detectObstacle
   * With the target bounding box given, publish pointcloud without
   * the object
   * @param min_x
   * @param min_y
   * @param max_x
   * @param max_y
   */
  void detectObstacleInCloud(int min_x, int min_y, int max_x, int max_y);
  
  void detectObstacleInDepth(cv_bridge::CvImagePtr src_depth_ptr, 
                             int min_x, int min_y, int max_x, int max_y);
  
  void detectObstacleInDepth(int min_x, int min_y, int max_x, int max_y);
  
  /**
   * @brief detectPutTable
   * @param put_pose
   * @param ref_pose
   * @param need_move Whether need to move the robot to put down object
   * @return True if current scene contains table
   */
  bool detectPutTable(geometry_msgs::PoseStamped &put_pose, 
                      geometry_msgs::PoseStamped &ref_pose, 
                      bool &need_move);
  
  inline void setZOffset(float z_offset) {z_offset_ = z_offset;}
  
  // Whether perform obstacle detection
  bool use_od_;
  inline void setUseOD(bool use) {use_od_ = use;}
  inline bool getUseOD() {return use_od_;}
  
private:
  string param_running_mode_;
  
  ros::NodeHandle nh_;
  boost::shared_ptr<image_transport::ImageTransport> depth_it_;
  image_transport::ImageTransport pub_it_;
  
  image_transport::SubscriberFilter sub_depth_;
  image_transport::ImageTransport sub_it_;
  image_transport::Subscriber sub_d_;
  void initDepthCallback();
  
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;

  typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::Image, sensor_msgs::CameraInfo>
  SyncPolicyDepth;

  typedef message_filters::Synchronizer<SyncPolicyDepth> SynchronizerDepth;
  boost::shared_ptr<SynchronizerDepth> sync_depth_;
  
  // Depth image temp
  cv_bridge::CvImagePtr src_depth_ptr_;
  // Depth image info
  sensor_msgs::CameraInfo depth_cam_info_;
  void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg,
                     const sensor_msgs::CameraInfoConstPtr& camera_info_msg);
  
  // Source point cloud and its inliers after z filter
  PointCloudMono::Ptr src_cloud_;
  pcl::PointIndices::Ptr src_z_inliers_;
  ros::Subscriber sub_pointcloud_;
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  
  // The transform object can't be shared between Classes
  Transform *m_tf_;
  // Frame for point cloud to transfer
  string base_frame_;
  
  ros::Publisher pub_table_pose_;
  ros::Publisher pub_table_points_;
  ros::Publisher pub_exp_obj_cloud_;
  
  ros::Publisher pub_depth_cam_info_;
  image_transport::Publisher pub_exp_obj_depth_;
  
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
  float tolerance_;
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
  
  /**
   * @brief analysePutPose
   * @param put_pose If grasp area center already in hull, use it as put_pose
   * @param ref_pose The nearest point on hull from grasp area center
   * @return true if need move the robot
   */
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
  void publishCloud(PointTPtr cloud, ros::Publisher pub);
  void depthCb(const sensor_msgs::ImageConstPtr &depth_msg);
};

#endif // OBSTACLEDETECT_H
