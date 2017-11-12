#include <ros/ros.h>

#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>

#include <math.h>
#include <string.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt16MultiArray.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

// Custom message
#include <drv_msgs/recognized_target.h>
#include "makeplan.h"
#include "getsourcecloud.h"

// 3rd party package
#ifndef USE_CENTER
#include <gpd/GraspConfigList.h>
#endif

//PCL-ROS
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

//PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>


using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher graspPubStatus_;
ros::Publisher graspPubPose_;
ros::Publisher graspPubMarker_;

enum ModeType{m_wander, m_search, m_track};
int modeType_ = m_wander;

string param_running_mode = "/status/running_mode";

bool hasGraspPlan_ = false; // for output grasp info 
bool posePublished_ = false; // only works in simple mode

// marker type
uint32_t shape = visualization_msgs::Marker::ARROW;

// transform frame
string base_frame_ = "base_link"; // Base frame that NVG link to
string camera_optical_frame_ = "vision_depth_optical_frame";

geometry_msgs::TransformStamped trans_c_;

bool pub_pose_once_ = false;

/* The graspable area of robot left arm,
 * relative to base_link, in meter */
float x_min_ = 0.4;
float x_max_ = 0.6;
float y_min_ = 0.2;
float y_max_ = 0.35;
float tolerance_ = 0.03;

/* This location will be published if the target is out of range
   This describe the offset x, y value to be adjusted, z and
   orientation are not used */
ros::Publisher graspPubLocation_;

bool offsetNeedPub_ = true;

#ifdef USE_CENTER
int idx_;
int row_;
int col_;

float fx_ = 579.77;
float fy_ = 584.94;
float cx_ = 333.77;
float cy_ = 237.86;

double min_depth_ = 0.2;
double max_depth_ = 5.0;

#else
pcl::PointIndices::Ptr inliers_(new pcl::PointIndices);
ros::Publisher graspPubCloud_;
#endif

void trackResultCallback(const drv_msgs::recognized_targetConstPtr &msg)
{
  if (modeType_ != m_track)
    return;
  
  posePublished_ = false;
  
#ifdef USE_CENTER
  // Directly use bbox center as tgt location
  idx_ = msg->tgt_bbox_center.data[0] + msg->tgt_bbox_center.data[1] * 640;
  row_ = msg->tgt_bbox_center.data[1];
  col_ = msg->tgt_bbox_center.data[0];
#else
  inliers_->indices.clear();
  // use roi to extact point cloud
  for (size_t x = msg->tgt_bbox_array.data[0]; x < msg->tgt_bbox_array.data[2]; x++) {
    for (size_t y = msg->tgt_bbox_array.data[1]; y < msg->tgt_bbox_array.data[3]; y++) {
      int id = y * 640 + x;
      inliers_->indices.push_back(id);
    }
  }
#endif
}

void publishMarker(float x, float y, float z, std_msgs::Header header)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp. See the TF tutorials for information on these.
  marker.header.frame_id = base_frame_;
  marker.header.stamp = header.stamp;
  
  // Set the namespace and id for this marker. This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "grasp";
  marker.id = 0;
  
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  
  // Set the pose of the marker, which is a full 6DOF pose
  // relative to the frame/time specified in the header
  marker.points.resize(2);
  // The point at index 0 is assumed to be the start point,
  // and the point at index 1 is assumed to be the end.
  marker.points[0].x = x;
  marker.points[0].y = y;
  marker.points[0].z = z;
  
  marker.points[1].x = x;
  marker.points[1].y = y;
  marker.points[1].z = z + 0.15; // arrow height = 0.15m
  
  // If we use pose and scale to represent the arrow,
  // remember the original direction is along x axis
  // marker.scale.x = 0.1;
  // marker.scale.y = 0.01;
  // marker.scale.z = 0.01;
  // marker.pose = ps.pose;
  
  // scale.x is the shaft diameter, and scale.y is the head diameter.
  // If scale.z is not zero, it specifies the head length.
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.01;
  marker.scale.y = 0.015;
  marker.scale.z = 0.04;
  
  // Set the color -- be sure to set alpha as non-zero value!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  
  graspPubMarker_.publish(marker);
}

bool isInGraspRange(float x, float y, float z,
                    geometry_msgs::PoseStamped &offset)
{
  if (!offsetNeedPub_)
    return true;

  // Upper value
  float x_off_u = x - x_max_;
  float y_off_u = y - y_max_;
  // Lower value
  float x_off_l = x - x_min_;
  float y_off_l = y - y_min_;

  if (x_off_u > 0)
    offset.pose.position.x = x_off_u; // Robot move foreward
  else if (x_off_l < 0)
    offset.pose.position.x = x_off_l; // Robot move backward
  else
    offset.pose.position.x = 0;

  if (y_off_u > 0)
    offset.pose.position.y = y_off_u; // To left
  else if (y_off_l < 0)
    offset.pose.position.y = y_off_l; // To right
  else
    offset.pose.position.y = 0;

  if (fabs(offset.pose.position.x) < tolerance_ &&
      fabs(offset.pose.position.y) < tolerance_) {
    offsetNeedPub_ = false;
    return true;
  }
  else
    return false;
}

#ifdef USE_CENTER
void doTransform(pcl::PointXYZ p_in, pcl::PointXYZ &p_out, 
                 const geometry_msgs::TransformStamped& t_in)
{
  Eigen::Transform<float,3,Eigen::Affine> t = 
      Eigen::Translation3f(t_in.transform.translation.x,   
                           t_in.transform.translation.y,
                           t_in.transform.translation.z) * 
      Eigen::Quaternion<float>(t_in.transform.rotation.w, 
                               t_in.transform.rotation.x,
                               t_in.transform.rotation.y, 
                               t_in.transform.rotation.z);
  
  Eigen::Vector3f point;
  
  point = t * Eigen::Vector3f(p_in.x, p_in.y, p_in.z);
  p_out.x = point.x();
  p_out.y = point.y();
  p_out.z = point.z();
}

void depthCallback(const sensor_msgs::ImageConstPtr& imageDepth)
{
  // In simple mode, pose only publish once after one detection
  if (modeType_ != m_track || (pub_pose_once_ && posePublished_))
    return;
  
  if(!(imageDepth->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
       imageDepth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
       imageDepth->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0))
  {
    ROS_ERROR("Depth image type is wrong.");
    return;
  }
  
  cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(imageDepth);
  
  MakePlan MP;
  
  pcl::PointXYZ graspPt; // target xyz center in robot's referance frame
  pcl::PointXYZ opticalPt; // target xyz center in camera optical frame
  
  // Get opticalPt and transfer to graspPt
  if (GetSourceCloud::getPoint(imageDepthPtr->image, row_, col_,
                               fx_, fy_, cx_, cy_, 
                               max_depth_, min_depth_, opticalPt))
  {
    doTransform(opticalPt, graspPt, trans_c_);
    MP.smartOffset(graspPt, 0.02); //TODO: make this smart
    
    publishMarker(graspPt.x, graspPt.y, graspPt.z, imageDepth->header);
    
    /* Judge if the graspPt is within the graspable area, if so,
     * publish the pose via graspPubPose_, otherwise, 
     * publish the adjustment value via graspPubLocation_ */
    geometry_msgs::PoseStamped offset;
    bool in_range = isInGraspRange(graspPt.x, graspPt.y, graspPt.z, offset);
    if (in_range) {
      geometry_msgs::PoseStamped grasp_pose;
      grasp_pose.header.frame_id = base_frame_;
      grasp_pose.header.stamp = imageDepth->header.stamp;
      grasp_pose.pose.position.x = graspPt.x;
      grasp_pose.pose.position.y = graspPt.y;
      grasp_pose.pose.position.z = graspPt.z;
      
      grasp_pose.pose.orientation.w = 1;
      grasp_pose.pose.orientation.x = 0;
      grasp_pose.pose.orientation.y = 0;
      grasp_pose.pose.orientation.z = 0;
      graspPubPose_.publish(grasp_pose);

      posePublished_ = true;
    }
    else {
      offset.header.frame_id = base_frame_;
      offset.header.stamp = imageDepth->header.stamp;
      offset.pose.position.z = 0;
      offset.pose.orientation.w = 1;
      offset.pose.orientation.x = 0;
      offset.pose.orientation.y = 0;
      offset.pose.orientation.z = 0;
      graspPubLocation_.publish(offset);
    }
    hasGraspPlan_ = true;
  }
  else
    ROS_INFO("Get grasp point failed!");
}
#else
void getCloudByInliers(PointCloud::Ptr cloud_in, PointCloud::Ptr &cloud_out,
                       pcl::PointIndices::Ptr inliers, bool negative, bool organized)
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setNegative (negative);
  extract.setInputCloud (cloud_in);
  extract.setIndices (inliers);
  extract.setKeepOrganized(organized);
  extract.filter (*cloud_out);
}

void doTransform(PointCloud::Ptr cloud_in, PointCloud::Ptr &cloud_out,
                 const geometry_msgs::TransformStamped& t_in)
{
  Eigen::Transform<float,3,Eigen::Affine> t = Eigen::Translation3f(t_in.transform.translation.x,
                                                                   t_in.transform.translation.y,
                                                                   t_in.transform.translation.z)
      * Eigen::Quaternion<float>(t_in.transform.rotation.w, t_in.transform.rotation.x,
                                 t_in.transform.rotation.y, t_in.transform.rotation.z);
  
  Eigen::Vector3f point;
  
  cloud_out->height = cloud_in->height;
  cloud_out->width  = cloud_in->width;
  cloud_out->is_dense = false;
  cloud_out->resize(cloud_out->height * cloud_out->width);
  
  for (size_t i = 0; i < cloud_in->size(); ++i) {
    point = t * Eigen::Vector3f(cloud_in->points[i].x, 
                                cloud_in->points[i].y, 
                                cloud_in->points[i].z);
    cloud_out->points[i].x = point.x();
    cloud_out->points[i].y = point.y();
    cloud_out->points[i].z = point.z();
  }
}

void getOrientation(gpd::GraspConfig g, geometry_msgs::Quaternion &q)
{
  float r11 = g.approach.x;
  float r21 = g.approach.y;
  float r31 = g.approach.z;
  float r12 = g.binormal.x;
  float r22 = g.binormal.y;
  float r32 = g.binormal.z;
  float r13 = - g.axis.x;
  float r23 = - g.axis.y;
  float r33 = - g.axis.z;
  
  q.w = 0.5 * sqrt(1 + r11 + r22 + r33);
  
  if (q.w == 0.0)
    q.x = sqrt(0.5 * (1 + r11));
  else
    q.x = (r23 - r32) / (4 * q.w);
  
  if (q.w == 0.0)
    q.y = sqrt(0.5 * (1 + r22));
  else
    q.y = (r31 - r13) / (4 * q.w);
  
  if (q.w == 0.0)
    q.z = sqrt(0.5 * (1 + r33));
  else
    q.z = (r12 - r21) / (4 * q.w);
}

void sourceCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // In simple mode, pose only publish once after one detection
  if (modeType_ != m_track || (pub_pose_once_ && posePublished_))
    return;
  
  if (inliers_->indices.empty()) {
    ROS_ERROR("Grasp: Target ROI contains no point.");
    return;
  }
  
  PointCloud::Ptr cloud_in(new PointCloud);
  PointCloud::Ptr cloud_filted(new PointCloud);
  PointCloud::Ptr cloud_trans(new PointCloud);
  
  // Filt the cloud using inliers
  pcl::fromROSMsg(*msg, *cloud_in);
  getCloudByInliers(cloud_in, cloud_filted, inliers_, false, false);
  
  // Convert point cloud inliers into base frame
  doTransform(cloud_filted, cloud_trans, trans_c_);
  
  // Publish filted cloud for gpd
  sensor_msgs::PointCloud2 rosCloud;
  pcl::toROSMsg(*cloud_trans, rosCloud);
  rosCloud.header.frame_id = base_frame_;
  rosCloud.header.stamp = msg->header.stamp;
  
  graspPubCloud_.publish(rosCloud);
}

void graspPlanCallback(const gpd::GraspConfigListConstPtr &msg)
{
  // In simple mode, pose only publish once after one detection
  if (modeType_ != m_track || (pub_pose_once_ && posePublished_))
    return;
  
  if (msg->grasps.size() == 0) {
    ROS_ERROR("Get grasp plan failed!");
    return;
  }
  
  gpd::GraspConfig grasp;
  gpd::GraspConfig grasp_max_score;
  float score = 0.0; // classification score for grasp
  for (size_t i = 0; i < msg->grasps.size(); ++i) {
    grasp = msg->grasps[i];
    
    if (grasp.score.data > score) {
      score = grasp.score.data;
      grasp_max_score = grasp;
    }
  }
  
  // Publish grasp pose with max score
  geometry_msgs::PoseStamped grasp_pose;
  grasp_pose.header = msg->header;
  grasp_pose.pose.position.x = grasp_max_score.bottom.x;
  grasp_pose.pose.position.y = grasp_max_score.bottom.y;
  grasp_pose.pose.position.z = grasp_max_score.bottom.z;

  geometry_msgs::PoseStamped offset;
  bool in_range = isInGraspRange(grasp_pose.pose.position.x,
                                 grasp_pose.pose.position.y,
                                 grasp_pose.pose.position.z,
                                 offset);
  if (in_range) {
    grasp_pose.header.frame_id = base_frame_;
    grasp_pose.header.stamp = msg->header.stamp;

    /* Use default orientation, which indicates that the
     * hand's coord is identical to the base */
    //grasp_ps.pose.orientation.w = 1;
    //grasp_ps.pose.orientation.x = 0;
    //grasp_ps.pose.orientation.y = 0;
    //grasp_ps.pose.orientation.z = 0;

    // Get orientation from message calculated by gpd
    getOrientation(grasp_max_score, grasp_pose.pose.orientation);

    graspPubPose_.publish(grasp_pose);
    posePublished_ = true;
  }
  else {
    offset.header.frame_id = base_frame_;
    offset.header.stamp = msg->header.stamp;

    // Set the rest values in offset
    offset.pose.position.z = 0;
    offset.pose.orientation.w = 1;
    offset.pose.orientation.x = 0;
    offset.pose.orientation.y = 0;
    offset.pose.orientation.z = 0;
    graspPubLocation_.publish(offset);
  }
  
  // Publish marker of grasp pose
  publishMarker(grasp_max_score.bottom.x, grasp_max_score.bottom.y,
                grasp_max_score.bottom.z, msg->header);
  
  hasGraspPlan_ = true;
}
#endif

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drv_grasp");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  pnh.getParam("base_frame_id", base_frame_);
  pnh.getParam("camera_optical_frame_id", camera_optical_frame_);
  pnh.getParam("pub_pose_once_id", pub_pose_once_);
  
  // Get left arm movable area params
  pnh.getParam("left_arm_x_min", x_min_);
  pnh.getParam("left_arm_y_min", y_min_);
  pnh.getParam("left_arm_x_max", x_max_);
  pnh.getParam("left_arm_y_max", y_max_);
  
  graspPubStatus_ = nh.advertise<std_msgs::Int8>("status/grasp/feedback", 1);
  graspPubPose_ = nh.advertise<geometry_msgs::PoseStamped>("grasp/pose", 1);
  graspPubLocation_ = nh.advertise<geometry_msgs::PoseStamped>("grasp/location", 1);
  graspPubMarker_ = nh.advertise<visualization_msgs::Marker>("grasp/marker", 1);
  
  ros::Subscriber sub_track = nh.subscribe<drv_msgs::recognized_target>("track/recognized_target",
                                                                        1, trackResultCallback);
  
#ifdef USE_CENTER
  ros::NodeHandle cnh;
  ros::NodeHandle depth_nh(nh, "depth");
  ros::NodeHandle depth_pnh(cnh, "depth");
  image_transport::ImageTransport depth_it(depth_nh);
  image_transport::TransportHints hintsDepth("compressedDepth", ros::TransportHints(), depth_pnh);

  image_transport::Subscriber sub_depth = depth_it.subscribe("image_rect", 1, depthCallback, hintsDepth);
#else
  graspPubCloud_ = nh.advertise<sensor_msgs::PointCloud2>("grasp/points", 1);
  ros::Subscriber sub_cloud = nh.subscribe("depth_registered/points", 1, sourceCloudCallback);
  ros::Subscriber sub_grasp_plan = nh.subscribe<gpd::GraspConfigList>("/detect_grasps/clustered_grasps",
                                                                      1, graspPlanCallback);
#endif
  
  // These declaration must be after the node initialization
  tf2_ros::Buffer tfBufferCameraToBase_;
  // This is mandatory and should be declared before while loop
  tf2_ros::TransformListener tfListener(tfBufferCameraToBase_);
  
  ROS_INFO("Grasp planning function initialized!");
  
  while (ros::ok()) {
    if (ros::param::has(param_running_mode))
      ros::param::get(param_running_mode, modeType_);
    
    if (modeType_ != m_track) {
      // Reset the status
      hasGraspPlan_ = false;
      posePublished_ = false;
      offsetNeedPub_ = true;
      pub_pose_once_ = false;
      continue;
    }
    
    try {
      // The 1st frame is the base frame for transform
      trans_c_ = tfBufferCameraToBase_.lookupTransform(base_frame_, camera_optical_frame_,
                                                       ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    ros::spinOnce();

    std_msgs::Int8 flag;
    flag.data = 0;

    if (hasGraspPlan_ && !posePublished_)
      flag.data = 1;
    if (hasGraspPlan_ && posePublished_) {
      // Force pub_pose_once_ = true to prevent the pose being published twice
      pub_pose_once_ = true;
      flag.data = 2;
    }
    
    graspPubStatus_.publish(flag);
  }
  
  return 0;
}
