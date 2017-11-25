#include "transform.h"

Transform::Transform() :
  tf_listener_(tf_buffer_)
{
}

bool Transform::getTransform(string base_frame, string header_frame)
{
  try {
    // Convert header_frame value to base_frame value
    tf_handle_ = tf_buffer_.lookupTransform(base_frame, header_frame, ros::Time(0));
    return true;
  }
  catch(tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }
}

void Transform::doTransform(PointCloudMono::Ptr cloud_in, 
                            PointCloudMono::Ptr &cloud_out)
{
  geometry_msgs::Vector3 trans = tf_handle_.transform.translation;
  geometry_msgs::Quaternion rotate = tf_handle_.transform.rotation;
  
  Eigen::Transform<float,3,Eigen::Affine> t = Eigen::Translation3f(trans.x,
                                                                   trans.y,
                                                                   trans.z)
      * Eigen::Quaternion<float>(rotate.w, rotate.x, rotate.y, rotate.z);
  
  Eigen::Vector3f point;
  
  cloud_out->height = cloud_in->height;
  cloud_out->width  = cloud_in->width;
  cloud_out->is_dense = false;
  cloud_out->resize(cloud_out->height * cloud_out->width);
  
  size_t i = 0;
  for (PointCloudMono::const_iterator pit = cloud_in->begin();
       pit != cloud_in->end(); ++pit) {
    point = t * Eigen::Vector3f(pit->x, pit->y, pit->z);
    cloud_out->points[i].x = point.x();
    cloud_out->points[i].y = point.y();
    cloud_out->points[i].z = point.z();
  }
}

void Transform::doTransform(pcl::PointXYZ p_in, pcl::PointXYZ &p_out)
{
  geometry_msgs::Vector3 trans = tf_handle_.transform.translation;
  geometry_msgs::Quaternion rotate = tf_handle_.transform.rotation;
  
  Eigen::Transform<float,3,Eigen::Affine> t = Eigen::Translation3f(trans.x,
                                                                   trans.y,
                                                                   trans.z)
      * Eigen::Quaternion<float>(rotate.w, rotate.x, rotate.y, rotate.z);
  
  Eigen::Vector3f point;
  
  point = t * Eigen::Vector3f(p_in.x, p_in.y, p_in.z);
  p_out.x = point.x();
  p_out.y = point.y();
  p_out.z = point.z();
}
