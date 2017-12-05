#include "transform.h"

Transform::Transform() :
  tf_buffer_(),
  tf_listener_(tf_buffer_, nh_)
{
  // Initialize node handler before tf_buffer is important
}

bool Transform::getTransform(string base_frame, string header_frame)
{
  try {
    // While we aren't supposed to be shutting down
    while (ros::ok()) {
      // Check if the transform from map to quad can be made right now
      if (tf_buffer_.canTransform(base_frame, header_frame, ros::Time(0))) {
        // Get the transform
        tf_handle_ = tf_buffer_.lookupTransform(base_frame, header_frame, ros::Time(0));
        return true;
      }
      else {
        ROS_WARN("Transform: Frame %s does not exist.", base_frame.c_str());
      }
      
      // Handle callbacks and sleep for a small amount of time
      // before looping again
      ros::spinOnce();
      ros::Duration(0.005).sleep();
    }
  }
  // Catch any exceptions that might happen while transforming
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("Exception transforming %s to %s: %s",
              base_frame.c_str(),
              header_frame.c_str(),
              ex.what());
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
  
  cloud_out->height = cloud_in->height;
  cloud_out->width  = cloud_in->width;
  cloud_out->is_dense = false;
  cloud_out->resize(cloud_out->height * cloud_out->width);
  
  Eigen::Vector3f point;
  size_t i = 0;
  for (PointCloudMono::const_iterator pit = cloud_in->begin();
       pit != cloud_in->end(); ++pit) {
    point = t * Eigen::Vector3f(pit->x, pit->y, pit->z);
    cloud_out->points[i].x = point.x();
    cloud_out->points[i].y = point.y();
    cloud_out->points[i].z = point.z();
    ++i;
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
