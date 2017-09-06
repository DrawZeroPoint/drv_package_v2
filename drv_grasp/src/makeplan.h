#ifndef MAKEPLAN_H
#define MAKEPLAN_H

#define USE_CENTER

//STL
#include <iostream>
#include <math.h>
#include <vector>

//PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
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

class MakePlan
{
public:
  MakePlan();

  bool process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, float a, float b, float c, float d, pcl::PointXYZRGB &avrPt);
  void removeNans(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);
  void smartOffset(pcl::PointXYZRGB &p_in, float off_val);

private:
  void removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);
  bool getAveragePoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointXYZRGB &avrPt);

};

#endif // MAKEPLAN_H

