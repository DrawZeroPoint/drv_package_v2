#include "makeplan.h"

MakePlan::MakePlan()
{
}

void MakePlan::removeOutliers(PointCloud::Ptr cloud_in, PointCloud::Ptr &cloud_out)
{
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // build the filter
  outrem.setInputCloud(cloud_in);
  outrem.setRadiusSearch(0.01);
  outrem.setMinNeighborsInRadius (4);
  // apply filter
  outrem.filter (*cloud_out);
}

bool MakePlan::getAveragePoint(PointCloud::Ptr cloud_in, pcl::PointXYZ &avrPt)
{
  if (!cloud_in->points.size())
  {
    return false;
  }
  
  pcl::PointXYZ min_dis_point;
  float min_dis = 100.0;
  for (PointCloud::const_iterator it = cloud_in->begin(); it != cloud_in->end(); ++ it)
  {
    float dis = pow(it->x, 2) + pow(it->y, 2);
    if (dis < min_dis)
    {
      min_dis = dis;
      min_dis_point = *it;
    }
  }
  
  smartOffset(min_dis_point, 0.02, 0.05);
  avrPt = min_dis_point;
  return true;
}

void MakePlan::removeNans(PointCloud::Ptr cloud_in, PointCloud::Ptr &cloud_out)
{
  std::vector< int >	index;
  PointCloud::Ptr cloud_filtered_nan (new PointCloud);
  
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_filtered_nan, index);
  
  pcl::PointIndices::Ptr indices_x(new pcl::PointIndices);
  pcl::PointIndices::Ptr indices_xy(new pcl::PointIndices);
  
  pcl::PassThrough<pcl::PointXYZ> ptfilter; // Initializing with true will allow us to extract the removed indices
  ptfilter.setInputCloud (cloud_filtered_nan);
  ptfilter.setFilterFieldName ("x");
  ptfilter.setFilterLimits (-0.2, 0.2);
  ptfilter.filter (indices_x->indices);
  
  ptfilter.setIndices (indices_x);
  ptfilter.setFilterFieldName ("y");
  ptfilter.setFilterLimits (-0.2, 0.2);
  ptfilter.filter (indices_xy->indices);
  
  ptfilter.setIndices (indices_xy);
  ptfilter.setFilterFieldName ("z");
  ptfilter.setFilterLimits (0.5, 2.0);// this method can only be used on source cloud, aka cloud whose frame is camera optical frame
  ptfilter.filter (*cloud_out);
}

bool MakePlan::process(PointCloud::Ptr cloud_in, float a, float b, float c, float d, pcl::PointXYZ &avrPt)
{
  if (!cloud_in->points.size())
  {
    return false;
  }
  
  PointCloud::Ptr cloud_filtered (new PointCloud);
#ifdef USE_CENTER
  cloud_filtered = cloud_in;
#else
  removeOutliers(cloud_in, cloud_filtered);
#endif
  bool success = getAveragePoint(cloud_filtered, avrPt);
  return success;
}
