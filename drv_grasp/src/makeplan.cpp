#include "makeplan.h"

MakePlan::MakePlan()
{
}

void MakePlan::removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
  // build the filter
  outrem.setInputCloud(cloud_in);
  outrem.setRadiusSearch(0.01);
  outrem.setMinNeighborsInRadius (4);
  // apply filter
  outrem.filter (*cloud_out);
}

bool MakePlan::getAveragePoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointXYZRGB &avrPt)
{
  if (!cloud_in->points.size())
  {
    return false;
  }

  pcl::PointXYZRGB min_dis_point;
  float min_dis = 100.0;
  for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = cloud_in->begin(); it != cloud_in->end(); ++ it)
  {
    float dis = pow(it->x, 2) + pow(it->y, 2);
    if (dis < min_dis)
    {
      min_dis = dis;
      min_dis_point = *it;
    }
  }

  smartOffset(min_dis_point, 0.02);
  avrPt = min_dis_point;
  return true;
}

void MakePlan::smartOffset(pcl::PointXYZRGB &p_in, float off_val)
{
  float y_off = off_val / sqrt(1 + pow(p_in.x / p_in.y, 2)) * p_in.y / fabs(p_in.y);
  float x_off = p_in.x / p_in.y * y_off;
  p_in.x = p_in.x + x_off;
  p_in.y = p_in.y + y_off;
}

void MakePlan::removeNans(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
  std::vector< int >	index;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_nan (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_filtered_nan, index);

  pcl::PointIndices::Ptr indices_x(new pcl::PointIndices);
  pcl::PointIndices::Ptr indices_xy(new pcl::PointIndices);

  pcl::PassThrough<pcl::PointXYZRGB> ptfilter; // Initializing with true will allow us to extract the removed indices
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

bool MakePlan::process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, float a, float b, float c, float d, pcl::PointXYZRGB &avrPt)
{
  if (!cloud_in->points.size())
  {
    return false;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
#ifdef USE_CENTER
  cloud_filtered = cloud_in;
#else
  removeOutliers(cloud_in, cloud_filtered);
#endif
  bool success = getAveragePoint(cloud_filtered, avrPt);
  return success;
}
