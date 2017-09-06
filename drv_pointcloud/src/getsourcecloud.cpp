#include "getsourcecloud.h"

GetSourceCloud::GetSourceCloud()
{
}

template<class T>
inline bool uIsFinite(const T & value)
{
#if _MSC_VER
		return _finite(value) != 0;
#else
		return std::isfinite(value);
#endif
}


float getDepth(
				const cv::Mat & depthImage,
				float x, float y,
				bool smoothing,
				float maxZError,
				bool estWithNeighborsIfNull)
{

		int u = int(x+0.5f);
		int v = int(y+0.5f);
		if(u == depthImage.cols && x<float(depthImage.cols))
				{
						u = depthImage.cols - 1;
				}
		if(v == depthImage.rows && y<float(depthImage.rows))
				{
						v = depthImage.rows - 1;
				}

		if(!(u >=0 && u<depthImage.cols && v >=0 && v<depthImage.rows))
				{
						return 0;
				}

		bool isInMM = depthImage.type() == CV_16UC1; // is in mm?

		// Inspired from RGBDFrame::getGaussianMixtureDistribution() method from
		// https://github.com/ccny-ros-pkg/rgbdtools/blob/master/src/rgbd_frame.cpp
		// Window weights:
		//  | 1 | 2 | 1 |
		//  | 2 | 4 | 2 |
		//  | 1 | 2 | 1 |
		int u_start = std::max(u-1, 0);
		int v_start = std::max(v-1, 0);
		int u_end = std::min(u+1, depthImage.cols-1);
		int v_end = std::min(v+1, depthImage.rows-1);

		float depth = 0.0f;
		if(isInMM)
				{
						if(depthImage.at<unsigned short>(v,u) > 0 &&
										depthImage.at<unsigned short>(v,u) < std::numeric_limits<unsigned short>::max())
								{
										depth = float(depthImage.at<unsigned short>(v,u))*0.001f;
								}
				}
		else
				{
						depth = depthImage.at<float>(v,u);
				}

		if((depth==0.0f || !uIsFinite(depth)) && estWithNeighborsIfNull)
				{
						// all cells no2 must be under the zError to be accepted
						float tmp = 0.0f;
						int count = 0;
						for(int uu = u_start; uu <= u_end; ++uu)
								{
										for(int vv = v_start; vv <= v_end; ++vv)
												{
														if((uu == u && vv!=v) || (uu != u && vv==v))
																{
																		float d = 0.0f;
																		if(isInMM)
																				{
																						if(depthImage.at<unsigned short>(vv,uu) > 0 &&
																										depthImage.at<unsigned short>(vv,uu) < std::numeric_limits<unsigned short>::max())
																								{
																										depth = float(depthImage.at<unsigned short>(vv,uu))*0.001f;
																								}
																				}
																		else
																				{
																						d = depthImage.at<float>(vv,uu);
																				}
																		if(d!=0.0f && uIsFinite(d))
																				{
																						if(tmp == 0.0f)
																								{
																										tmp = d;
																										++count;
																								}
																						else if(fabs(d - tmp) < maxZError)
																								{
																										tmp+=d;
																										++count;
																								}
																				}
																}
												}
								}
						if(count > 1)
								{
										depth = tmp/float(count);
								}
				}

		if(depth!=0.0f && uIsFinite(depth))
				{
						if(smoothing)
								{
										float sumWeights = 0.0f;
										float sumDepths = 0.0f;
										for(int uu = u_start; uu <= u_end; ++uu)
												{
														for(int vv = v_start; vv <= v_end; ++vv)
																{
																		if(!(uu == u && vv == v))
																				{
																						float d = 0.0f;
																						if(isInMM)
																								{
																										if(depthImage.at<unsigned short>(vv,uu) > 0 &&
																														depthImage.at<unsigned short>(vv,uu) < std::numeric_limits<unsigned short>::max())
																												{
																														depth = float(depthImage.at<unsigned short>(vv,uu))*0.001f;
																												}
																								}
																						else
																								{
																										d = depthImage.at<float>(vv,uu);
																								}

																						// ignore if not valid or depth difference is too high
																						if(d != 0.0f && uIsFinite(d) && fabs(d - depth) < maxZError)
																								{
																										if(uu == u || vv == v)
																												{
																														sumWeights+=2.0f;
																														d*=2.0f;
																												}
																										else
																												{
																														sumWeights+=1.0f;
																												}
																										sumDepths += d;
																								}
																				}
																}
												}
										// set window weight to center point
										depth *= 4.0f;
										sumWeights += 4.0f;

										// mean
										depth = (depth+sumDepths)/sumWeights;
								}
				}
		else
				{
						depth = 0;
				}
		return depth;
}


pcl::PointXYZ projectDepthTo3D(
				const cv::Mat & depthImage,
				float x, float y,
				float cx, float cy,
				float fx, float fy,
				bool smoothing,
				float maxZError)
{

		pcl::PointXYZ pt;

		float depth = getDepth(depthImage, x, y, smoothing, maxZError, true);
		if(depth > 0.0f)
				{
						// Use correct principal point from calibration
						cx = cx > 0.0f ? cx : float(depthImage.cols/2) - 0.5f; //cameraInfo.K.at(2)
						cy = cy > 0.0f ? cy : float(depthImage.rows/2) - 0.5f; //cameraInfo.K.at(5)

						// Fill in XYZ
						pt.x = (x - cx) * depth / fx;
						pt.y = (y - cy) * depth / fy;
						pt.z = depth;
				}
		else
				{
						pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
				}
		return pt;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDepthRGB(const cv::Mat & imageRgb, const cv::Mat & imageDepth,
																												 float cx, float cy, float fx, float fy, float maxDepth, float minDepth)
{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		bool mono;
		if(imageRgb.channels() == 3) // BGR
				{
						mono = false;
				}
		else if(imageRgb.channels() == 1) // Mono
				{
						mono = true;
				}
		else
				{
						return cloud;
				}

		int decimation = 1;

		//cloud.header = cameraInfo.header;
		cloud->height = imageRgb.rows/decimation;
		cloud->width  = imageRgb.cols/decimation;
		cloud->is_dense = false;
		cloud->resize(cloud->height * cloud->width);

		float rgbToDepthFactorX = 1.0f/float((imageRgb.cols / imageDepth.cols));
		float rgbToDepthFactorY = 1.0f/float((imageRgb.rows / imageDepth.rows));
		float depthFx = fx * rgbToDepthFactorX;
		float depthFy = fy * rgbToDepthFactorY;
		float depthCx = cx * rgbToDepthFactorX;
		float depthCy = cy * rgbToDepthFactorY;

		int oi = 0;
		for(int h = 0; h < imageRgb.rows && h/decimation < (int)cloud->height; h+=decimation)
				{
						for(int w = 0; w < imageRgb.cols && w/decimation < (int)cloud->width; w+=decimation)
								{
										pcl::PointXYZRGB & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));
										if(!mono)
												{
														pt.b = imageRgb.at<cv::Vec3b>(h,w)[0];
														pt.g = imageRgb.at<cv::Vec3b>(h,w)[1];
														pt.r = imageRgb.at<cv::Vec3b>(h,w)[2];
												}
										else
												{
														unsigned char v = imageRgb.at<unsigned char>(h,w);
														pt.b = v;
														pt.g = v;
														pt.r = v;
												}

										pcl::PointXYZ ptXYZ = projectDepthTo3D(imageDepth, w*rgbToDepthFactorX, h*rgbToDepthFactorY, depthCx, depthCy, depthFx, depthFy, false,3.0);
										if(pcl::isFinite(ptXYZ) && ptXYZ.z>=minDepth && (maxDepth<=0.0f || ptXYZ.z <= maxDepth))
												{
														pt.x = ptXYZ.x;
														pt.y = ptXYZ.y;
														pt.z = ptXYZ.z;
														++oi;
												}
										else
												{
														pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
												}
								}
				}
		if(oi == 0)
				{
						PCL_WARN("Cloud with only NaN values created!\n");
				}
		return cloud;
}

bool GetSourceCloud::getCloud(cv::Mat color, cv::Mat depth, float fx, float fy, float cx, float cy, float maxDepth, float minDepth,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr  &cloudSource)
{
    if (color.empty())
        {
            std::cerr << "color image is empty" << std::endl;
            return false;
        }
    if (depth.empty())
        {
            std::cerr << "depth image is empty" << std::endl;
            return false;
        }

    cloudSource = cloudFromDepthRGB(color, depth, cx, cy, fx, fy, maxDepth, minDepth);
    return true;
}
