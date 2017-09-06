#include "targetselect.h"

using namespace cv;

TargetSelect::TargetSelect() : rgb_it(nh)
{
  searchPubImage_ = rgb_it.advertise("search/labeled_image", 1);
  searchPubInfo_ = nh.advertise<std_msgs::String>("/comm/vision/info", 1);
  client = nh.serviceClient<drv_msgs::user_select>("drv_user");
}

int TargetSelect::select(std::string targetLabel, drv_msgs::recognizeResponse response,
                         sensor_msgs::Image img_in, cv_bridge::CvImagePtr &img_out, int &choosed_id)
{
  int selectedNum = 0; // 0 represent no target

  // get scene image
  try
  {
    img_out = cv_bridge::toCvCopy(img_in, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  std::vector<int> id_candidate;

  // fetch target(s) number and paint on image
  int fc = 0;
  for (size_t i = 0; i < response.obj_info.labels.size(); i++)
  {
    if (targetLabel == response.obj_info.labels[i].data)
    {
      id_candidate.push_back(i);
      fc++;
      paintTarget(img_out->image, i, fc, response.obj_info.bbox_arrays);
    }
  }

  searchPubImage_.publish(img_out->toImageMsg());

  if (fc == 0)
  {
    selectedNum = 0;
  }
  else
  {
    ROS_INFO("Found %d object(s) that may be the target!\n", fc);
    selectedNum = callService(fc); // value range [0,inf)
  }

  if (selectedNum > 0)
    choosed_id = id_candidate[selectedNum - 1];

  return selectedNum;
}

int TargetSelect::callService(int num)
{
  drv_msgs::user_select srv;

  srv.request.select_num = num;// target number to be selected, start from 1
  int result = 0;

  if (client.call(srv))
  {
    result = srv.response.selected_id;
    if (result == 0)
    {
      ROS_INFO("You have confirmed current scene have no target.");
    }
    else
    {
      if (result <= num && result >= 1)
      {
        ROS_INFO("The number %d object confirmed to be the target!\n", result);
      }
      else
      {
        result = 0;
        ROS_ERROR("Invalid target number, the selectd number should between 1 to %d!\n", num);
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to call user select service.");
  }

  return result;
}

void TargetSelect::paintTarget(cv::Mat &img, int id, int fc, std::vector<std_msgs::UInt16MultiArray> box_array)
{
  std_msgs::UInt16MultiArray array = box_array[id]; // array index start from 0, while num start from 1
  int x = (array.data[0] + array.data[2]) / 2;
  int y = (array.data[1] + array.data[3]) / 2;

  Scalar color = Scalar(195, 42, 242); // bgr order, don't miss 'Scalar'

  circle(img, Point(x,y), 12, color, 2);
  if(y - 25 > 0)
    line(img, Point(x,y), Point(x,y - 25), color, 2);
  else line(img, Point(x,y), Point(x, 0), color, 2);
  if(y + 25 < 480)
    line(img, Point(x, y), Point(x, y + 25), color, 2);
  else line(img, Point(x, y), Point(x, 480), color, 2);
  if(x - 25 > 0)
    line(img, Point(x, y), Point(x - 25,y), color, 2);
  else line(img, Point(x, y), Point(0, y), color, 2);
  if(x + 25 < 640)
    line(img, Point(x,y), Point(x + 25,y), color, 2);
  else line(img, Point(x,y), Point(640, y), color, 2);

  putText(img, intToString(fc), Point(x + 10, y - 10), 1, 2, color, 2);
}
