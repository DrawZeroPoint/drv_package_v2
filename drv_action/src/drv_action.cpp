#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/server/simple_action_server.h>
#include <drv_msgs/VisionAction.h>

#include <drv_msgs/recognized_faces.h>

using namespace std;

enum FeedbackType{f_wander, f_working, f_failed, f_success};
enum GoalType{g_none, g_object, g_face};

class VisionAction
{
protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<drv_msgs::VisionAction> as_;
  string action_name_;
  // create messages that are used to published feedback/result
  boost::shared_ptr<const drv_msgs::VisionGoal_ <std::allocator<void> > > goal_;
  int goal_mode_;
  string target_label_;
  drv_msgs::VisionFeedback feedback_;
  drv_msgs::VisionResult result_;

  ros::Subscriber sub_face_;
  ros::Subscriber sub_object_pose_;
  ros::Subscriber sub_object_location_;
  ros::Publisher pub_info_;

public:
  VisionAction(std::string name) :
    as_(nh_, name,  false), action_name_(name)
  {
    param_action_target_set =   "/vision/param/action/target/is_set";
    param_action_target_label = "/vision/param/action/target/label";
    param_action_need_recognize_face = "/vision/param/action/face/need_recognize";

    param_vision_feedback = "/comm/param/feedback/vision/overall";

    status_feedback_ = f_wander;

    as_.registerGoalCallback(boost::bind(&VisionAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&VisionAction::preemptCB, this));

    sub_face_ = nh_.subscribe<drv_msgs::recognized_faces>("face/recognized_faces", 1, &VisionAction::faceCB, this);
    // the sub order should be the same with the order that msg published
    sub_object_location_ = nh_.subscribe<geometry_msgs::PoseStamped>("grasp/location", 1, &VisionAction::objectLocationCB, this);
    sub_object_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("grasp/pose", 1, &VisionAction::objectCB, this);

    pub_info_ = nh_.advertise<std_msgs::String>("/comm/msg/vision/mode", 1);

    trigger_ = false;
    as_.start();
  }

  ~VisionAction(void)
  {
  }

  void pubInfo(string info)
  {
    ROS_INFO(info.c_str());
    std_msgs::String msg;
    msg.data = info;
    pub_info_.publish(msg);
  }

  void getStatus()
  {
    ros::param::get(param_vision_feedback, status_feedback_);
  }

  void resetStatus()
  {
    status_feedback_ = f_wander;
    ros::param::set(param_vision_feedback, f_wander);
    ros::param::set(param_action_target_set, false);
    ros::param::set(param_action_target_label, "");
    ros::param::set(param_action_need_recognize_face, false);
    trigger_ = false;
  }

  void goalCB()
  {
    resetStatus(); // clear the way for action goal
    goal_ = as_.acceptNewGoal();
    goal_mode_ = goal_->mode;
    target_label_ = goal_->target_label;
    if (goal_mode_ == g_object && target_label_ != "")
    {
      ros::param::set(param_action_target_label, target_label_);
      ros::param::set(param_action_target_set, true);
    }
    if (goal_mode_ == g_face)
    {
      ros::param::set(param_action_need_recognize_face, true);
    }
    trigger_ = true;
  }

  void preemptCB()
  {
    pubInfo("Vision Action: Preempted.");
    as_.setPreempted();
    resetStatus();
  }

  void faceCB(const drv_msgs::recognized_facesConstPtr &face)
  {
    if (!as_.isActive() || goal_mode_ != g_face || !trigger_)
      return;

    getStatus();
    if (status_feedback_ == f_failed)
    {
      pubInfo("Vision Action: Failed to find face in current scene!");
      as_.setAborted(result_);
    }
    else
    {
      if (status_feedback_ != f_success)
        return;

      result_.ids = face->name_ids;
      result_.names = face->names;
      pubInfo("Vision Action: Face recognition succeeded.");

      if (face->names.empty())
        pubInfo("Vision Action: Did not find face in current scene.");

      as_.setSucceeded(result_);
    }
    resetStatus();
  }

  void objectLocationCB(const geometry_msgs::PoseStampedConstPtr &lo)
  {
    if (!as_.isActive() || goal_mode_ !=  g_object || !trigger_)
      return;
    location_.header = lo->header;
    location_.pose = lo->pose;
  }

  void objectCB(const geometry_msgs::PoseStampedConstPtr &ps)
  {
    getStatus();
    if (!as_.isActive() || goal_mode_ !=  g_object || !trigger_)
      return;

    if (ps->pose.position.z == 0)
      return;

    if (status_feedback_ == f_failed)
    {
      pubInfo("Vision Action: Failed to find target in current scene!");
      as_.setAborted(result_);
    }
    else
    {
      if (status_feedback_ != f_success)
        return;

      result_.target_pose.header = ps->header;
      result_.target_pose.pose = ps->pose;

      // result_.target_location = location_;
      result_.target_location.pose.position = ps->pose.position;
      result_.target_location.pose.orientation.w = 1;

      pubInfo("Vision Action: Object recognition succeeded.");
      as_.setSucceeded(result_);
    }
    resetStatus();
  }

private:
  // main switch, true if the goal has been received
  bool trigger_;

  // goal
  string param_action_target_set;
  string param_action_target_label;
  string param_action_need_recognize_face;

  // overall feedback
  int status_feedback_;
  string param_vision_feedback;

  // result
  geometry_msgs::PoseStamped location_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "drv_action");

  VisionAction action(ros::this_node::getName());
  ros::spin();

  return 0;
}


