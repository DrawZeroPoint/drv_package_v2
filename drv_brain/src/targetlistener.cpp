#include "targetlistener.h"

TargetListener::TargetListener()
{
  isTargetSet_ = false;
  targetLabel_ = "";

  param_target_set =   "/comm/param/control/target/is_set";
  param_target_label = "/comm/param/control/target/label";

  param_action_target_set =   "/vision/param/action/target/is_set";
  param_action_target_label = "/vision/param/action/target/label";
}

void TargetListener::getTargetStatus(bool &is_tgt_set, string &tgt_label)
{
  // first get target from action, then from control
  if (ros::param::has(param_action_target_set))
  {
    ros::param::get(param_action_target_set, is_tgt_set);
    if (is_tgt_set)
    {
      if (ros::param::has(param_action_target_label))
      {
        ros::param::get(param_action_target_label, tgt_label);
      }
      else
        is_tgt_set = false;
    }
    else
    {
      // if action doesn't set the target, check whether user has set the target
      if (ros::param::has(param_target_set))
      {
        ros::param::get(param_target_set, is_tgt_set);
        if (is_tgt_set)
        {
          if (ros::param::has(param_target_label))
          {
            ros::param::get(param_target_label, tgt_label);
          }
          else
          {
            // TODO feedback lack of label
            tgt_label = "user selected object";
          }
        }
      }
      else
        is_tgt_set = false;
    }
  }
  else
  {
    if (ros::param::has(param_target_set))
    {
      ros::param::get(param_target_set, is_tgt_set);
      if (is_tgt_set)
      {
        if (ros::param::has(param_target_label))
        {
          ros::param::get(param_target_label, tgt_label);
        }
        else
        {
          // TODO feedback lack of label
          tgt_label = "user selected object";
        }
      }
    }
    else
      is_tgt_set = false;
  }
}
