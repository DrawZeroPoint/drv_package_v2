#include "targetlistener.h"

TargetListener::TargetListener()
{
  isTargetSet_ = false;
  targetLabel_ = "";

  // Param come from outside of vision system
  param_target_set =   "/comm/param/control/target/is_set";
  param_target_label = "/comm/param/control/target/label";
  param_comm_is_put = "/comm/param/ctrl/is_put";
  
  param_train_face_name = "/vision/face/train/name";
  // If this set to be true, perform training on face data
  param_train_face_run = "/vision/face/need_train";

  param_action_target_set =   "/vision/param/action/target/is_set";
  param_action_target_label = "/vision/param/action/target/label";
}

bool TargetListener::checkLabel(string label)
{
  size_t space = label.find_first_of(":");
  if (space == string::npos) {
    // No space in current string, so it should be target
    return true;
  }
  
  // Divide label by the first space
  string str_p = label.substr(0, space);
  string str_s = label.substr(space + 1);
  
  string name = "name";
  string train = "train";
  string face = "face";
  if (str_p.find(name) != string::npos) {
    ros::param::set(param_train_face_name, str_s);
  }
  else if (str_p.find(train) != string::npos && 
           str_s.find(face) != string::npos) {
    ros::param::set(param_train_face_run, true);
  }
  return false;
}

void TargetListener::getTargetStatus(bool &is_tgt_set, 
                                     string &tgt_label,
                                     bool &is_put)
{
  // First get target from action, then from Android device
  if (ros::param::has(param_action_target_set)) {
    ros::param::get(param_action_target_set, is_tgt_set);
    if (is_tgt_set) {
      if (ros::param::has(param_action_target_label))
        ros::param::get(param_action_target_label, tgt_label);
      else
        is_tgt_set = false;
    }
    else {
      // If action doesn't set the target, check whether user has set the target
      if (ros::param::has(param_target_set)) {
        ros::param::get(param_target_set, is_tgt_set);
        if (is_tgt_set) {
          if (ros::param::has(param_target_label)) {
            string label;
            ros::param::get(param_target_label, label);
            // Check if the label contains a object or not
            if (checkLabel(label)) {
              // checkLabel return true indicate that the label is a object
              is_tgt_set = true;
              tgt_label = label;
            }
            else {
              // If the target is not detection object, reset the params
              is_tgt_set = false;
              tgt_label = "";
              ros::param::del(param_target_label);
              ros::param::del(param_target_set);
            }
          }
          else {
            // TODO feedback lack of label
            tgt_label = "user selected object";
          }
        }
      }
      else
        is_tgt_set = false;
    }
  }
  else {
    if (ros::param::has(param_target_set)) {
      ros::param::get(param_target_set, is_tgt_set);
      if (is_tgt_set) {
        if (ros::param::has(param_target_label)) {
          string label;
          ros::param::get(param_target_label, label);
          // Check if the label contains a object or name to be trained
          if (checkLabel(label)) {
            // checkLabel return true indicate that the label is a object
            is_tgt_set = true;
            tgt_label = label;
          }
          else {
            is_tgt_set = false;
            tgt_label = "";
          }
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
  
  // Check if put the target
  if (is_tgt_set) {
    is_put = false;
    if (ros::param::has(param_comm_is_put)) {
      ros::param::del(param_comm_is_put);
    }
  }
  else {
    if (ros::param::has(param_comm_is_put)) {
      ros::param::get(param_comm_is_put, is_put);
      ros::param::del(param_comm_is_put);
    }
  }
}
