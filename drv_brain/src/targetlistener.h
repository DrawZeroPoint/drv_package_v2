#ifndef TARGETLISTENER_H
#define TARGETLISTENER_H

#include <ros/ros.h>

using namespace std;

class TargetListener
{
public:
  TargetListener();
  void getTargetStatus(bool &is_tgt_set, string &tgt_label);

private:
  bool isTargetSet_;
  string targetLabel_;

  // target info from control
  string param_target_set;
  string param_target_label;
  // target info from action
  string param_action_target_set;
  string param_action_target_label;
};

#endif // TARGETLISTENER_H
