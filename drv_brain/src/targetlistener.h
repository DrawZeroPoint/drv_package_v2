#ifndef TARGETLISTENER_H
#define TARGETLISTENER_H

#include <ros/ros.h>
#include <string>

using namespace std;

class TargetListener
{
public:
  TargetListener();
  void getTargetStatus(bool &is_tgt_set, string &tgt_label);

private:
  bool isTargetSet_;
  string targetLabel_;

  // Target info from Android device
  string param_target_set;
  string param_target_label;
  
  // Training face recognition info
  string param_train_face_name;
  string param_train_face_run;
  
  // Target info from action
  string param_action_target_set;
  string param_action_target_label;
  
  bool checkLabel(string label);
};

#endif // TARGETLISTENER_H
