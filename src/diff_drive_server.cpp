#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <teleop/diff_drive_conConfig.h>

void callback(teleop::diff_drive_conConfig &config, uint32_t level) {
  // ROS_INFO("Reconfigure Request: %d %f %s %s %d",
  //           config.int_para, config.double_param,
  //           config.str_param.c_str(),
  //           config.bool_param?"True":"False",
  //           config.size);
    ROS_INFO("Reconfigure Request: %f %f %f %f",
              config.Left_Forward_Scale,
              config.Right_Forward_Scale,
              config.Left_Backward_Scale,
              config.Right_Backward_Scale);

}

int main(int argc, char **argv) {
  //ros::init(argc, argv, "dynamic_tutorials");
  ros::init(argc, argv, "diff_drive_server_node");

  //dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig> server;
  dynamic_reconfigure::Server<teleop::diff_drive_conConfig> server;
  dynamic_reconfigure::Server<teleop::diff_drive_conConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
