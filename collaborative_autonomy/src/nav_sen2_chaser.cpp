#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <thread>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

/* This is designed to repeatedly drive a singular TB3 to four corners of the map */

int main(int argc, char** argv){

  ros::init(argc, argv, "nav_sen2_chaser");
  ROS_INFO_STREAM("STARTING nav_sen2_chaser...");

  while(ros::ok()){
    ROS_INFO_STREAM("Chaser Placeholder running...");
    this_thread::sleep_for (chrono::milliseconds(2000));
  }

  return 0;
}