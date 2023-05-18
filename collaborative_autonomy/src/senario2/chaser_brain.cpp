#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <thread>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

/* This is designed to chase the LEADER TB3 */
/* Simple chase algorithm that stops chasing if within proximity of LEADER */

int main(int argc, char** argv){

  ros::init(argc, argv, "chaser_brain");
  ROS_INFO_STREAM("STARTING chaser_brain...");

  while(ros::ok()){
    ROS_INFO_STREAM("Chaser Placeholder running...");
    this_thread::sleep_for (chrono::milliseconds(2000));
  }

  return 0;
}