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
/* As we have a subscriber, looks like we'll need ros::spin() */
// http://wiki.ros.org/evarobot_odometry/Tutorials/indigo/Writing%20a%20Simple%20Subscriber%20for%20Odometry
// <node name="chaser_brain" pkg="collaborative_autonomy"  type="chaser_brain" output="screen" launch-prefix="gnome-terminal --command"/>

void chaserCallback(){

}


int main(int argc, char** argv){

  ros::init(argc, argv, "chaser_brain");
  ROS_INFO_STREAM("STARTING chaser_brain...");
  ros::NodeHandle nh;

  ros::Subscriber sub = n.subscribe("tb3/leader/odom", 1000, chatterCallback);

  ros::spin();

  ros::shutdown();

  return 0;
}