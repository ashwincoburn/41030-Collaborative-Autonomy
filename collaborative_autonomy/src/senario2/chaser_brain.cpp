#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <thread>
#include "nav_msgs/Odometry.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

/* This is designed to chase the LEADER TB3 */
/* Simple chase algorithm that stops chasing if within proximity of LEADER */
/* As we have a subscriber, looks like we'll need ros::spin() */
// http://wiki.ros.org/evarobot_odometry/Tutorials/indigo/Writing%20a%20Simple%20Subscriber%20for%20Odometry
// <node name="chaser_brain" pkg="collaborative_autonomy"  type="chaser_brain" output="screen" launch-prefix="gnome-terminal --command"/>
//Write a class that saves odom data, thread that checks for changes in the data and sends updates

void chaserCallback(const nav_msgs::Odometry::ConstPtr& msg, unsigned int &headerSequencer = 0;){
  /*
    Non-Blocking Function, designed to run once and fast
  */
  move_base_msgs::MoveBaseGoal pointToReach;

  /* Head Towards XYZ of LEADER */
  pointToReach.target_pose.pose.position.x = msg->pose.pose.position.x; 
  pointToReach.target_pose.pose.position.y = msg->pose.pose.position.y; 
  pointToReach.target_pose.pose.position.z = msg->pose.pose.position.z;
  pointToAdd.target_pose.header.frame_id = "map"; //Global Frame

  headerSequencer += 1;
  goals.at(currentGoal).target_pose.header.seq = headerSequencer;
  goals.at(currentGoal).target_pose.header.stamp = ros::Time::now();

  ac.sendGoal(goals.at(currentGoal));
  ROS_INFO_STREAM("Travelling Towards Goal " << currentGoal << " with YAW " << goalPose.at(currentGoal));

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO_STREAM("Goal " << currentGoal << " Reached!");
  } else {
    ROS_INFO_STREAM("ERROR: Failed to Reach Goal, BREAKING THE LOOP");
    break;
  }

}


int main(int argc, char** argv){

  ros::init(argc, argv, "chaser_brain");
  ROS_INFO_STREAM("STARTING chaser_brain...");
  ros::NodeHandle nh;

  /* Tell the action client(ac) that we want to spin a thread by default */
  //Connects to move_base action client created by move_base node in amcl package
  //Allows us to know when goal is reached
  MoveBaseClient ac("tb3_chaser/move_base", true);

  /* Wait for the action server to come up */
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
    if(!ros::ok()){return 0;}
  }

  unsigned int headerSequencer = 0;

  //Subscriber runs whenever a new message arrives
  //50 means buffer, stores missed topic readings
  //Small buffer means that the message read will be closest to latest
  ros::Subscriber sub = nh.subscribe("tb3/leader/odom", 10, chaserCallback);

  //blocking call, allows subscribers to run and get callbacks and stuff
  //ends when cntrl+c is detected
  ros::spin();

  ros::shutdown();

  return 0;
}