#include "chaser_brain.h"
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
// Write a class that saves odom data, thread that checks for changes in the data and sends updates

Chaser_Brain::Chaser_Brain(ros::NodeHandle nh,MoveBaseClient ac) : 
nh_(nh), ac_(*ac_ptr), headerSequencer_(0), runLoop_(true), dataCollected_(false), robotsDistTolerance_(2.0),
robotsDistance_(100.0) 
{
  // Subscriber runs whenever a new message arrives
  // 50 means buffer, stores missed topic readings
  // Small buffer means that the message read will be closest to latest
  sub1_ = nh_.subscribe("tb3/leader/odom", 10, leaderOdomCallback);
  sub2_ = nh_.subscribe("tb3/chaser/odom", 10, chaserOdomCallback);
  {
  unique_lock<mutex> lck(chaserMutex_);
  pointToReach_.target_pose.pose.position.z = 0;
  pointToReach_.target_pose.header.frame_id = "map"; // Global Frame
  }
  
}

Chaser_Brain::~Chaser_Brain()
{
}

void Chaser_Brain::leaderOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  /*
    Non-Blocking Function, designed to run once and fast when it's activated by new data
    Simply updates the current location of LEADER
  */
  {
  unique_lock<mutex> lck(leaderMutex_);
  pointToReach_.target_pose.pose.position.x = msg->pose.pose.position.x;
  pointToReach_.target_pose.pose.position.y = msg->pose.pose.position.y;
  }
  if(!dataCollected_){dataCollected_ = true;}
}

void Chaser_Brain::chaserOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  /*
    Non-Blocking Function, designed to run once and fast when it's activated by new data
    Simply updates the current location of CHASER
  */
  {
  unique_lock<mutex> lck(chaserMutex_);
  currentPosition_.target_pose.pose.position.x = msg->pose.pose.position.x;
  currentPosition_.target_pose.pose.position.y = msg->pose.pose.position.y;
  }
}

bool Chaser_Brain::closeProximity(void){

  {
  unique_lock<mutex> lck(chaserMutex_);
  unique_lock<mutex> lck(leaderMutex_);
  robotsDistance_ = abs(sqrt(pow(pointToReach_.target_pose.pose.position.x-currentPosition_.target_pose.pose.position.x,2)
    +pow(pointToReach_.target_pose.pose.position.y-currentPosition_.target_pose.pose.position.y,2)));
  }

  if(robotsDistance_ <= robotsDistTolerance_){
    return true;
  } else {
    return false;
  }
}

void Chaser_Brain::killThreads(void){
  ROS_INFO_STREAM("Killing Threads...");
  runLoop_ = false;
}

void Chaser_Brain::sendNewGoal(void){

  ROS_INFO_STREAM("Travelling Towards Goal (" << pointToReach_.target_pose.pose.position.x << 
  "," << pointToReach_.target_pose.pose.position.y << ")...");
  
  {
  unique_lock<mutex> lck(leaderMutex_);
  ac_.cancelGoal();
  
  headerSequencer_ += 1;
  pointToReach_.target_pose.header.seq = headerSequencer_;
  pointToReach_.target_pose.header.stamp = ros::Time::now();

  ac_.sendGoal(pointToReach_);
  }
}

void Chaser_Brain::chaserThread(void)
{
  /*
    Blocking function, designed to run in a seperate thread
    Checks for new odometry and sends new goal request, Low Loop Rate
  */

  move_base_msgs::MoveBaseGoal previousTarget;

  // Wait for readings to come in
  while (!dataCollected_){}

  previousTarget = pointToReach_; //Means we'll miss inital spawn but we need a value

  while (runLoop_){
    // if(closeProximity() == true && ac_.getState() == ACTIVE){
    if(closeProximity() == true){
      
      ac_.cancelAllGoals();
    
    } else if (previousTarget.target_pose.pose.position.x != currentPosition_.target_pose.pose.position.x ||
      previousTarget.target_pose.pose.position.y != currentPosition_.target_pose.pose.position.y){

      sendNewGoal();
    
    }
    //ros::Rate is a better alternative, but for now just use this
    std::this_thread::sleep_for (std::chrono::milliseconds(100));
  }
}