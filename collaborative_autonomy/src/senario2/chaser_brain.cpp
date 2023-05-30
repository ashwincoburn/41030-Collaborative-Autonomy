#include "chaser_brain.h"
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread>
#include "nav_msgs/Odometry.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

#define DEBUG true;

/* This is designed to chase the LEADER TB3 */
/* Simple chase algorithm that stops chasing if within proximity of LEADER */
/* As we have a subscriber, looks like we'll need ros::spin() */
// http://wiki.ros.org/evarobot_odometry/Tutorials/indigo/Writing%20a%20Simple%20Subscriber%20for%20Odometry
// https://answers.ros.org/question/259418/sending-goals-to-navigation-stack-using-code/
// <node name="chaser_brain" pkg="collaborative_autonomy"  type="chaser_brain" output="screen" launch-prefix="gnome-terminal --command"/>
// Write a class that saves odom data, thread that checks for changes in the data and sends updates

Chaser_Brain::Chaser_Brain(ros::NodeHandle nh) : 
nh_(nh), ac_("tb3_chaser/move_base", true), headerSequencer_(0), runLoop_(true), dataCollected_(false), 
robotsDistTolerance_(0.90), robotsDistance_(100.0), allowCollectData_(true)
{
  /* Wait for the action server to come up */
  while (!ac_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
  }
  ROS_INFO_STREAM("move_base action server is up");

  pointToReach_.target_pose.pose.position.z = 0;
  pointToReach_.target_pose.header.frame_id = "map"; // Global Frame

  // Subscriber runs whenever a new message arrives
  // 10 means buffer, stores missed topic readings
  // Small buffer means that the message read will be closest to latest
  sub1_ = nh_.subscribe("tb3_leader/odom", 10, &Chaser_Brain::leaderOdomCallback,this);
  sub2_ = nh_.subscribe("tb3_chaser/odom", 10, &Chaser_Brain::chaserOdomCallback,this);
}

Chaser_Brain::~Chaser_Brain()
{
}

void Chaser_Brain::leaderOdomCallback(const nav_msgs::Odometry::ConstPtr &msg) /// look for /nav_msgs/Odometry
{
  /*
    Non-Blocking Function, designed to run once and fast when it's activated by new data
    Simply updates the current odom of LEADER
  */

  if(allowCollectData_){
    {
    unique_lock<mutex> lck(dataMutex_);
    pointToReach_.target_pose.pose.position.x = msg->pose.pose.position.x;
    pointToReach_.target_pose.pose.position.y = msg->pose.pose.position.y;
    pointToReach_.target_pose.pose.orientation = msg->pose.pose.orientation;
    }
  }

  if(!dataCollected_){dataCollected_ = true;}
}

void Chaser_Brain::chaserOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  /*
    Non-Blocking Function, designed to run once and fast when it's activated by new data
    Simply updates the current odom of CHASER
  */
  // #ifdef DEBUG
  // ROS_INFO_STREAM("[BRAIN] chaserOdom: (" << msg->pose.pose.position.x << "," << msg->pose.pose.position.y << ")");
  // #endif
  
  if(allowCollectData_){
    {
    unique_lock<mutex> lck(dataMutex_);
    currentPosition_.target_pose.pose.position.x = msg->pose.pose.position.x;
    currentPosition_.target_pose.pose.position.y = msg->pose.pose.position.y;
    }
  }
}

bool Chaser_Brain::closeProximity(void){

  robotsDistance_ = abs(sqrt(pow(pointToReach_.target_pose.pose.position.x-currentPosition_.target_pose.pose.position.x,2)
    +pow(pointToReach_.target_pose.pose.position.y-currentPosition_.target_pose.pose.position.y,2)));

  if(robotsDistance_ <= robotsDistTolerance_){
    #ifdef DEBUG
    ROS_INFO_STREAM("[BRAIN] Close Proximity Detected!");
    #endif
    return true;
  } else {
    return false;
  }
}

void Chaser_Brain::killThreads(void){
  #ifdef DEBUG
  ROS_INFO_STREAM("[BRAIN] Killing Threads...");
  #endif
  runLoop_ = false;
}

void Chaser_Brain::sendNewGoal(void){

  ROS_INFO_STREAM("Travelling Towards Goal (" << pointToReach_.target_pose.pose.position.x << 
  "," << pointToReach_.target_pose.pose.position.y << ")...");
  
  {
  unique_lock<mutex> lck(dataMutex_);
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
    Checks for new odometry and sends new goal request, Slow Loop Rate
  */

  move_base_msgs::MoveBaseGoal previousTarget;

  // Wait for readings to come in
  ros::Rate loop_rate(2);
  while (!dataCollected_){

    #ifdef DEBUG
    ROS_INFO_STREAM("[BRAIN] Waiting for odom data to show...");
    #endif

    if(!ros::ok()){return;}
    
    loop_rate.sleep();
  }

  #ifdef DEBUG
  ROS_INFO_STREAM("[BRAIN] Got odom data, starting logic...");
  #endif

  // loop_rate = ros::Rate(1); //Change Goal every 1s
  ros::Rate loop_rate2(1); //1 Cycle Per Second
  previousTarget = pointToReach_; //Means we'll miss inital spawn but we need a value

  while (runLoop_ || ros::ok()){
    if(!ros::ok()){return;}

    //First check if we are too close and the robot is running - We want to stop
    if(closeProximity() == true && ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE){

      ac_.cancelGoal();
    
    //Next case, check if new XY location is not the same as the last XY location, if new we send a command to go
    } else if (previousTarget.target_pose.pose.position.x != pointToReach_.target_pose.pose.position.x ||
      previousTarget.target_pose.pose.position.y != pointToReach_.target_pose.pose.position.y){

      sendNewGoal();

      #ifdef DEBUG
      ROS_INFO_STREAM("[BRAIN] New Goal Sent!");
      #endif
    
    }

    previousTarget = pointToReach_;
    loop_rate2.sleep();
  }
}