/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Taehun Lim (Darby) */
/* Modularity Edits by Ashwin Coburn for Collaborative Autonomy */
/* From Turtlebot3's Gazebo Package */

#include "multi_drive.h"

#define DEBUG true;

Turtlebot3Drive::Turtlebot3Drive()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  auto ret = init();
  ROS_ASSERT(ret);
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Drive::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");
  /* ADDED */
  std::string scan_topic_name = nh_.param<std::string>("scan_topic_name", "");
  std::string odom_topic_name = nh_.param<std::string>("odom_topic_name", "");

  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.7;
  check_side_dist_    = 0.6;

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // initialize subscribers
  // laser_scan_sub_  = nh_.subscribe("scan", 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
  // odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);
  /* ADDED */
  laser_scan_sub_  = nh_.subscribe(scan_topic_name, 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe(odom_topic_name, 10, &Turtlebot3Drive::odomMsgCallBack, this);

  #ifdef DEBUG
  ROS_INFO_STREAM("Subscribed to " << cmd_vel_topic_name);
  ROS_INFO_STREAM("Subscribed to " << scan_topic_name);
  ROS_INFO_STREAM("Subscribed to " << odom_topic_name);
  #endif

  return true;
}

void Turtlebot3Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  #ifdef DEBUG
  ROS_INFO_STREAM("odomCallback Triggered!");
  #endif

  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	tb3_pose_ = atan2(siny, cosy);
}

void Turtlebot3Drive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{

  #ifdef DEBUG
  ROS_INFO_STREAM("laserScanCallback Triggered!");
  #endif

  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Turtlebot3Drive::updatecommandVelocity(double linear, double angular)
{
  #ifdef DEBUG
  ROS_INFO_STREAM("updatecommandVelocity() Triggered!");
  #endif
  
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  #ifdef DEBUG
  ROS_INFO_STREAM("Publishing on cmd_vel...");
  #endif

  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Turtlebot3Drive::controlLoop()
{
  #ifdef DEBUG
  ROS_INFO_STREAM("controlLoop() Activated...");
  #endif
  static uint8_t turtlebot3_state_num = 0;

  switch(turtlebot3_state_num)
  {
    case GET_TB3_DIRECTION:
      #ifdef DEBUG
      ROS_INFO_STREAM("case 0 selected");
      #endif

      if (scan_data_[CENTER] > check_forward_dist_)
      {
        if (scan_data_[LEFT] < check_side_dist_)
        {
          prev_tb3_pose_ = tb3_pose_;
          turtlebot3_state_num = TB3_RIGHT_TURN;
        }
        else if (scan_data_[RIGHT] < check_side_dist_)
        {
          prev_tb3_pose_ = tb3_pose_;
          turtlebot3_state_num = TB3_LEFT_TURN;
        }
        else
        {
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
        }
      }

      if (scan_data_[CENTER] < check_forward_dist_)
      {
        prev_tb3_pose_ = tb3_pose_;
        turtlebot3_state_num = TB3_RIGHT_TURN;
      }
      break;

    case TB3_DRIVE_FORWARD:
      #ifdef DEBUG
      ROS_INFO_STREAM("case 1 selected");
      #endif
      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_RIGHT_TURN:
      #ifdef DEBUG
      ROS_INFO_STREAM("case 2 selected");
      #endif
      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
      break;

    case TB3_LEFT_TURN:
      #ifdef DEBUG
      ROS_INFO_STREAM("case 3 selected");
      #endif
      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0, ANGULAR_VELOCITY);
      break;

    default:
      #ifdef DEBUG
      ROS_INFO_STREAM("case default selected");
      #endif
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  #ifdef DEBUG
  ROS_INFO_STREAM("STARTING multi_drive...");
  #endif
  ros::init(argc, argv, "multi_drive");
  Turtlebot3Drive turtlebot3_drive;

  ros::Rate loop_rate(125);

  #ifdef DEBUG
  ROS_INFO_STREAM("Entering Loop...");
  #endif

  // ADDED WATCH OUT ADDED WATCH OUT ADDED WATCH OUT ADDED WATCH OUT ADDED WATCH OUT
      while(ros::ok()){ //remove me when ready
      }
// ADDED WATCH OUT ADDED WATCH OUT ADDED WATCH OUT ADDED WATCH OUT ADDED WATCH OUT

  while (ros::ok())
  {
    turtlebot3_drive.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
