#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

/* This is designed to repeatedly drive a singular TB3 to four corners of the map */

int main(int argc, char** argv){

  ros::init(argc, argv, "sen2_node_leader");
  ROS_INFO_STREAM("STARTING Senario 2: Leader...");

  /* (Optional) Start all the required packages and nodes using senario1.launch file */
  // look at how turtlebot3_navigation.launch does things, especially for amcl.launch

  /* Tell the action client(ac) that we want to spin a thread by default */
  //Connects to move_base action client created by move_base node in amcl package
  //Allows us to know when goal is reached
  MoveBaseClient ac("tb3_leader/move_base", true); 

  /* Wait for the action server to come up */
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
  }

  /* Define the vector of goals */
  vector<move_base_msgs::MoveBaseGoal> goals;
  move_base_msgs::MoveBaseGoal pointToAdd;
  tf2::Quaternion myQuaternion;
  vector<double> goalPose;

  //Goal 0, Bottom Left, 90 degs
  pointToAdd.target_pose.pose.position.x = -1.55; 
  pointToAdd.target_pose.pose.position.y = 1.5; 
  pointToAdd.target_pose.pose.position.z = 0.0;
  myQuaternion.setRPY(0,0,90*(180/M_PI));
  pointToAdd.target_pose.pose.orientation.x = myQuaternion.getX();
  pointToAdd.target_pose.pose.orientation.y = myQuaternion.getY();
  pointToAdd.target_pose.pose.orientation.z = myQuaternion.getZ();
  pointToAdd.target_pose.pose.orientation.w = myQuaternion.getW();
  pointToAdd.target_pose.header.frame_id = "map"; //Global Frame, base_link is local I think, passes onto other goals
  goals.push_back(pointToAdd);
  goalPose.push_back(90);
  //Goal 1, Top Left, 0 degs
  pointToAdd.target_pose.pose.position.x = 1.45; 
  pointToAdd.target_pose.pose.position.y = 1.6; 
  pointToAdd.target_pose.pose.position.z = 0.0;
  myQuaternion.setRPY(0,0,0*(180/M_PI));
  pointToAdd.target_pose.pose.orientation.x = myQuaternion.getX();
  pointToAdd.target_pose.pose.orientation.y = myQuaternion.getY();
  pointToAdd.target_pose.pose.orientation.z = myQuaternion.getZ();
  pointToAdd.target_pose.pose.orientation.w = myQuaternion.getW();
  goals.push_back(pointToAdd);
  goalPose.push_back(0);
  //Goal 2, Top Right, -90 degs
  pointToAdd.target_pose.pose.position.x = 1.45; 
  pointToAdd.target_pose.pose.position.y = -1.6; 
  pointToAdd.target_pose.pose.position.z = 0.0;
  myQuaternion.setRPY(0,0,-90*(180/M_PI));
  pointToAdd.target_pose.pose.orientation.x = myQuaternion.getX();
  pointToAdd.target_pose.pose.orientation.y = myQuaternion.getY();
  pointToAdd.target_pose.pose.orientation.z = myQuaternion.getZ();
  pointToAdd.target_pose.pose.orientation.w = myQuaternion.getW();
  goals.push_back(pointToAdd);
  goalPose.push_back(-90);
  //Goal 3, Bottom Right, -180 degs
  pointToAdd.target_pose.pose.position.x = -1.55; 
  pointToAdd.target_pose.pose.position.y = -1.5; 
  pointToAdd.target_pose.pose.position.z = 0.0;
  myQuaternion.setRPY(0,0,-179*(180/M_PI));
  pointToAdd.target_pose.pose.orientation.x = myQuaternion.getX();
  pointToAdd.target_pose.pose.orientation.y = myQuaternion.getY();
  pointToAdd.target_pose.pose.orientation.z = myQuaternion.getZ();
  pointToAdd.target_pose.pose.orientation.w = myQuaternion.getW();
  goals.push_back(pointToAdd);
  goalPose.push_back(-179);

  /* Loop Goals */
  unsigned int headerSequencer = 0;
  unsigned short currentGoal = 0;

  while(ros::ok()){
 
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

    if(currentGoal < goals.size() - 1){
      currentGoal += 1;
    } else {currentGoal = 0;}
    ROS_INFO_STREAM("New currentGoal = " << currentGoal << " for goals.size() = " << goals.size());
  }

  return 0;
}