#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

/* This is designed to repeatedly drive a singular TB3 to four corners of the map */

int main(int argc, char** argv){

  ros::init(argc, argv, "nav_test");
  ROS_INFO_STREAM("STARTING nav_test...");
  ROS_INFO_STREAM("STARTING nav_test...");
  ROS_INFO_STREAM("STARTING nav_test...");

  /* (Optional) Start all the required packages and nodes using senario1.launch file */
  // look at how turtlebot3_navigation.launch does things, especially for amcl.launch

  /* Tell the action client(ac) that we want to spin a thread by default */
  //Connects to move_base action client created by move_base node
  //Allows us to know when goal is reached
  MoveBaseClient ac("move_base", true); 

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

  // Values in .launch for multi
  // <arg name="first_tb3_x_pos" default="-2.0"/>
  // <arg name="first_tb3_y_pos" default=" -0.5"/>
  // <arg name="first_tb3_z_pos" default=" 0.0"/>
  // <arg name="first_tb3_yaw"   default=" 0.0"/>

  // <arg name="second_tb3_x_pos" default=" 2.0"/>
  // <arg name="second_tb3_y_pos" default=" 0.5"/>
  // <arg name="second_tb3_z_pos" default=" 0.0"/>
  // <arg name="second_tb3_yaw"   default=" 0.0"/>

/*
     if(currentGoal == 1){
      //Goal 1 [Bottom Left]:
      // X: -1.55, Y: 1.5, YAW = 1.5567947 or roughly 90 degs //Facing Left
      goalToSend.target_pose.pose.position.x = -1.5;
      goalToSend.target_pose.pose.position.y = 1.6;
      myQuaternion.setRPY(0,0,45*(180/M_PI));
      goalToSend.target_pose.pose.orientation.z = myQuaternion.z(); //0.7021392
      goalToSend.target_pose.pose.orientation.w = myQuaternion.w(); //0.7120397
      currentGoal = 2;
      ROS_INFO("Goal 1 SET");
    } else if (currentGoal == 2){
      //Goal 2 [Top Left]:
      // X: 1.45, Y: 1.6, YAW = 0 //Facing up
      goalToSend.target_pose.pose.position.x = 1.5;
      goalToSend.target_pose.pose.position.y = 1.6;
      myQuaternion.setRPY(0,0,-45*(180/M_PI));
      goalToSend.target_pose.pose.orientation.z = myQuaternion.z(); //0.0
      goalToSend.target_pose.pose.orientation.w = myQuaternion.w(); //0.0
      currentGoal = 3;
      ROS_INFO("Goal 2 SET");
    } else if (currentGoal == 3){
      //Goal 3 [Top Right]:
      // X: 1.45, Y: -1.6, YAW: -1.4911555 or roughly -90 degs //Facing Right
      goalToSend.target_pose.pose.position.x = 1.5;
      goalToSend.target_pose.pose.position.y = -1.6;
      myQuaternion.setRPY(0,0,-135*(180/M_PI));
      goalToSend.target_pose.pose.orientation.z = myQuaternion.z(); //-0.6783964
      goalToSend.target_pose.pose.orientation.w = myQuaternion.w(); //0.7346961
      currentGoal = 4;
      ROS_INFO("Goal 3 SET");
    } else {
      //Goal 4 [Bottom Right]:
      // X: -1.55, Y: -1.5, YAW: -3.1415927 or roughly -180 degs //Facing Down
      goalToSend.target_pose.pose.position.x = -1.5;
      goalToSend.target_pose.pose.position.y = -1.6;
      // goalToSend.target_pose.pose.orientation.z = -1;
      // goalToSend.target_pose.pose.orientation.w = 0.0;
      myQuaternion.setRPY(0,0,135*(180/M_PI));
      goalToSend.target_pose.pose.orientation.z = myQuaternion.z(); //-1.0
      goalToSend.target_pose.pose.orientation.w = myQuaternion.w(); //0.0
      currentGoal = 1;
      ROS_INFO("Goal 4 SET");
    }
*/

/* To get this to work I'm running from the following given packages:
TURTLEBOT3:
http://wiki.ros.org/turtlebot3_gazebo - Gazebo simulation package for the TurtleBot3
  Subscribed Topics: (Stuff you can send it, 1-way)
  - scan (sensor_msgs/LaserScan): Subscribe scan data
  - joint_states (sensor_msgs/JointState): Subscribe wheel joint states
  Published Topics: (Stuff you hear from it, 1-way)
  - cmd_vel (geometry_msgs/Twist): Publish velocity to TurtleBot3
  
http://wiki.ros.org/turtlebot3_fake - Class that creates and updates TB3 values
  Subscribed Topics: (Stuff you can send it, 1-way)
  - cmd_vel (geometry_msgs/Twist): Control the translational and rotational speed of the robot unit in m/s, rad/s.
  Published Topics: (Stuff you hear from it, 1-way)
  - joint_states (sensor_msgs/JointState): The state of a set of torque controlled joints.
  - magnetic_field (sensor_msgs/MagneticField): Measurement of the Magnetic Field vector at a specific location.
  - tf (tf2_msgs/tfMessage): Contains the coordinate transformation such as base_footprint and odom.

http://wiki.ros.org/turtlebot3_simulations - ROS packages for the turtlebot3 simulation (meta package)
  Subscribed Topics: (Stuff you can send it, 1-way)
  - a
  Published Topics: (Stuff you hear from it, 1-way)
  - a

NAVIGATION:
http://wiki.ros.org/amcl - amcl is a probabilistic localization system for a robot moving in 2D. 
It implements the adaptive (or KLD-sampling) Monte Carlo localization approach (as described by Dieter Fox), 
which uses a particle filter to track the pose of a robot against a known map.
  Subscribed Topics: (Stuff you can send it, 1-way)
  - scan (sensor_msgs/LaserScan): Laser scans.
  - tf (tf/tfMessage): Transforms.
  - initialpose (geometry_msgs/PoseWithCovarianceStamped): Mean and covariance with which to (re-)initialize the particle filter.
  - map (nav_msgs/OccupancyGrid): When the use_map_topic parameter is set, AMCL subscribes to this topic to retrieve the map used for laser-based localization. New in navigation 1.4.2.
  Published Topics: (Stuff you hear from it, 1-way)
  - amcl_pose (geometry_msgs/PoseWithCovarianceStamped): Robot's estimated pose in the map, with covariance. -GLOBAL POSE?
  - particlecloud (geometry_msgs/PoseArray): The set of pose estimates being maintained by the filter.
  - tf (tf/tfMessage): Publishes the transform from odom (which can be remapped via the ~odom_frame_id parameter) to map. -LOCAL POSE?
  Services: (Stuff you can ask from it, 2-way)
  - global_localization (std_srvs/Empty): Initiate global localization, wherein all particles are dispersed randomly through the free space in the map.
  - request_nomotion_update (std_srvs/Empty): Service to manually perform update and publish updated particles.
  - set_map (nav_msgs/SetMap): Service to manually set a new map and pose.
  Useful Parameters: (Variables you can set - curated)
  - ~initial_pose_x (double, default: 0.0 meters): Initial pose mean (x), used to initialize filter with Gaussian distribution.
  - ~initial_pose_y (double, default: 0.0 meters): Initial pose mean (y), used to initialize filter with Gaussian distribution.
  - ~initial_pose_a (double, default: 0.0 radians): Initial pose mean (yaw), used to initialize filter with Gaussian distribution.
  - ~initial_cov_xx (double, default: 0.5*0.5 meters): Initial pose covariance (x*x), used to initialize filter with Gaussian distribution.
  - ~initial_cov_yy (double, default: 0.5*0.5 meters): Initial pose covariance (y*y), used to initialize filter with Gaussian distribution.
  - ~initial_cov_aa (double, default: (π/12)*(π/12) radian): Initial pose covariance (yaw*yaw), used to initialize filter with Gaussian distribution.
  - ~first_map_only (bool, default: false): When set to true, AMCL will only use the first map it subscribes to, rather than updating each time a new one is received. New in navigation 1.4.2

  http://wiki.ros.org/move_base - The move_base package provides an implementation of an action (see the actionlib package) that, 
  given a goal in the world, will attempt to reach it with a mobile base. The move_base node links together a global and local planner to 
  accomplish its global navigation task. It supports any global planner adhering to the nav_core::BaseGlobalPlanner interface specified in 
  the nav_core package and any local planner adhering to the nav_core::BaseLocalPlanner interface specified in the nav_core package. 
  The move_base node also maintains two costmaps, one for the global planner, and one for a local planner (see the costmap_2d package) that are used to accomplish navigation tasks.
    Node: (A process that performs computation, connects to others through topics, services, actions...)
    - move_base: Good diagram on website. The move_base node provides a ROS interface for configuring, running, and interacting with the navigation stack on a robot.
      Running the move_base node on a robot that is properly configured (please see navigation stack documentation for more details) results in a robot that will 
      attempt to achieve a goal pose with its base to within a user-specified tolerance.
      (More details on wiki)
    Subscribed Topics: (Stuff you can send it, 1-way)
    - move_base_simple/goal (geometry_msgs/PoseStamped): Provides a non-action interface to move_base for users that don't care about tracking the execution status of their goals.
    Published Topics: (Stuff you hear from it, 1-way)
    - cmd_vel (geometry_msgs/Twist): A stream of velocity commands meant for execution by a mobile base.
    Services: (Stuff you can ask from it, 2-way)
    - ~make_plan (nav_msgs/GetPlan): Allows an external user to ask for a plan to a given pose from move_base without causing move_base to execute that plan.
    - ~clear_unknown_space (std_srvs/Empty): Allows an external user to tell move_base to clear unknown space in the area directly around the robot. 
      This is useful when move_base has its costmaps stopped for a long period of time and then started again in a new location in the environment.
    - ~clear_costmaps (std_srvs/Empty): Allows an external user to tell move_base to clear obstacles in the costmaps used by move_base. 
      This could cause a robot to hit things and should be used with caution.
*/

/* Planner */
//turtlebot3_fake.h has all the values like max angular rotation and stuff

/* AMCL (localisation) & Gmapping (map building) */
// Can be tweaked by modifying parameters on launch/includes/_amcl.launch file 
// and launch/includes/_gmapping.launch