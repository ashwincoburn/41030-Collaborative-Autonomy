#ifndef CHASER_BRAIN_H
#define CHASER_BRAIN_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include "nav_msgs/Odometry.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

class Chaser_Brain
{
public:
    Chaser_Brain(ros::NodeHandle nh);

    ~Chaser_Brain();

    void chaserOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    void leaderOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    void chaserThread(void);

    bool closeProximity(void);

    void killThreads(void);

private:

    void sendNewGoal(void);

    ros::NodeHandle nh_;           // Node handle for communication
    MoveBaseClient ac_;            // move_base action client
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;

    unsigned int headerSequencer_; // Action Client Sequencer
    move_base_msgs::MoveBaseGoal pointToReach_; //XYZ point to reach
    move_base_msgs::MoveBaseGoal currentPosition_; //Current Location
    mutex chaserMutex_; //For accessing shared chaser's data
    mutex leaderMutex_; //For accessing shared leader's data
    atomic<bool> runLoop_; //For breaking while loop of chaser odom and leader odom threads
    atomic<bool> dataCollected_; //Controls waiting for odom data from leader
    double robotsDistTolerance_; //Scalar distance tolerance that stops chaser from getting any closer
    double robotsDistance_; //Scalar distance between robots
};

#endif
