#include "ros/ros.h"
#include "chaser_brain.h"
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <thread>
#include "nav_msgs/Odometry.h"

using namespace std;

std::unique_ptr<MoveBaseClient> ac_ptr;

int main(int argc, char **argv)
{

    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line. For programmatic
     * remappings you can use a different version of init() which takes remappings
     * directly, but for most command-line programs, passing argc and argv is the easiest
     * way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     *
     * The third argument below "week10_quad" is the rosnode name
     * The name must be unique, only one node of the same name can ever register with the roscore
     * If a rosnode with same name exists, it will be terminated
     */
    ros::init(argc, argv, "chaser_brain");
    ROS_INFO_STREAM("STARTING chaser_brain...");

    /* Tell the action client(ac) that we want to spin a thread by default */
    // Connects to move_base action client created by move_base node in amcl package
    // Allows us to know when goal is reached
    MoveBaseClient ac("tb3_chaser/move_base", true);
    //added:
    ac_ptr = std::make_unique<MoveBaseClient>(std::move(ac));

    /* Wait for the action server to come up */
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO_STREAM("Waiting for the move_base action server to come up");
        if (!ros::ok()){return 0;}
    }

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle nh;

    /**
     * Create an object of type Chaser_Brain and pass it a node handle
     */
    // std::shared_ptr<Chaser_Brain> ChaserBrainPtr(new Chaser_Brain(nh, ac));
    std::shared_ptr<Chaser_Brain> ChaserBrainPtr(new Chaser_Brain(nh, std::move(ac_ptr)));

    /**
     * Let's start seperate thread first, to do that we need to create object
     * and thereafter start the thread on the function we desire
     */
    std::thread chaserThread(&Chaser_Brain::chaserThread, ChaserBrainPtr);

    // blocking call, allows subscribers to run and get callbacks and stuff
    // ends when cntrl+c is detected
    ros::spin();

    Chaser_Brain::killThreads();

    ros::shutdown();

    chaserThread.join();

    return 0;
}