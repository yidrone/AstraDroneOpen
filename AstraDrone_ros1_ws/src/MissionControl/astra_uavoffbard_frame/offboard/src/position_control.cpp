#include <ros/ros.h>
#include <offboard/position_control.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle NodeHandle;

    DroneControl Drone_Control(NodeHandle);   

    ros::spin();
    return 0;
}

