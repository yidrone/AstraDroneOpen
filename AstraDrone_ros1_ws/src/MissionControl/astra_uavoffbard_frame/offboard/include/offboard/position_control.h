#ifndef _POSITION_CONTROLLER_NEW_H_
#define _POSITION_CONTROLLER_NEW_H_

#include <map>
#include <mavros_msgs/SetMode.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>


class DroneControl{
public:
    
    /** constructor **/
    DroneControl(ros::NodeHandle nh);
    /** destructor **/
    ~DroneControl();

    void DroneControl_main();
    void ReadParams();

private:

    ros::NodeHandle nh_;

    //goal position receive topic 
    ros::Subscriber position_sub;

    //drone control pubilsh topic 
    ros::Publisher local_pos_pub;

    //init goal
    geometry_msgs::PoseStamped goal_position;

    /**
    * @brief    receive new goal's position, then goto this CallBack
    * @param[in]  msg, position and oritentation
    **/
    void position_CallBack(const geometry_msgs::PoseStamped& msg);
};


#endif

