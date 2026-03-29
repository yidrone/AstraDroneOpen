#include <ros/ros.h>
#include <offboard/position_control.h>
#include <geometry_msgs/PoseStamped.h>

//构造函数，在另一个文件position_control.cpp中定义了一个对象，然后进入这里
DroneControl::DroneControl(ros::NodeHandle nh):nh_(nh) {   
    std::cout << "enter constructor function" << std::endl;
    //进入真正的主函数
    DroneControl_main();
}
//析构函数
DroneControl::~DroneControl()    {}

//main
void DroneControl::DroneControl_main()
{
    //defind pub and sub
    position_sub = nh_.subscribe<const geometry_msgs::PoseStamped&>("/drone_control/goal_position", 10, &DroneControl::position_CallBack, this);
    local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    //main loop
    while(ros::ok()){

		//echo goal_position ever 3 second
		ROS_INFO_THROTTLE(3,"goal(x,y,z) : %f, %f, %f",goal_position.pose.position.x,goal_position.pose.position.y,goal_position.pose.position.z);

    	local_pos_pub.publish(goal_position);
        ros::spinOnce();

    }
}

//read launch params
void DroneControl::ReadParams()
    {
        nh_.getParam("/goal_init_x", goal_position.pose.position.x);
        nh_.getParam("/goal_init_y",  goal_position.pose.position.y);
        nh_.getParam("/goal_init_z",  goal_position.pose.position.z);
    }


void DroneControl::position_CallBack(const geometry_msgs::PoseStamped& msg){
	
    //set frame_id to local_origin
	// goal_position.header.frame_id = "local_origin";
	
    //set position
    goal_position.pose.position.x = msg.pose.position.x;
    goal_position.pose.position.y = msg.pose.position.y;
    goal_position.pose.position.z = msg.pose.position.z;

}