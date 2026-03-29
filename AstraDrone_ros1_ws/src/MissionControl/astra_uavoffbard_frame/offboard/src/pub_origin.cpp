#include <ros/ros.h>
#include <geographic_msgs/GeoPointStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_origin");
    ros::NodeHandle nh("~");

    ros::Publisher gp_origin_pub = nh.advertise<geographic_msgs::GeoPointStamped>("/mavros/global_position/gp_origin", 10);

    ros::Rate rate(10.0);

    // Read parameters 
    double latitude, longitude, altitude;
    nh.param<double>("latitude", latitude, 0.0);
    nh.param<double>("longitude", longitude, 0.0);
    nh.param<double>("altitude", altitude, 0.0);

    geographic_msgs::GeoPointStamped gp_origin;
    gp_origin.header.stamp = ros::Time::now();
    gp_origin.header.frame_id = "map";
    gp_origin.position.latitude = latitude;
    gp_origin.position.longitude = longitude;
    gp_origin.position.altitude = altitude;

    while (ros::ok()) {
        gp_origin_pub.publish(gp_origin);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}