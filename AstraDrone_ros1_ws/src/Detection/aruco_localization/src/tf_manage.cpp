#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msgs::TransformStamped invertTransform(const geometry_msgs::TransformStamped &transform) {
    geometry_msgs::TransformStamped inverse_transform;

    // Swap the frame_id and child_frame_id
    inverse_transform.header.frame_id = "downcamera";
    inverse_transform.child_frame_id = "aruco_localization";

    // Inverse the translation
    inverse_transform.transform.translation.x = -transform.transform.translation.x;
    inverse_transform.transform.translation.y = -transform.transform.translation.y;
    inverse_transform.transform.translation.z = -transform.transform.translation.z;

    // Inverse the rotation
    tf2::Quaternion quat;
    tf2::fromMsg(transform.transform.rotation, quat);
    tf2::Quaternion quat_inverse = quat.inverse();
    inverse_transform.transform.rotation = tf2::toMsg(quat_inverse);

    return inverse_transform;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_manage");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster tfBroadcaster;

    ros::Rate rate(5.0);  // Set the loop frequency

    while (ros::ok()) {
        try {
            // Get the transform from aruco to camera
            geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform("camera", "aruco", ros::Time(0));
            
            // Invert the transform
            geometry_msgs::TransformStamped inverse_transform = invertTransform(transform);
            inverse_transform.header.stamp = ros::Time::now();  // Update the timestamp
            
            // Broadcast the inverse transform
            tfBroadcaster.sendTransform(inverse_transform);

        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        rate.sleep();
    }

    return 0;
}
