#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <zed_interfaces/reset_tracking.h>

bool isOrientationInRange = false;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg)
{
    // Check if pose.orientation.z is between 89 and 92
    double z_orientation = poseMsg->pose.orientation.z;
    if (z_orientation >= 89.0 && z_orientation <= 92.0)
    {
        isOrientationInRange = true;
    }
    else
    {
        isOrientationInRange = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lateral_shift_controller");
    ros::NodeHandle nh;

    // Subscribe to /zed_orientation topic
     // Subscribe to /zed_orientation topic
    ros::Subscriber poseSubscriber = nh.subscribe("/zed_orientation", 10, poseCallback);

    // Call /zed2i/zed_node/reset_tracking service
    ros::ServiceClient resetTrackingClient = nh.serviceClient<zed_interfaces::reset_tracking>("/zed2i/zed_node/reset_tracking");
    zed_interfaces::reset_tracking resetTrackingSrv;

    // Add a delay for node initialization
    ros::Duration(1.0).sleep();

    // Reset tracking
    if (resetTrackingClient.call(resetTrackingSrv))
    {
        ROS_INFO("Tracking reset successful: true");
    }
    else
    {
        ROS_ERROR("Failed to call reset_tracking service.");
        return 1;
    }

    // Publish a Twist message to /skid_steer/cmd_vel
    ros::Publisher cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("/skid_steer/cmd_vel", 10);

    // Publish the Twist message repeatedly
    ros::Rate loop_rate(1);  // Adjust the publishing rate as needed
    while (ros::ok())
    {
        geometry_msgs::Twist cmdVelMsg;

        if (isOrientationInRange)
        {
            // If orientation is in range, publish a new Twist message with zeros
            cmdVelMsg.linear.x = 0.0;
            cmdVelMsg.linear.y = 0.0;
            cmdVelMsg.linear.z = 0.0;
            cmdVelMsg.angular.x = 0.0;
            cmdVelMsg.angular.y = 0.0;
            cmdVelMsg.angular.z = 0.0;
        }
        else
        {
            // If orientation is not in range, use your original Twist message with -15.0 angular.z
            cmdVelMsg.linear.x = 0.0;
            cmdVelMsg.linear.y = 0.0;
            cmdVelMsg.linear.z = 0.0;
            cmdVelMsg.angular.x = 0.0;
            cmdVelMsg.angular.y = 0.0;
            cmdVelMsg.angular.z = -15.0;
        }

        cmdVelPublisher.publish(cmdVelMsg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}