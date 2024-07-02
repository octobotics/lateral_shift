#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "hector_mapping/ResetMapping.h"
#include "std_msgs/Int32.h"
#include "ros/callback_queue.h"
#include <chrono>
#include <thread>

ros::Publisher cmdVelPublisher;
ros::ServiceClient resetServiceClient;
ros::CallbackQueue serviceCallbackQueue;
ros::Time resetStartTime;
bool executeSecondIfStatement = false;
ros::Time endTime;
bool resetCompleted = false;

void resetTracking() {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<hector_mapping::ResetMapping>("/restart_mapping_with_new_pose");
    hector_mapping::ResetMapping srv;

    srv.request.initial_pose.position.x = 0.0;
    srv.request.initial_pose.position.y = 0.0;
    srv.request.initial_pose.position.z = 0.0;
    srv.request.initial_pose.orientation.x = 0.0;
    srv.request.initial_pose.orientation.y = 0.0;
    srv.request.initial_pose.orientation.z = 0.0;
    srv.request.initial_pose.orientation.w = 1.0;

    if (client.call(srv)) {
        ROS_INFO("restart_mapping_with_new_pose service called successfully");
    } else {
        ROS_ERROR("Failed to call restart_mapping_with_new_pose service");
    }
}

void publishCmdVel() {
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = -5.0;

    cmdVelPublisher.publish(cmd_vel_msg);
    ROS_INFO("Published on /skid_steer/cmd_vel");
}

void resetServiceCallback(const std_msgs::Int32::ConstPtr& response) {
    if (response->data == 1) {
        ROS_INFO("restart_mapping_with_new_pose service called successfully");
    } else {
        ROS_ERROR("Failed to call restart_mapping_with_new_pose service");
    }
}

void checkOrientation(const geometry_msgs::PoseStamped::ConstPtr& orientation_msg) {
    static ros::Time resetStartTime = ros::Time::now();
    static ros::Time endTime = resetStartTime + ros::Duration(7.0);
    static bool orientationConditionMet = false;

    if (orientation_msg->pose.orientation.z >= 45.0 && orientation_msg->pose.orientation.z <= 48.0) {
        if (!orientationConditionMet) {
            ROS_INFO("pose.orientation.z is between 45 and 48");
            hector_mapping::ResetMapping srv;

            srv.request.initial_pose.position.x = 0.0;
            srv.request.initial_pose.position.y = 0.0;
            srv.request.initial_pose.position.z = 0.0;
            srv.request.initial_pose.orientation.x = 0.0;
            srv.request.initial_pose.orientation.y = 0.0;
            srv.request.initial_pose.orientation.z = 0.0;
            srv.request.initial_pose.orientation.w = 1.0;

            if (resetServiceClient.call(srv)) {
                geometry_msgs::Twist cmd_vel_msg;
                cmd_vel_msg.linear.x = 2.0;
                cmd_vel_msg.linear.y = 0.0;
                cmd_vel_msg.linear.z = 0.0;
                cmd_vel_msg.angular.x = 0.0;
                cmd_vel_msg.angular.y = 0.0;
                cmd_vel_msg.angular.z = 0.0;

                cmdVelPublisher.publish(cmd_vel_msg);
                ROS_INFO("Published on /skid_steer/cmd_vel");

                orientationConditionMet = true;
            }
        }
    }

    if (orientationConditionMet && (ros::Time::now() < endTime) && (orientation_msg->pose.position.x <= -0.40)) {
        ROS_INFO("Received orientation message. position.x: %f", orientation_msg->pose.position.x);
        ROS_INFO("Sufficient time has elapsed since restart_mapping_with_new_pose service was called");

        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.0;
        vel_msg.linear.y = 0.0;
        vel_msg.linear.z = 0.0;
        vel_msg.angular.x = 0.0;
        vel_msg.angular.y = 0.0;
        vel_msg.angular.z = 5.0;

        cmdVelPublisher.publish(vel_msg);
        ROS_INFO("Position is less than -0.40");

        ros::NodeHandle nh;
        ros::ServiceClient client = nh.serviceClient<hector_mapping::ResetMapping>("/restart_mapping_with_new_pose");
        hector_mapping::ResetMapping srver;

        srver.request.initial_pose.position.x = 0.0;
        srver.request.initial_pose.position.y = 0.0;
        srver.request.initial_pose.position.z = 0.0;
        srver.request.initial_pose.orientation.x = 0.0;
        srver.request.initial_pose.orientation.y = 0.0;
        srver.request.initial_pose.orientation.z = 0.0;
        srver.request.initial_pose.orientation.w = 1.0;

        client.call(srver);
    }

    if (orientation_msg->pose.orientation.z >= 130.0 && orientation_msg->pose.orientation.z <= 135.0) {
        geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;

        cmdVelPublisher.publish(msg);
        ROS_INFO("Published on /skid_steer/cmd_vel");
        resetCompleted = true;
    }
}

void topicCallback(const std_msgs::Int32::ConstPtr& msg) {
    if (msg->data == 1) {
        ROS_INFO("Received 1 on /lateral_shift");
        resetTracking();
        publishCmdVel();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lateral_shift_controller");
    ros::NodeHandle nh;
    resetServiceClient = nh.serviceClient<hector_mapping::ResetMapping>("/restart_mapping_with_new_pose");
    ros::Subscriber sub_response = nh.subscribe("/reset_mapping_response", 1, resetServiceCallback);
    ros::Subscriber sub = nh.subscribe("/lateral_shift", 10, topicCallback);
    ros::Subscriber sub_orientation = nh.subscribe("/position_yaw", 10, checkOrientation);

    cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("/skid_steer/cmd_vel", 10);

    ROS_INFO("Initialization complete. Entering main loop...");

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
