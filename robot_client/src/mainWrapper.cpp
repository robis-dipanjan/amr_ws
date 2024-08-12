#include <iostream>
#include <ros/ros.h>
#include "DeviceManager.h"
#include <std_srvs/SetBool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

DeviceManager dm;

float linearVelocity{0.0};
float angularVelocity{0.0};

void setMotorVelocity(const geometry_msgs::Twist::ConstPtr &msg) {
    linearVelocity = msg->linear.x;
    angularVelocity = msg->angular.z;

    dm.motorClient->initializeMotor();


    if (dm.motorClient->faultStates()){
        std::cout << "Fault State Detected" << std::endl;
        dm.diffDrive->setVelocity(0.0, 0.0);
    }
    else{
        dm.diffDrive->setVelocity(linearVelocity, angularVelocity);

    }
}

bool reset_odom(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    if (req.data){
        bool odomResponse = dm.diffDrive->resetOdom();
        res.success = odomResponse;
        res.message = "Odometry reset done";
    }
    else{
        res.success = false;
        res.message = "Invalid command";
    }
    return true;
}

bool reset_error(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    if (req.data){
        bool errorResponse = dm.kincoMotor->resetErrors();
        res.success = errorResponse;
        res.message = "error reset done";
    }
    else{
        res.success = false;
        res.message = "Invalid command";
    }
    return true;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "MotorClient");
    ROS_INFO("Kinco Motor Client executing");

    tf::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped encoder_tf;
    nav_msgs::Odometry odomMsg;
    ros::NodeHandle nh;

    bool useKinco;
    if (nh.getParam("useKinco", useKinco)){
        dm.Kinco = true;
    }
    else{
        dm.Kinco = false;
    }
    
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, setMotorVelocity);
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    ros::ServiceServer srv = nh.advertiseService("reset_odom", reset_odom);
    ros::ServiceServer err_srv = nh.advertiseService("reset_error", reset_error);
    ros::Rate loop_rate(50);

    while(ros::ok()){
        Pose2D poseAMR = dm.diffDrive->calculateOdom();
        odomMsg.header.stamp = ros::Time::now();
        odomMsg.header.frame_id = "odom";
        odomMsg.child_frame_id = "base_footprint";

        encoder_tf.header.stamp = ros::Time::now();
        encoder_tf.header.frame_id = "odom";  
        encoder_tf.child_frame_id = "base_footprint";

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(poseAMR.theta);

        odomMsg.pose.pose.position.x = poseAMR.x;
        odomMsg.pose.pose.position.y = poseAMR.y;
        odomMsg.pose.pose.position.z = 0.0;
        odomMsg.pose.pose.orientation = odom_quat;
        
        odomMsg.twist.twist.linear.x = poseAMR.Vx;
        odomMsg.twist.twist.linear.y = poseAMR.Vy;
        odomMsg.twist.twist.angular.z = poseAMR.Vw;
          
        encoder_tf.transform.translation.x = poseAMR.x;  
        encoder_tf.transform.translation.y = poseAMR.y;
        encoder_tf.transform.translation.z = 0.0;
        encoder_tf.transform.rotation = odom_quat;
    
        tf_broadcaster.sendTransform(encoder_tf);
        pub.publish(odomMsg);
        loop_rate.sleep();
        ros::spinOnce();
    }

    dm.motorClient->stopAll();
    ROS_INFO("Kinco Motor Client Stopped");
    ros::spin();
    return 0;
}
