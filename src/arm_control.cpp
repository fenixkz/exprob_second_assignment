/**
 * @file arm_control.cpp
 * @author Ayan Mazhitov
 * @brief Node done to move the arm of the robot
 * @version 1.0
 * @date 2023-02-26
 * 
 * @copyright Copyright (c) 2023
 * @details
 * 
 * Subscribes to: <BR>
 *  /robot/joint_states 
 *  /isDone
 * 
 * Publishes to: <BR>
 *  /robot/joint1_position_controller/command 
 *  /robot/joint2_position_controller/command 
 * 
 * 
 * Description:
 * Allows the arm structure of the robot to rotate permitting the detection of all the markers located in the 
 * initial room. The base joint rotates around itself of 360 dergees whereas the camera tilts of about 20
 * degree both upwards and downwards. This is done until all markers have been detected.
 * When all markers have been detected, a message from aruco_detector node is made. This server node makes the arm
 * return in its home position and after that the node is shut down.
 */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int64.h>
std::size_t state_machine = 0;
std_msgs::Float64 j_msg;
ros::Publisher j1_pub, j2_pub, j3_pub;
double j1_pos, j2_pos;
/**
     * @brief Function that executes a callback for the /robot/joint_states topic
     * 
     * @param [in] msg JointState message
     * 
     * This function is executed every time a new message has been published to the topic. It reads the position of joint 1 and joint 2 of the arm and stores them in the global variables.
     */
void js_cb(const sensor_msgs::JointState& msg){
    j1_pos = msg.position[0];
    j2_pos = msg.position[1];
}
/**
     * @brief Function that executes a callback for the /isDone topic
     * 
     * @param [in] msg Int64 message
     * 
     * This function is executed every time a new message has been published to the topic. It reads the message and if it is 1, then the arm is moved to the home position and the nodes finishes.
     */
void done_cb(const std_msgs::Int64& msg){
    if (msg.data == 1){
        j_msg.data = 0.0;
        j1_pub.publish(j_msg);
        j2_pub.publish(j_msg);
        ros::shutdown();
    }
}

/**
 * @brief Main function
 * 
 * @param argc 
 * @param argv 
 * @return 0
 * 
 * Main function, initializes the node and does the rotation of the arm pipiline.
 */
int main(int argc, char **argv){
    ros::init(argc, argv, "arm_control");
    ros::NodeHandle nh;
    // ROS stuff
    ros::Subscriber js_sub, done_sub;
    js_sub = nh.subscribe("/robot/joint_states", 10, js_cb);
    done_sub = nh.subscribe("/isDone", 10, done_cb);
    j1_pub = nh.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 10);
    j2_pub = nh.advertise<std_msgs::Float64>("/robot/joint2_position_controller/command", 10);
    j3_pub = nh.advertise<std_msgs::Float64>("/robot/joint3_position_controller/command", 10);
    ros::Rate loop_rate(50);
    while(ros::ok()) {
        // Arm control pipeline, first tilt the camera a little bit downwards.
        if (state_machine == 0){
            if(abs(j2_pos - 0.3) > 0.1){
                j_msg.data = 0.3;
                j2_pub.publish(j_msg);
            }else{
                state_machine = 1;
            }
        // Then rotate the base joint to the left
        }else if (state_machine == 1){
            if(abs(j1_pos + 3.14) > 0.1){
                j_msg.data = -3.14;
                j1_pub.publish(j_msg);
            }else{
                state_machine = 2;
            }
        // Then rotate the base joint to the right
        }else if (state_machine == 2){
            if(abs(j1_pos - 3.14) > 0.1){
                j_msg.data = +3.14;
                j1_pub.publish(j_msg);
            }else{
                state_machine = 3;
            }
        // Tilt the camera upwards to see the markers above
        }else if (state_machine == 3){
            if( abs(j2_pos + 0.77) > 0.1 ){
                j_msg.data = -0.77;
                j2_pub.publish(j_msg);
            }else{
                state_machine = 4;
            }
        // Spin the base joint
        }else if (state_machine == 4){
            if(abs(j1_pos + 3.14) > 0.1){
                j_msg.data = -3.14;
                j1_pub.publish(j_msg);
            }else{
                state_machine = 5;
            }
        }
    	ros::spinOnce();
    	loop_rate.sleep();
    }
    return 0;
}