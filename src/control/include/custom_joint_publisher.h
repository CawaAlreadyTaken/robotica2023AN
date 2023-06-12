#ifndef CUSTOM_JOINT_PUB_H
#define CUSTOM_JOINT_PUB_H


#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>

typedef  Eigen::Matrix<double, 9, 1> JointStateVector;


// Methods
void send_des_jstate(const JointStateVector & joint_pos);
JointStateVector secondOrderFilter(const JointStateVector & input, const double rate, const double settling_time);

// Variables
JointStateVector q_des = JointStateVector::Zero();
JointStateVector q_des0 = JointStateVector::Zero();
JointStateVector qd_des = JointStateVector::Zero();
JointStateVector tau_ffwd = JointStateVector::Zero();
JointStateVector filter_1 = JointStateVector::Zero();
JointStateVector filter_2 = JointStateVector::Zero();

double loop_time = 0.;
double loop_frequency = 1000.;
double TIME_FOR_MOVING = 1.;
double TIME_FOR_LOWERING_RISING = 3.;
double TIME_FOR_CLOSING_OPENING = 1.;
float OPEN_GRIP = 0;
float CLOSE_GRIP = 2.5;
float UP_HEIGHT = 0.55;
float DOWN_HEIGHT = 0.70;
int NUMBER_OF_CLASSES = 11;

Eigen::Vector3d FINAL_POSITIONS[] = {
    Eigen::Vector3d(0.65, 0.55, 1),
    Eigen::Vector3d(0.65, 0.4, 1),
    Eigen::Vector3d(0.8, 0.7, 1),
    Eigen::Vector3d(0.8, 0.55, 1),
    Eigen::Vector3d(0.8, 0.25, 1),
    Eigen::Vector3d(0.8, 0.4, 1),
    Eigen::Vector3d(0.95, 0.7, 1),
    Eigen::Vector3d(0.95, 0.55, 1),
    Eigen::Vector3d(0.95, 0.25, 1),
    Eigen::Vector3d(0.95, 0.4, 1)
};

Eigen::Vector3d FINAL_POSITIONS_ASS4[] = {
    Eigen::Vector3d(0.65, 0.55, 1),
    Eigen::Vector3d(0.8, 0.7, 1),
    Eigen::Vector3d(0.8, 0.55, 1),
    Eigen::Vector3d(0.8, 0.4, 1),
    Eigen::Vector3d(0.95, 0.7, 1),
    Eigen::Vector3d(0.95, 0.55, 1),
    Eigen::Vector3d(0.95, 0.4, 1),
};

// Publishers
//std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > pub_des_jstate_sim_rt;
ros::Publisher pub_des_jstate;
sensor_msgs::JointState jointState_msg_sim;
std_msgs::Float64MultiArray jointState_msg_robot;

bool real_robot = false;

#endif
