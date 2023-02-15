#ifndef CUSTOM_JOINT_PUB_H
#define CUSTOM_JOINT_PUB_H


#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>

typedef  Eigen::Matrix<double, 8, 1> JointStateVector;


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

double  loop_time = 0.;
double  loop_frequency = 1000.;
double TIME_FOR_MOVING = 3.;
double TIME_FOR_LOWERING_RISING = 3.;
double TIME_FOR_CLOSING_OPENING = 1.;
float OPEN_GRIP = 1;
float CLOSE_GRIP = -0.1;
float UP_HEIGHT = 0.55;
float DOWN_HEIGHT = 0.79;
int NUMBER_OF_CLASSES = 11;
Eigen::Vector3f FINAL_POSITIONS[] = {
    Eigen::Vector3f(1, 2, 3),
    Eigen::Vector3f(1, 2, 3)
};

// Publishers
//std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > pub_des_jstate_sim_rt;
ros::Publisher pub_des_jstate;
sensor_msgs::JointState jointState_msg_sim;
std_msgs::Float64MultiArray jointState_msg_robot;

bool real_robot = false;

#endif

