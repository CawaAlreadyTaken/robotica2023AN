#include <custom_joint_publisher.h>
#include <math.h>
#include <vision/custMsg.h>

#include "inverseKin.cpp"

const double pi = 2 * acos(0.0);

InverseKinematic invKin;
bool is_moving = false;

void send_des_jstate(const JointStateVector& joint_pos) {
  if (real_robot || true) {
    for (int i = 0; i < joint_pos.size(); i++) {
      jointState_msg_robot.data[i] = joint_pos[i];
    }

    pub_des_jstate.publish(jointState_msg_robot);

  } else {
    for (int i = 0; i < joint_pos.size(); i++) {
      jointState_msg_sim.position[i] = joint_pos[i];
      jointState_msg_sim.velocity[i] = 0.0;
      jointState_msg_sim.effort[i] = 0.0;
    }

    pub_des_jstate.publish(jointState_msg_sim);
  }

  /*   if (pub_des_jstate_sim_rt->trylock())
    {
      pub_des_jstate_sim_rt->msg_ = jointState_msg;
      pub_des_jstate_sim_rt->unlockAndPublish();
    } */
}

void moveRobot(Vector3f dest) {
  Vector3f m0(0, 0, pi);
  ros::Rate loop_rate(loop_frequency);
  float g0 = 1.0;
  invKin.setDestinationPoint(dest,m0,g0);
  JointStateVector q_des_0;
  invKin.getJointsPositions(q_des_0);
  JointStateVector q_des;
  while (ros::ok()) {
    if(loop_time > 3.0) break;
    q_des = secondOrderFilter(q_des_0, loop_frequency, 0.1);
    send_des_jstate(q_des);
    loop_time += (double)1 / loop_frequency;
    send_des_jstate(q_des);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void visionCallback(const vision::custMsg::ConstPtr& msg) {
  if(is_moving) return;
  is_moving = true;
  std::cout << "Received vision message, starting to move..." << std::endl;
  std::cout << "x: " << msg->x << std::endl;
  std::cout << "y: " << msg->y << std::endl;
  std::cout << "z: " << msg->z << std::endl;
  std::cout << "index: " << msg->index << std::endl;

  Vector3f WorldCoords = Vector3f(msg->x, msg->y, msg->z);
  Vector3f Ur5Coords = invKin.fromWorldToUrd5(WorldCoords);

  moveRobot(Ur5Coords);
  is_moving = false;
}

void initFilter(const JointStateVector& joint_pos) {
  filter_1 = joint_pos;
  filter_2 = joint_pos;
}

JointStateVector secondOrderFilter(const JointStateVector& input,
                                   const double rate,
                                   const double settling_time) {
  double dt = 1 / rate;
  double gain = dt / (0.1 * settling_time + dt);
  filter_1 = (1 - gain) * filter_1 + gain * input;
  filter_2 = (1 - gain) * filter_2 + gain * filter_1;
  return filter_2;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "custom_joint_publisher");
  ros::NodeHandle node;

  node.getParam("/real_robot", real_robot);

  invKin = InverseKinematic();

  std::cout << real_robot << std::endl;

  pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>(
      "/ur5/joint_group_pos_controller/command", 1);

  ros::Subscriber sub =
      node.subscribe("/vision/blocksCoords", 1, visionCallback);

  jointState_msg_sim.position.resize(8);
  jointState_msg_sim.velocity.resize(8);
  jointState_msg_sim.effort.resize(8);
  jointState_msg_robot.data.resize(8);

  JointStateVector q_des_init;
  q_des_init << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49, 0,0;
  initFilter(q_des_init);

  while (ros::ok());

  return 0;
}
