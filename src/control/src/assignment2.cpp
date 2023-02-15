#include <custom_joint_publisher.h>
#include <math.h>
#include <vision/custMsg.h>

#include "inverseKin.cpp"

const double pi = 2 * acos(0.0);
Vector3f finalDestination;
ros::Subscriber sub;

InverseKinematic invKin;
bool is_moving;

void send_des_jstate(const JointStateVector& joint_pos) {
  for (int i = 0; i < joint_pos.size(); i++) {
    jointState_msg_robot.data[i] = joint_pos[i];
  }

  pub_des_jstate.publish(jointState_msg_robot);
}

void moveRobot(Vector3f dest, float height, float g, double time) {
  ros::Rate loop_rate(loop_frequency);
  dest(2) = height;
  Vector3f m(0, 0, pi);
  JointStateVector q_des;
  JointStateVector q_des_filtered;
  invKin.setDestinationPoint(dest,m,g);
  invKin.getJointsPositions(q_des);
  while (loop_time < TIME_FOR_MOVING) {
    q_des_filtered = secondOrderFilter(q_des, loop_frequency, time);
    send_des_jstate(q_des_filtered);
    loop_time += (double)1 / loop_frequency;
    ros::spinOnce();
    loop_rate.sleep();
  }
  loop_time = 0;
  std::cout << "Robot moved" << std::endl;
}

void getMoveAndDropObject(Vector3f initialPosition, Vector3f finalPosition) {
  moveRobot(initialPosition, UP_HEIGHT, OPEN_GRIP, TIME_FOR_MOVING);
  moveRobot(initialPosition, DOWN_HEIGHT, OPEN_GRIP, TIME_FOR_LOWERING_RISING);
  moveRobot(initialPosition, DOWN_HEIGHT, CLOSE_GRIP, TIME_FOR_CLOSING_OPENING);
  moveRobot(initialPosition, UP_HEIGHT, CLOSE_GRIP, TIME_FOR_LOWERING_RISING);
  moveRobot(finalPosition, UP_HEIGHT, CLOSE_GRIP, TIME_FOR_MOVING);
  moveRobot(finalPosition, DOWN_HEIGHT, CLOSE_GRIP, TIME_FOR_LOWERING_RISING);
  moveRobot(finalPosition, DOWN_HEIGHT, OPEN_GRIP, TIME_FOR_CLOSING_OPENING);
  moveRobot(finalPosition, UP_HEIGHT, OPEN_GRIP, TIME_FOR_LOWERING_RISING);
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

  getMoveAndDropObject(Ur5Coords, finalDestination);
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
  is_moving = false;
  node.getParam("/real_robot", real_robot);
  invKin = InverseKinematic();
  finalDestination << 0.35, -0.35, DOWN_HEIGHT;

  pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>(
      "/ur5/joint_group_pos_controller/command", 1);

  sub =
      node.subscribe("/vision/blocksCoords", 1, visionCallback);

  std::cout << "Waiting for vision message..." << std::endl;

  jointState_msg_sim.position.resize(8);
  jointState_msg_sim.velocity.resize(8);
  jointState_msg_sim.effort.resize(8);
  jointState_msg_robot.data.resize(8);

  JointStateVector q_des_init;
  q_des_init << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49, 0,0;
  initFilter(q_des_init);

  while (ros::ok()) {
    ros::spinOnce();
  };

  return 0;
}
