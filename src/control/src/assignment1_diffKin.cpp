#include <custom_joint_publisher.h>
#include <math.h>
#include <vision/custMsg.h>

#include "differentialKin.cpp"
#include "inverseKin.cpp"
#include "diffKinLibrary.cpp"

using namespace std;
using namespace Eigen;

InverseKinematic invKin;
DifferentialKinematic diffKin;

const double pi = 2 * acos(0.0);
Vector3d finalDestination;
ros::Subscriber sub;

bool is_moving;
bool debug;

void send_des_jstate(const JointStateVector& joint_pos) {
  for (int i = 0; i < joint_pos.size(); i++) {
    jointState_msg_robot.data[i] = joint_pos[i];
  }

  pub_des_jstate.publish(jointState_msg_robot);
}

/*void moveRobot(Vector3d dest, double height, double g, double time) {
  ros::Rate loop_rate(loop_frequency);
  dest(2) = height;
  Vector3d m(0, 0, pi);
  //Vector<float, 6> Qdot(joint_states, desired_pos, desired_pos_vel, current_RPY, desired_RPY, desired_RPY_velocities, Kq, KRPY)
  JointStateVector q_curr;
  //how to get q_curr ?
  JointStateVector q_des;
  JointStateVector v_des;
  Vector3d current_rpy;
  Vector3d desired_rpy; desired_rpy = diffKin.fromRotToRPY(Matrix<double, 3, 3>::Identity());
  Vector3d rpy_velocities;
  Matrix<double, 3, 3> K;
  K <<
  0.1, 0, 0,
  0, 0.1, 0,
  0, 0, 0.1;

  invKin.setDestinationPoint(dest,m,g);
  invKin.getJointsPositions(q_des);
  while (loop_time < TIME_FOR_MOVING) {

    loop_time += (double)1 / loop_frequency;
    ros::spinOnce();
    loop_rate.sleep();
  }
  loop_time = 0;
  cout << "Robot moved" << endl;
}

void getMoveAndDropObject(Vector3d initialPosition, Vector3d finalPosition) {
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
  cout << "Received vision message, starting to move..." << endl;
  cout << "x: " << msg->x << endl;
  cout << "y: " << msg->y << endl;
  cout << "z: " << msg->z << endl;
  cout << "index: " << msg->index << endl;

  Vector3d WorldCoords = Vector3d(msg->x, msg->y, msg->z);
  Vector3d Ur5Coords = invKin.fromWorldToUrd5(WorldCoords);
  Vector3d finalDestination = invKin.fromWorldToUrd5(FINAL_POSITIONS[(msg->index)%NUMBER_OF_CLASSES]);

  getMoveAndDropObject(Ur5Coords, finalDestination);
  sub.shutdown();  // Assignment1 only has one block
  is_moving = false;
  exit(0);
}*/

void initFilter(JointStateVector s) {
  filter_1 = s;
  filter_2 = s;
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

void homing_procedure() {
  Vector3d dest(0.0, -0.4, 0.58);
  ros::Rate loop_rate(loop_frequency);
  Vector3d m(0, 0, pi);
  JointStateVector q_des;
  JointStateVector q_des_filtered;
  invKin.setDestinationPoint(dest,m,0);
  invKin.getJointsPositions(q_des);
  while (loop_time < TIME_FOR_MOVING) {
    q_des_filtered = secondOrderFilter(q_des, loop_frequency, TIME_FOR_MOVING);
    send_des_jstate(q_des_filtered);
    loop_time += (double)1 / loop_frequency;
    ros::spinOnce();
    loop_rate.sleep();
  }
  loop_time = 0;

  diffKin.updateRobotsPosition(q_des.head(6));
}

//moves to point in robot coordinates
void move_inv(Vector3d point, Vector3d rpy, int gripper=0) {
  ros::Rate loop_rate(loop_frequency);
  invKin.setDestinationPoint(point, rpy, gripper);
  JointStateVector q_des;
  JointStateVector q_des_filtered;
  invKin.getJointsPositions(q_des); //fill q_des with result from inverse kinematic
  JointStateVector init;
  double g; g = invKin.getCurrentGripper();
  init << invKin.getCurrentPosition(), g, g, g;
  initFilter(init);
  while(loop_time < TIME_FOR_MOVING) {
    q_des_filtered = secondOrderFilter(q_des, loop_frequency, TIME_FOR_MOVING);
    send_des_jstate(q_des_filtered);
    loop_time += (double)1 / loop_frequency;
    ros::spinOnce();
    loop_rate.sleep();
  }

  loop_time = 0;
  invKin.setCurrentPosition(q_des_filtered.head(6));
}

// Vector3d circle(double t)
// {
//   double omega = 1;
//   return Vector3d(0.2 * cos(omega * t), -0.04, 0.2 * sin(omega * t));
// }

// Vector3d circle_speed(double t)
// {
//   double omega = 1;
//   return Vector3d(0.2 * -sin(omega * t), 0, 0.2 * cos(omega * t));}

Vector3d retta(Vector3d pi, Vector3d pf, double t)
{
  return t * (pf) + (1 - t) * pi;
}

Vector3d retta_speed(Vector3d pi, Vector3d pf) {
  return (pf - pi)/TIME_FOR_MOVING;
}

Vector3d retta_speed_normalized(Vector3d pi, Vector3d pf) {
  return (pf - pi)/(pf - pi).norm();
}

Vector3d retta_speed_divided(Vector3d pi, Vector3d pf) {
  return (pf - pi) / TIME_FOR_MOVING;
}

Vector3d retta_speed_divided_by_loopfreq(Vector3d pi, Vector3d pf) {
  return (pf - pi) / loop_frequency ;
}




void move_to(Vector3d pf, Vector3d rpy, double gripper = 0, double k=0.1, double k_rpy=0.1)  {
  ros::Rate loop_rate(loop_frequency);

  Matrix<double, 6, 1> q0;
  Matrix<double, 6, 1> qk;
  Vector3d v;
  Vector3d v_rpy;
  JointStateVector to_send;
  Matrix<double, 3, 3> K;
  Matrix<double, 3, 3> K_rpy;

  K <<
  k, 0, 0,
  0, k, 0,
  0, 0, k;
  K_rpy <<
  k_rpy, 0, 0,
  0, k_rpy, 0,
  0, 0, k_rpy;

  double t = 0;
  int count = 0;

  q0 << diffKin.getCurrentPosition(); //current joint coordinates
  qk = q0;
  v = retta_speed(diffKin.getEECoords(), pf); // in robot coords
  v_rpy << 0, 0, 0;

  cout << "initial pos:" << endl << diffKin.getEECoords() << endl;
  cout << "final pos:" << endl << pf << endl;
  cout << "vel:" << endl << v << endl;
  cout << "rpy vel:" << endl << v_rpy << endl;
  // temporary
  TIME_FOR_MOVING = 1;

  while(loop_time < TIME_FOR_MOVING) {
    count++;
    t = loop_time/TIME_FOR_MOVING;
    qk = q0 + diffKin.Qdot(qk, retta(diffKin.getEECoords(), pf, t), v, retta(diffKin.getEulerAngles(), rpy, t) , v_rpy, K, K_rpy);

    to_send << qk, gripper, gripper, gripper;
    send_des_jstate(to_send);

    loop_time += (double)1 / loop_frequency;
    cout << "retta position:" << endl << retta(diffKin.getEECoords(), pf, t) << endl;
    cout << "joints:" << endl << qk << endl;

    ros::spinOnce();
    loop_rate.sleep();
  }

  diffKin.updateRobotsPosition(qk);
  loop_time = 0;
}


void move_diff(Vector3d pi, Vector3d pf, Vector3d rpy, int gripper=0, double k=0.1, double k_rpy=0.1) {
  ros::Rate loop_rate(loop_frequency);

  Matrix<double, 6, 1> q0;
  Matrix<double, 6, 1> qk;
  Vector3d v;
  Vector3d v_rpy;
  JointStateVector to_send;
  Matrix<double, 3, 3> K;
  Matrix<double, 3, 3> K_rpy;

  K <<
  k, 0, 0,
  0, k, 0,
  0, 0, k;
  K_rpy <<
  k_rpy, 0, 0,
  0, k_rpy, 0,
  0, 0, k_rpy;

  double t = 0;
  q0 << diffKin.getCurrentPosition(); //current joint coordinates
  qk = q0;
  v = retta_speed(pi, pf);
  v_rpy = Vector3d(0, 0, 0);

  TIME_FOR_MOVING = 1;

  while(loop_time < TIME_FOR_MOVING) {
    t = loop_time/TIME_FOR_MOVING;
    qk = q0 + diffKin.Qdot(qk, retta(pi, pf, t), v, retta(diffKin.getEulerAngles(), rpy, t) , v_rpy, K, K_rpy);

    send_des_jstate(to_send);

    to_send << qk, gripper, gripper, gripper;


    loop_time += (double)1 / loop_frequency;
    ros::spinOnce();
    loop_rate.sleep();

  }

  diffKin.updateRobotsPosition(qk);
  loop_time = 0;

}

int main(int argc, char** argv) {
  debug = false;
  invKin = InverseKinematic();
  diffKin = DifferentialKinematic();

  ros::init(argc, argv, "custom_joint_publisher");
  ros::NodeHandle node;
  is_moving = false;
  node.getParam("/real_robot", real_robot);

  pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>(
    "/ur5/joint_group_pos_controller/command", 1);

  // sub =
  //     node.subscribe("/vision/blocksCoords", 1, visionCallback);

  //cout << "Waiting for vision message..." << endl;

  jointState_msg_sim.position.resize(9);
  jointState_msg_sim.velocity.resize(9);
  jointState_msg_sim.effort.resize(9);
  jointState_msg_robot.data.resize(9);

  JointStateVector q_des_init;
  q_des_init << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  initFilter(q_des_init);

  homing_procedure();

  /*
  Dopo la homing, il braccio si trova in coordinate: (0, -0.4, 0.58) rispetto al frame del robot
  gli angoli sono: (0, 0, pi) rispetto al frame del robot
  i giunti valgono: (1.23105, -1.81617, 2.05917, -1.8138, 1.5708, -2.80184)
  */
  Vector3d pinit = diffKin.getEECoords();

  Vector3d pf; pf << 0.3, -0.4, 0.58; //
  Vector3d rpy; rpy << 0, 0, pi;

  //move_diff(pi, pf, rpy, 0, 0.5, 0.5);
  move_to(pf, rpy, 0, 0.1, 0.1);
  // pf = pinit;
  // //move_to(pf, rpy, 0, 0.1, 0.1);
  // pf = Vector3d(0.5, 0.75, 1.17);
  //move_to(diffKin.fromWorldToUrd5(pf), rpy, 0, 0, 0);

  //move_to(point, rpy, gripper, kq, krpy)

  // pi=pf;
  // pf << 0.3, -0.4, 0.58;
  //move_diff(pi, pf);
  //cout << "Reached home" << endl;
  //move(Vector3d(0.2, -0.4, 0.58), Vector3d(0.3, -0.4, 0.58));
  return 0;
}

/*
  TODO: read joint states from topic
*/
