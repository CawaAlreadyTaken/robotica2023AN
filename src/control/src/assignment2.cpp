/*
  TODO:
   - fix rotation
*/

#include <custom_joint_publisher.h>
#include <math.h>
#include <vision/custMsg.h>

#include "inverseKin.cpp"
#include "differentialKin.cpp"
#include <path_finding.h>

#define DIM 10
#define SET_HEIGHT 1.174
#define INCREMENT 0.1
#define SCALE 10
#define HIGH_COST 100

ros::Subscriber sub;

InverseKinematic invKin;
DifferentialKinematic diffKin;
bool is_moving;

void send_des_jstate(const JointStateVector& joint_pos) {
  for (int i = 0; i < joint_pos.size(); i++) {
    jointState_msg_robot.data[i] = joint_pos[i];
  }

  pub_des_jstate.publish(jointState_msg_robot);
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

void homing_procedure() {
  Vector3d dest(0.2, -0.4, 0.58);
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
  diffKin.setRobotsPosition(q_des_filtered.head(6)); // actual position
}

void move_to(Vector3d pf, Vector3d rpy, double gripper = 0, double k=0.1, double k_rpy=0.1) {
  ros::Rate loop_rate(loop_frequency);

  Vector3d ee_pos = diffKin.getEECoords();
  Vector3d current_rpy = diffKin.getEulerAngles();

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

  double Dt = (double)1 / (loop_frequency * TIME_FOR_MOVING);
  double t = Dt;


  q0 << diffKin.getCurrentPosition(); //current joint coordinates
  qk = q0;
  v_rpy = rpy - diffKin.getEulerAngles();

  while(loop_time <= TIME_FOR_MOVING - Dt) {
    Matrix<double, 6, 1> tmp = qk;

    qk += diffKin.Qdot(qk, linear_interpol(ee_pos, pf, t), trapezoidal_velocity(ee_pos, pf, t), linear_interpol(current_rpy, rpy, t) , v_rpy, K, K_rpy) * Dt;

    to_send << qk, gripper, gripper, gripper;
    send_des_jstate(to_send);

    loop_time += (double)1 / loop_frequency;
    t+=Dt;

    ros::spinOnce();
    loop_rate.sleep();
  }

  loop_rate.sleep(); // serve per far arrivare loop_time a 3 secondi esatti

  diffKin.setRobotsPosition(qk);
  loop_time = 0;

  ee_pos = diffKin.getEECoords();
  cout << "final position:" << endl << ee_pos << endl;
}


void move_collision_detection(Vector3d start_rcoords, Matrix<double, 6, 1> joints, Vector3d end_rcoords, double gripper = 0, double k = 0.1, double k_rpy = 0.1) {
  // get Nodes from pi and pf
  Vector3d pi_wcoords = diffKin.fromUr5ToWorld(start_rcoords);
  Vector3d pf_wcoords = diffKin.fromUr5ToWorld(end_rcoords);

  Node start = get_closest_node(pi_wcoords);
  Node end = get_closest_node(pf_wcoords);

  // init A* algorithm
  Envmap gscores(DIM, vector<double>(DIM, INFINITY));
  Envmap hscores(DIM, vector<double>(DIM, INFINITY)); 
  Envmap fscores(DIM, vector<double>(DIM, INFINITY));
  Fathers fathers(DIM, vector<Node>(DIM)); 
  Jointmap jointmap(DIM, vector<Matrix<double, 6, 1>>(DIM));

  jointmap[int(start.first * SCALE)][int(start.second * SCALE)] = joints;
  init(start, end, gscores, hscores, fscores, fathers);

  Path path = a_star(start, end, gscores, hscores, fscores, fathers, jointmap, manhattan_distance, close_to_collisions);
  Path points = get_lines(path);

  // for each point, convert into robots coordinates and move robot

  for(int i = 1; i < points.size(); ++i) {
    Vector3d dest = diffKin.fromWorldToUr5(Vector3d(points[i].first, points[i].second, SET_HEIGHT));
    move_to(dest, Vector3d(0, 0, pi));
  }

}


void pick_and_place(Vector3d pick, Vector3d place, double rotation) {

  // assume i start at a set height
  Vector3d rpy; rpy << 0, 0, pi;
  Vector3d rpy_approaching; rpy_approaching << 0, 0, rotation;

  pick(2) = UP_HEIGHT;
  move_to(pick, rpy);

  pick(2) = DOWN_HEIGHT;
  move_to(pick, rpy, OPEN_GRIP);

  move_to(pick, rpy, CLOSE_GRIP);

  pick(2) = UP_HEIGHT;
  move_to(pick, rpy, CLOSE_GRIP);

  place(2) = UP_HEIGHT;
  move_to(place, rpy, CLOSE_GRIP);

  place(2) = DOWN_HEIGHT;
  move_to(place, rpy, CLOSE_GRIP);

  move_to(place, rpy, OPEN_GRIP);

  place(2) = UP_HEIGHT;
  move_to(place, rpy);
}

void visionCallback(const vision::custMsg::ConstPtr& msg) {
  if(is_moving) return;
  is_moving = true;
  std::cout << "Received vision message, starting to move..." << std::endl;
  std::cout << "x: " << msg->x << std::endl;
  std::cout << "y: " << msg->y << std::endl;
  std::cout << "z: " << msg->z << std::endl;
  std::cout << "index: " << msg->index << std::endl;

  Vector3d WorldCoords = Vector3d(msg->x, msg->y, msg->z);
  Vector3d Ur5Coords = invKin.fromWorldToUrd5(WorldCoords);

  double rotation = -(2*pi*(double)((msg->index/11)*45)/360) + pi;
  pick_and_place(Ur5Coords, invKin.fromWorldToUrd5(Vector3d(FINAL_POSITIONS[msg->index%11][0], FINAL_POSITIONS[msg->index%11][1], DOWN_HEIGHT)), rotation);
  is_moving = false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "custom_joint_publisher");
  ros::NodeHandle node;
  is_moving = false;
  node.getParam("/real_robot", real_robot);
  invKin = InverseKinematic();
  diffKin = DifferentialKinematic();

  pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>(
      "/ur5/joint_group_pos_controller/command", 1);

  sub =
      node.subscribe("/vision/blocksCoords", 1, visionCallback);


  jointState_msg_sim.position.resize(9);
  jointState_msg_sim.velocity.resize(9);
  jointState_msg_sim.effort.resize(9);
  jointState_msg_robot.data.resize(9);

  JointStateVector q_des_init;
  q_des_init << 0, 0, 0, 0, 0, 0, 0, 0, 0;

  initFilter(q_des_init);
  homing_procedure();
  std::cout << "reached home" << std::endl;
  std::cout << "Waiting for vision message..." << std::endl;

  Vector3d start_rcoords(0.2, -0.4, 0.58);
  Matrix<double, 6, 1> homing_joints = diffKin.getCurrentPosition();
  Vector3d end_rcoords(0.2, -0.2, 0.58);
  move_collision_detection(start_rcoords, homing_joints, end_rcoords);

  // while (ros::ok()) {
  //   ros::spinOnce();
  // };

  return 0;
}
