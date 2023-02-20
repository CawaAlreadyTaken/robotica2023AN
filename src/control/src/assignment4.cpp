#include <custom_joint_publisher.h>
#include <math.h>
#include <vision/custMsg.h>
#include <vector>

#include "inverseKin.cpp"

const double pi = 2 * acos(0.0);
ros::Subscriber sub;

int counter = 0;

InverseKinematic invKin;
bool is_moving;

typedef struct Blocks{
  int index;
  Vector3d position;
} Blocks;

Blocks blocks[8];

vector <Blocks> blocksToMove = {
  {10, Vector3d(0.1, 0.35, 0.87)},
  {10, Vector3d(0.1, 0.5, 0.87)},
  {10, Vector3d(0.1, 0.35, 0.925)},
  {10, Vector3d(0.1, 0.5, 0.925)},
  {10, Vector3d(0.1, 0.35, 0.98)},
  {10, Vector3d(0.1, 0.5, 0.98)},
  {8, Vector3d(0.1, 0.425, 1.035)}
};


void send_des_jstate(const JointStateVector& joint_pos) {
  for (int i = 0; i < joint_pos.size(); i++) {
    jointState_msg_robot.data[i] = joint_pos[i];
  }

  pub_des_jstate.publish(jointState_msg_robot);
}

void moveRobot(Vector3d dest, double height, double g, double time, double rotation = pi) {
  ros::Rate loop_rate(loop_frequency);
  dest(2) = height;
  Vector3d m(0, 0, rotation);
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


void getMoveAndDropObject(Vector3d initialPosition, Vector3d finalPosition, int index/*index della classe che spostiamo*/) {
  double rotation = -(2*pi*(double)((index/11)*45)/360) + pi;
  cout << "initialPosition: " << endl << initialPosition << endl << "finalPosition: " << endl << finalPosition << endl << "rotation: " << endl << rotation << endl;
  moveRobot(initialPosition, UP_HEIGHT, OPEN_GRIP, TIME_FOR_MOVING,rotation);
  moveRobot(initialPosition, DOWN_HEIGHT, OPEN_GRIP, TIME_FOR_LOWERING_RISING,rotation);
  moveRobot(initialPosition, DOWN_HEIGHT, CLOSE_GRIP, TIME_FOR_CLOSING_OPENING,rotation);
  moveRobot(initialPosition, UP_HEIGHT, CLOSE_GRIP, TIME_FOR_LOWERING_RISING,rotation);
  moveRobot(finalPosition, UP_HEIGHT, CLOSE_GRIP, TIME_FOR_MOVING);
  moveRobot(finalPosition, DOWN_HEIGHT, CLOSE_GRIP, TIME_FOR_LOWERING_RISING);
  moveRobot(finalPosition, DOWN_HEIGHT, OPEN_GRIP, TIME_FOR_CLOSING_OPENING);
  moveRobot(finalPosition, UP_HEIGHT, OPEN_GRIP, TIME_FOR_LOWERING_RISING);
  blocks[counter].index = index;
  blocks[counter].position = finalPosition;
}

void buildBlock(Vector3d initialPosition, Vector3d finalPosition, int index){
  double rotation = -(2*pi*(double)((index/11)*45)/360) + pi;
  cout << "initialPosition: " << endl << initialPosition << endl << "finalPosition: " << endl << finalPosition << endl << "rotation: " << endl << rotation << endl;
  moveRobot(initialPosition, UP_HEIGHT, OPEN_GRIP, TIME_FOR_MOVING,rotation);
  moveRobot(initialPosition, DOWN_HEIGHT, OPEN_GRIP, TIME_FOR_LOWERING_RISING,rotation);
  moveRobot(initialPosition, DOWN_HEIGHT, CLOSE_GRIP, TIME_FOR_CLOSING_OPENING,rotation);
  moveRobot(initialPosition, UP_HEIGHT, CLOSE_GRIP, TIME_FOR_LOWERING_RISING,rotation);
  moveRobot(finalPosition, UP_HEIGHT, CLOSE_GRIP, TIME_FOR_MOVING);
  moveRobot(finalPosition, DOWN_HEIGHT, CLOSE_GRIP, TIME_FOR_LOWERING_RISING);
  moveRobot(finalPosition, DOWN_HEIGHT, OPEN_GRIP, TIME_FOR_CLOSING_OPENING);
  moveRobot(finalPosition, UP_HEIGHT, OPEN_GRIP, TIME_FOR_LOWERING_RISING);
}

void build(){
  cout<<"BUILDING STARTED"<<endl;
  for(int j = 0; j < 7; j++){
    for(int i = 0; i < 7; i++){
      if(blocks[i].index%11 == blocksToMove[j].index ){
        buildBlock(invKin.fromWorldToUrd5(blocks[i].position), invKin.fromWorldToUrd5(blocksToMove[j].position), blocks[i].index);
        blocks[i].index = -1;
        break;
      }
    }
  }
  cout << "BUILDING FINISHED" << endl;
  exit(0);
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

  getMoveAndDropObject(Ur5Coords, invKin.fromWorldToUrd5(Vector3d(FINAL_POSITIONS_ASS4[counter][0], FINAL_POSITIONS_ASS4[counter][1], DOWN_HEIGHT)), msg->index);
  counter++;
  cout<<"counter: "<<counter<<endl;
  if(counter == 7) {
    build();
  };
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

  pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>(
      "/ur5/joint_group_pos_controller/command", 1);

  sub =
      node.subscribe("/vision/blocksCoords", 1, visionCallback);

  std::cout << "Waiting for vision message..." << std::endl;

  jointState_msg_sim.position.resize(9);
  jointState_msg_sim.velocity.resize(9);
  jointState_msg_sim.effort.resize(9);
  jointState_msg_robot.data.resize(9);

  JointStateVector q_des_init;
  q_des_init << 0., 0., 0., 0., 0., 0., 0., 0., 0.;
  initFilter(q_des_init);

  while (ros::ok()) {
    ros::spinOnce();
  };

  return 0;
}
