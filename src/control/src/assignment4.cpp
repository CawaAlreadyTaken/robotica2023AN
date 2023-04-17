/*
  TODO:
   - implement
*/

#include <custom_joint_publisher.h>
#include <math.h>
#include <vision/custMsg.h>
#include <vector>

#include "inverseKin.cpp"

<<<<<<< HEAD
#define X_INC 0.1
#define Y_INC 0.1

double x_min;
double y_min;
int xsteps;
int ysteps;
int max_x_steps;
double x_inc;
double y_inc;
Vector3d pos;

void increment(Vector3d &pos) {
    pos(0) = x_min + (xsteps%max_x_steps) * x_inc;
    ysteps = xsteps/max_x_steps;
    pos(1) = ysteps * y_inc; 
    xsteps++;
}

typedef struct block{ 
  int type;
  Vector3d pos;

  block(int _class, Vector3d _pos) : type(_class), pos(_pos) {}
} block;

typedef struct blocks {
  vector<block> block_arr;

  blocks() {}
  blocks(vector<block> blk) {
    for(auto i = blk.begin(); i != blk.end(); i++) {
      this->block_arr.push_back(*i);
    }
  }

  void inserisci(int cl, double x, double y, double z) {
    block_arr.push_back(block(cl, Vector3d(x, y, z)));
  }

  void inserisci(int cl, Vector3d pos){
    block_arr.push_back(block(cl, pos));
  }

  void riordina() {
    sort(this->block_arr.begin(), this->block_arr.end(), [](const block& a, const block& b) -> bool { return a.pos[2] < b.pos[2]; });
  }

  vector<block>::iterator find(int cl) {
    auto r = find_if(block_arr.begin(), block_arr.end(), [elem](const int cl){if(elem.type == cl) return true; return false;});
    if(r == NULL) {
      cout << "no blocks for class " << cl << " found" << endl;
    }
    return r;
  }
} blocks;

ostream &operator << (ostream &os, const blocks &b) {
  for(auto i = b.block_arr.begin(); i != b.block_arr.end(); i++) {
    os << i->type << ", pos: " << "[" << i->pos[0] << ", " << i->pos[1] << ", " << i->pos[2] << "]" << endl;
  }
  return os;
}

//my blocks
blocks my_blocks;

const double pi = 2 * acos(0.0);
Vector3d finalDestination;
ros::Subscriber sub;

InverseKinematic invKin;
bool is_moving;

=======
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


>>>>>>> f1224cb467ff0dce438994e4794012ad43c169db
void send_des_jstate(const JointStateVector& joint_pos) {
  for (int i = 0; i < joint_pos.size(); i++) {
    jointState_msg_robot.data[i] = joint_pos[i];
  }

  pub_des_jstate.publish(jointState_msg_robot);
}

<<<<<<< HEAD
void moveRobot(Vector3d dest, double height, double g, double time) {
  ros::Rate loop_rate(loop_frequency);
  dest(2) = height;
  Vector3d m(0, 0, pi);
=======
void moveRobot(Vector3d dest, double height, double g, double time, double rotation = pi) {
  ros::Rate loop_rate(loop_frequency);
  dest(2) = height;
  Vector3d m(0, 0, rotation);
>>>>>>> f1224cb467ff0dce438994e4794012ad43c169db
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

<<<<<<< HEAD
void getMoveAndDropObject(Vector3d initialPosition, Vector3d finalPosition) {
  moveRobot(initialPosition, UP_HEIGHT, OPEN_GRIP, TIME_FOR_MOVING);
  moveRobot(initialPosition, DOWN_HEIGHT, OPEN_GRIP, TIME_FOR_LOWERING_RISING);
  moveRobot(initialPosition, DOWN_HEIGHT, CLOSE_GRIP, TIME_FOR_CLOSING_OPENING);
  moveRobot(initialPosition, UP_HEIGHT, CLOSE_GRIP, TIME_FOR_LOWERING_RISING);
=======

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
>>>>>>> f1224cb467ff0dce438994e4794012ad43c169db
  moveRobot(finalPosition, UP_HEIGHT, CLOSE_GRIP, TIME_FOR_MOVING);
  moveRobot(finalPosition, DOWN_HEIGHT, CLOSE_GRIP, TIME_FOR_LOWERING_RISING);
  moveRobot(finalPosition, DOWN_HEIGHT, OPEN_GRIP, TIME_FOR_CLOSING_OPENING);
  moveRobot(finalPosition, UP_HEIGHT, OPEN_GRIP, TIME_FOR_LOWERING_RISING);
}

<<<<<<< HEAD
void visionCallback(const vision::custMsg::ConstPtr& msg) { //ordina i blocchi che vede
=======
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
>>>>>>> f1224cb467ff0dce438994e4794012ad43c169db
  if(is_moving) return;
  is_moving = true;
  std::cout << "Received vision message, starting to move..." << std::endl;
  std::cout << "x: " << msg->x << std::endl;
  std::cout << "y: " << msg->y << std::endl;
  std::cout << "z: " << msg->z << std::endl;
  std::cout << "index: " << msg->index << std::endl;
<<<<<<< HEAD
  
  Vector3d WorldCoords = Vector3d(msg->x, msg->y, msg->z);
  Vector3d Ur5Coords = invKin.fromWorldToUrd5(WorldCoords);
  Vector3d finalDestination = pos;
  my_blocks.inserisci(msg->index%11, finalDestination);
  increment(pos);
  cout << "moving to: " << "[" << pos(0) << ", " << pos(1) << ", " << pos(2) << "]" << endl;
 
  getMoveAndDropObject(Ur5Coords, finalDestination);
  sub.shutdown();
  is_moving = false;
  exit(0);
=======

  Vector3d WorldCoords = Vector3d(msg->x, msg->y, msg->z);
  Vector3d Ur5Coords = invKin.fromWorldToUrd5(WorldCoords);

  getMoveAndDropObject(Ur5Coords, invKin.fromWorldToUrd5(Vector3d(FINAL_POSITIONS_ASS4[counter][0], FINAL_POSITIONS_ASS4[counter][1], DOWN_HEIGHT)), msg->index);
  counter++;
  cout<<"counter: "<<counter<<endl;
  if(counter == 7) {
    build();
  };
  is_moving = false;
>>>>>>> f1224cb467ff0dce438994e4794012ad43c169db
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

<<<<<<< HEAD
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
}


int main(int argc, char** argv) {
  /*
   * inizializza figura
   * */
  
  xsteps = 0;
  ysteps = 0;
  x_min = -0.8;
  double x_max = 0.8; 
  y_min = -0.9;
  max_x_steps = (x_max - x_min)/x_inc;
  y_inc = 0.1; 
  x_inc = 0.1;
  pos << x_min, y_min;

=======
int main(int argc, char** argv) {
  
>>>>>>> f1224cb467ff0dce438994e4794012ad43c169db
  ros::init(argc, argv, "custom_joint_publisher");
  ros::NodeHandle node;
  is_moving = false;
  node.getParam("/real_robot", real_robot);
  invKin = InverseKinematic();
<<<<<<< HEAD
  finalDestination << 0.35, -0.35, DOWN_HEIGHT;
=======
>>>>>>> f1224cb467ff0dce438994e4794012ad43c169db

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
<<<<<<< HEAD
  q_des_init << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  initFilter(q_des_init);

  homing_procedure();
  std::cout << "Reached home" << std::endl;

=======
  q_des_init << 0., 0., 0., 0., 0., 0., 0., 0., 0.;
  initFilter(q_des_init);

>>>>>>> f1224cb467ff0dce438994e4794012ad43c169db
  while (ros::ok()) {
    ros::spinOnce();
  };

  return 0;
}
<<<<<<< HEAD
















=======
>>>>>>> f1224cb467ff0dce438994e4794012ad43c169db
