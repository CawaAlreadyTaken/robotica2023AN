#include <custom_joint_publisher.h>
#include <math.h>
#include "inverseKin.cpp"
#include <vision/custMsg.h>

const double pi = 2*acos(0.0);

void send_des_jstate(const JointStateVector & joint_pos)
{
    if (real_robot || true)
    {
        for (int i = 0; i < joint_pos.size(); i++)
        {
          jointState_msg_robot.data[i] = joint_pos[i];
        }

        pub_des_jstate.publish(jointState_msg_robot);


    } else {
        for (int i = 0; i < joint_pos.size(); i++)
        {
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


void initFilter(const JointStateVector & joint_pos)
{
        filter_1 = joint_pos;
        filter_2 = joint_pos;
}

JointStateVector secondOrderFilter(const JointStateVector & input, const double rate, const double settling_time)
{

        double dt = 1 / rate;
        double gain =  dt / (0.1*settling_time + dt);
        filter_1 = (1 - gain) * filter_1 + gain * input;
        filter_2 = (1 - gain) * filter_2 + gain *filter_1;
        return filter_2;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "custom_joint_publisher");
  //ros::init(argc, argv, "custom_joint_pub_node");
  ros::NodeHandle node;

  //pub_des_jstate_sim_rt.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(node, "/command", 1));

  node.getParam("/real_robot", real_robot);

  std::cout << real_robot << std::endl;

  if (true || real_robot)
  {
      pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);

  } else {
      pub_des_jstate = node.advertise<sensor_msgs::JointState>("/command", 1);
      // pub_des_jstate = node.advertise<sensor_msgs::JointState>("/ur5/joint_group_pos_controller/command", 1);
      // pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
  }

  ros::Rate loop_rate(loop_frequency);

  jointState_msg_sim.position.resize(8);
  jointState_msg_sim.velocity.resize(8);
  jointState_msg_sim.effort.resize(8);
  jointState_msg_robot.data.resize(8);
  
  //q_des0 << -0.3223527113543909, -1.7805794638446351, -1.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558, -3, 3, 1;
  //q_des0 << -0.32, -0.78, -2.56, -1.63, -1.57, -2.79, -3, 3, 1;
  //q_des0 << 1.54, -0.04, -0.6, -1.98, -1.32, 1.62, -3, 3, 1;

  JointStateVector q_des_fin;
  // JointStateVector q_des_init;

  // Vector3f point(-0.11, -0.23, 0.60);
  // Vector3f m(0, 0, pi/2);

  // Vector3f current(-0.11, -0.23, 0.73);
  // Vector3f mCurrent(0, 0, pi/2);
  
  // InverseKinematic inverseKin = InverseKinematic(point, m); //insert destination 
  // inverseKin.getJointsPositions(q_des_fin);

  // InverseKinematic inverseKinCurrent = InverseKinematic(current, mCurrent); //insert destination 
  // inverseKinCurrent.getJointsPositions(q_des_init);
  
  // initFilter(q_des_init);

  q_des_fin << 1, 1, 1, 1, 0, 0, 0, 0, 0;
  
  /*
  JointStateVector amp;
  JointStateVector freq;
  amp << 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  freq << 0.2, 0.0, 0.0, 0.0, 0., 0.0, 0.0, 0.0;
  */

  while (ros::ok())
  {
    //1- step reference
    if (true || loop_time < 5.)
    {
      q_des = secondOrderFilter(q_des_fin, loop_frequency, 5.);
    } else {
      JointStateVector delta_q;
      delta_q << 0., 0., 0., 0., 0., 0.,0.,0.,0.;
      //q_des = q_des0 + delta_q;
      q_des = secondOrderFilter(q_des0 + delta_q, loop_frequency, 5.);
    }

    //2- sine reference
//    q_des = q_des0.array() + amp.array()*(2*M_PI*freq*loop_time).array().sin();

    loop_time += (double)1/loop_frequency;
    //send_des_jstate_sim(q_des);
    send_des_jstate(q_des);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

