#pragma once

#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <custom_joint_publisher.h>

using namespace std;
using namespace Eigen;



typedef vector<vector<Matrix<double, 6, 1>>> Jointmap;


/*!
* Classe per computare la soluzione analitica dell'inverse kinematic del braccio UR5, adattata dal paper: https://smartech.gatech.edu/bitstream/handle/1853/50782/ur_kin_tech_report_1.pdf?sequence=1
*/
class DifferentialKinematic
{

  Matrix<double, 3, 3> R; // current rotation matrix
  Matrix<double, 6, 1> current_position; // current joint values
  Matrix<double, 4, 4> T; // current rototranslation matrix?
  double pi;
  Matrix<double, 6, 1> A;
  Matrix<double, 6, 1> D;
  double gripper;
  Vector3d ee_coords; // rispetto al frame del robot

public:
  /*!
  * Inizializza alcuni parametri noti, come le lunghezze del robot.
  */
  DifferentialKinematic()
  {
    pi = 2 * acos(0.0);
    A << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0;
    D << 0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996;
    current_position << 0, 0, 0, 0, 0, 0; // first configuration of homing position
  }

  // getters and setters for current joint position, gripper, rotation matrix and end effector's position
  Matrix<double, 6, 1> getCurrentPosition()
  {
    return this->current_position;
  }

  void setGripper(double gripper)
  {
    this->gripper = gripper;
  }

  Vector3d getGripper()
  {
    return Vector3d(this->gripper, this->gripper, this->gripper);
  }

  Vector3d getEulerAngles()
  {
    return fromRotToRPY(this->R);
  }

  Vector3d getEECoords()
  {
    return this->ee_coords;
  }

  void setRobotsPosition(Matrix<double, 6, 1> joints) {
    // update robots ee position to robots frame
    // update rotation matrix to robots frame
    // update joint positions

    Matrix<double, 4, 4> rototranslation;
    Matrix<double, 3, 3> rotation;

    this->current_position = joints;

    rototranslation = ur5_direct(joints);
    rotation << rototranslation.col(0).head(3), rototranslation.col(1).head(3), rototranslation.col(2).head(3);
    this-> R = rotation;

    this->ee_coords = rototranslation.col(3).head(3);
  }

  /*!
  * Metodo che ritorna le velocità da applicare all'end effector dati una serie di parametri
  */
  Matrix<double, 6, 1> Qdot(Matrix<double, 6, 1> joint_states, Vector3d desired_pos, Vector3d desired_pos_vel, Vector3d desired_RPY, Vector3d desired_RPY_velocities, Matrix<double, 3, 3> Kq, Matrix<double, 3, 3> KRPY)
  {
    Matrix<double, 6, 1> Qdot;
    Vector3d current_end_eff_pos; // relative to robot's frame
    Vector3d current_RPY;         // relative to robot's frame
    Matrix<double, 4, 4> dir_rot;
    Matrix<double, 3, 3> rot;
    Matrix<double, 6, 6> jac;

    dir_rot = ur5_direct(joint_states);                                            // matrice 4, 4 rispetto al robot
    rot << dir_rot.row(0).head(3), dir_rot.row(1).head(3), dir_rot.row(2).head(3); // matrice di rotazione del robot rispetto al frame del robot
    current_end_eff_pos = dir_rot.col(3).head(3);
    current_RPY = fromRotToRPY(rot);

    jac = jacobian(joint_states);
    jac = geometric_to_analytical(jac, current_RPY);

    EigenSolver<Matrix<double, 6, 6>> es;
    es.compute(jac, false);
    Matrix<double, 6, 1> eigenvalues = es.eigenvalues().real();

    double min_eigenvalue;
    min_eigenvalue = eigenvalues.minCoeff();
    if (abs(min_eigenvalue) < 0.2)
    {
      cout << "d" << endl;
      jac = dampedJacobian(jac, 0.1);
    }

    Eigen::FullPivLU<Matrix<double, 6, 6>> lu(jac);

    if (lu.isInvertible())
    {
      Matrix<double, 6, 1> t;
      t << desired_pos_vel + Kq * (desired_pos - current_end_eff_pos), desired_RPY_velocities + KRPY * (desired_RPY - current_RPY);
      Qdot = jac.inverse() * t;
      return Qdot;
    }
    else
    {
      Matrix<double, 6, 1> stop;
      stop << 0, 0, 0, 0, 0, 0;
      return stop;
    }
  }

  Vector3d fromUr5ToWorld(Vector3d p)
  {
    Vector4d homogeneous_p;
    homogeneous_p << p, 1;
    Vector3d Urd5Coords(0.501, 0.352, 1.754);
    Matrix<double, 4, 4> r;

    r << 1, 0, 0, Urd5Coords(0),
    0, -1, 0, Urd5Coords(1),
    0, 0, -1, Urd5Coords(2),
    0, 0, 0, 1;
    return (r * homogeneous_p).head(3);
  }

  Vector3d fromWorldToUr5(Vector3d p)
  {
    Vector3d Ur5Coords(0.501, 0.352, 1.754);
    Vector4d homogeneous_p;
    homogeneous_p << p, 1;
    Matrix<double, 4, 4> r;
    r << 1, 0, 0, -Ur5Coords(0),
    0, -1, 0, Ur5Coords(1),
    0, 0, -1, Ur5Coords(2),
    0, 0, 0, 1;

    return (r * homogeneous_p).head(3);
  }

  /*!
  * computa la matrice di rototraslazione corrispondente alla mano del robot
  */
  Matrix<double, 4, 4> ur5_direct(Matrix<double, 6, 1> joint_values)
  {
    double th1 = joint_values(0);
    double th2 = joint_values(1);
    double th3 = joint_values(2);
    double th4 = joint_values(3);
    double th5 = joint_values(4);
    double th6 = joint_values(5);

    // first matrix, rotation around z and translation about z
    Matrix<double, 4, 4> T10;
    T10 << cos(th1), -sin(th1), 0, 0,
    sin(th1), cos(th1), 0, 0,
    0, 0, 1, D(0),
    0, 0, 0, 1;

    // second matrix, variable rotation around z and -pi/2 rotation around x
    Matrix<double, 4, 4> T21;
    T21 << cos(th2), -sin(th2), 0, 0,
    0, 0, -1, 0,
    sin(th2), cos(th2), 0, 0,
    0, 0, 0, 1;

    // third matrix
    Matrix<double, 4, 4> T32;
    T32 << cos(th3), -sin(th3), 0, A(1),
    sin(th3), cos(th3), 0, 0,
    0, 0, 1, D(2),
    0, 0, 0, 1;

    // fourth matrix
    Matrix<double, 4, 4> T43;
    T43 << cos(th4), -sin(th4), 0, A(2),
    sin(th4), cos(th4), 0, 0,
    0, 0, 1, D(3),
    0, 0, 0, 1;
    // fifth matrix
    Matrix<double, 4, 4> T54;
    T54 << cos(th5), -sin(th5), 0, 0,
    0, 0, -1, -D(4),
    sin(th5), cos(th5), 0, 0,
    0, 0, 0, 1;
    // sixth matrix
    Matrix<double, 4, 4> T65;
    T65 << cos(th6), -sin(th6), 0, 0,
    0, 0, 1, D(5),
    -sin(th6), -cos(th6), 0, 0,
    0, 0, 0, 1;


    Matrix<double, 4, 4> T;
    T = T10 * T21 * T32 * T43 * T54 * T65;

    return T;
  }

  Vector3d linear_interpol(Vector3d pi, Vector3d pf, double t)
  {
    return t * (pf) + (1 - t) * pi;
  }

  Vector3d trapezoidal_velocity(Vector3d pi, Vector3d pf, double t) {
    Vector3d acc = (pf - pi) * 16 / 3;
    Vector3d vel = acc/4;

    if(t >= 1) {
      return Vector3d(0, 0, 0);
    }

    if(t <= 0.25) {
      return acc * t;
    }

    if(t > 0.25 && t <= 0.75) {
      return vel;
    }

    if(t > 0.75 && t < 1) {
      return vel - acc * (t - 0.75);
    }

    return Vector3d::Zero();
  }

  Vector3d linear_velocity(Vector3d pi, Vector3d pf) {
    return (pf - pi);
  }

  bool check_collisions(Vector3d pi_coords, Vector3d pf_coords)  {
    Vector3d rpy(0, 0, pi);
    Matrix<double, 6, 1> qk;
    int nsamples = 2;
    double Dt = (double)1 / nsamples;
    double t = Dt;
    vector<Matrix<double, 6, 1>> samples;

    qk = current_position;
    while(t <= 1) {
      qk += Qdot(qk, linear_interpol(pi_coords, pf_coords, t), linear_velocity(pi_coords, pf_coords), rpy , Matrix<double, 3, 1>::Zero(), Matrix<double, 3, 3>::Identity() * 0.1, Matrix<double, 3, 3>::Identity() * 0.1) * Dt;
      samples.push_back(qk);

      t+=Dt;
    }

    int k = 0;
    for(auto sample: samples) {
      vector<Vector3d> robot_joint_coords = get_joints_positions(sample);
      for(int i = 2; i < 6; i++) {
        if(check_world_collisions(robot_joint_coords[i])) {
          cout << "failed for sample " << k << endl;
          cout << "failed for joint " << i << ", at coords:" << endl << robot_joint_coords[i] << endl;
          return true;
        }
      }
      k++;
    }

    return false;
  }

  double close_to_collisions(Vector3d pos1, Vector3d pos2, Matrix<double, 6, 1> joints) {
   const double MURO = 0.1;
   const double BLOCCO = 0.2;
   const double TAVOLO_ALTO = 1.75;
   const double TAVOLO_BASSO = 0.87;
   const double ALTEZZA_MURO = 1;
   const int SCALE = 10;
   const int nsamples = 10;

   double cost = 0;
   double threshold = 1000;

   // correct
   Matrix<double, 6, 1> q; q = joints;

   Vector3d pos1_rcoords; pos1_rcoords = fromWorldToUr5(pos1);
   Vector3d pos2_rcoords; pos2_rcoords = fromWorldToUr5(pos2);

   vector<Matrix<double, 6, 1>> sampled_joints(nsamples-1); // we start from t = dt, like in the move_to function

   for(int i = 1; i < nsamples; i++) {
    q += Qdot(q, linear_interpol(pos1_rcoords, pos2_rcoords, (double)i/nsamples), linear_velocity(pos1_rcoords, pos2_rcoords), Vector3d(0, 0, pi), Vector3d::Zero(), Matrix3d::Identity() * 0.1, Matrix3d::Identity() * 0.1) * (double)1/nsamples;
    
    vector<Vector3d> joint_positions = get_joints_positions(q);
    for(int i = 2; i < 6; i++) {
      cost += abs(joint_positions[i](1) - MURO);
      if(joint_positions[i](2) < ALTEZZA_MURO) cost += abs(joint_positions[i](1) - BLOCCO);
      cost += abs(joint_positions[i](2) - TAVOLO_BASSO);
      cost += abs(joint_positions[i][2] - TAVOLO_ALTO);
    }
  } 

  //debugging: check sampling
  // cout << "in diffKin:" << endl;
  // cout << "start wcoords: " << pos1(0) << ", " << pos1(1) << endl;
  // cout << "start jcoords" << endl << joints << endl << endl;

  // cout << "end wcoords: " << pos2(0) << ", " << pos2(1) << endl;
  // cout << "end jcoords:" << endl << q << endl << endl << endl;

  if(cost > threshold) return 0;
  return (double) 100 / (cost);
}


private:
  Vector3d fromRotToRPY(Matrix<double, 3, 3> R) // radianti
  {
    double roll, pitch, yaw;
    const double pi = 2 * acos(0.0);
    const double epsilon = 0.01;
    //TODO: use abs(diff) < threshold
    if((R(0, 0) >= R(0, 1) - epsilon && R(0, 0) <= R(0, 1) + epsilon) && (R(0, 0) < epsilon && R(0, 0) > -epsilon)) {
      yaw = 0;
      pitch = pi/2;
      roll = atan2(R(0, 1), R(1, 1));
    } else {
      yaw = atan2(R(1, 0), R(0, 0));
      pitch = atan2(-R(2, 0), sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
      roll = atan2(R(2, 1), R(2, 2));
    }

    return Vector3d(roll, pitch, yaw);
  }

  /*!
  * Computa la matrice di rotazione specificati gli angoli di roll, pitch e yaw
  */
  Matrix<double, 3, 3> getRotationMatrix(Vector3d m)
  {
    Matrix<double, 3, 3> x;
    Matrix<double, 3, 3> y;
    Matrix<double, 3, 3> z;
    x << 1, 0, 0,
    0, cos(m(0)), -sin(m(0)),
    0, sin(m(0)), cos(m(0));
    y << cos(m(1)), 0, sin(m(1)),
    0, 1, 0,
    -sin(m(1)), 0, cos(m(1));
    z << cos(m(2)), -sin(m(2)), 0,
    sin(m(2)), cos(m(2)), 0,
    0, 0, 1;
    return z * y * x;
  }

  /*!
  * Rappresentazione della matrice di rotazione valida per ogni joint, secondo i parametri di Denavit-Hartenberg
  */
  Matrix<double, 4, 4> rot(double alpha, double a, double d, double theta)
  {
    Matrix<double, 4, 4> T;

    T << cos(theta), -sin(theta), 0, a,
    sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d,
    sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), cos(alpha) * d,
    0, 0, 0, 1;

    return T;
  }

  Vector3d cross_prod(Vector3d u, Vector3d v)
  {
    return Vector3d(u(1) * v(2) - u(2) * v(1), u(2) * v(0) - u(0) * v(2), u(0) * v(1) - u(1) * v(0));
  }

  /*!
  * Computa la matrice jacobiana per una data configurazione dei giunti
  */
  Matrix<double, 6, 6> jacobian(Matrix<double, 6, 1> joint_values)
  {
    // first matrix, rotation around z and translation about z
    Matrix<double, 4, 4> T10;
    T10 = rot(0, 0, D(0), joint_values(0));

    // second matrix, variable rotation around z and pi/2 rotation around x
    Matrix<double, 4, 4> T21;
    T21 = rot(pi / 2, 0, 0, joint_values(1));

    Matrix<double, 4, 4> T32;
    T32 = rot(0, A(1), 0, joint_values(2));

    // fourth matrix
    Matrix<double, 4, 4> T43;
    T43 = rot(0, A(2), D(3), joint_values(3));

    // fifth matrix
    Matrix<double, 4, 4> T54;
    T54 = rot(pi / 2, 0, D(4), joint_values(4));

    // sixth matrix
    Matrix<double, 4, 4> T65;
    T65 = rot(-pi / 2, 0, D(5), joint_values(5));

    Vector3d p1, p2, p3, p4, p5, p6;
    Vector3d z1, z2, z3, z4, z5, z6;

    p1 << T10(0, 3), T10(1, 3), T10(2, 3);
    Matrix<double, 4, 4> T20 = T10 * T21;
    p2 << T20(0, 3), T20(1, 3), T20(2, 3);
    Matrix<double, 4, 4> T30 = T20 * T32;
    p3 << T30(0, 3), T30(1, 3), T30(2, 3);
    Matrix<double, 4, 4> T40 = T30 * T43;
    p4 << T40(0, 3), T40(1, 3), T40(2, 3);
    Matrix<double, 4, 4> T50 = T40 * T54;
    p5 << T50(0, 3), T50(1, 3), T50(2, 3);
    Matrix<double, 4, 4> T60 = T50 * T65;
    p6 << T60(0, 3), T60(1, 3), T60(2, 3);

    z1 = T10.col(2).head(3);
    z2 = T20.col(2).head(3);
    z6 = T60.col(2).head(3);
    z3 = T30.col(2).head(3);
    z4 = T40.col(2).head(3);
    z5 = T50.col(2).head(3);

    // z1 << T10(0, 2), T10(1, 2), T10(2, 2);
    // z2 << T20(0, 2), T20(1, 2), T20(2, 2);
    // z3 << T30(0, 2), T30(1, 2), T30(2, 2);
    // z4 << T40(0, 2), T40(1, 2), T40(2, 2);
    // z5 << T50(0, 2), T50(1, 2), T50(2, 2);
    // z6 << T60(0, 2), T60(1, 2), T60(2, 2);

    Matrix<double, 6, 1> column1;
    column1 << cross_prod(z1, p6 - p1), z1;

    Matrix<double, 6, 1> column2;
    column2 << cross_prod(z2, p6 - p2), z2;

    Matrix<double, 6, 1> column3;
    column3 << cross_prod(z3, p6 - p3), z3;

    Matrix<double, 6, 1> column4;
    column4 << cross_prod(z4, p6 - p4), z4;

    Matrix<double, 6, 1> column5;
    column5 << cross_prod(z5, p6 - p5), z5;

    Matrix<double, 6, 1> column6;
    column6 << cross_prod(z6, p6 - p6), z6;

    Matrix<double, 6, 6> jac;
    jac << column1,
    column2,
    column3,
    column4,
    column5,
    column6;
    return jac;
  }

  Matrix<double, 6, 6> dampedJacobian(Matrix<double, 6, 6> jac, double damping_term)
  {
    Matrix<double, 6, 6> dampedJac;
    Matrix<double, 6, 6> damping_matrix;

    damping_matrix = Matrix<double, 6, 6>::Identity() * damping_term * damping_term;
    dampedJac = jac.transpose() * (jac * jac.transpose() + damping_matrix);
    return dampedJac;
  }

  Matrix<double, 6, 6> geometric_to_analytical(Matrix<double, 6, 6> jacobian, Vector3d RPY_current) // insert rpy_current as roll, pitch, yaw
  {
    //[ok]
    double roll, pitch, yaw;

    roll = RPY_current(0);
    pitch = RPY_current(1);
    yaw = RPY_current(2);

    Matrix<double, 3, 3> T;
    T << cos(pitch) * cos(roll), -sin(roll), 0,
    cos(pitch) * sin(roll), cos(roll), 0,
    -sin(pitch), 0, 1;


    Matrix<double, 6, 6> Ta;
    Ta << Matrix3d::Identity(), Matrix3d::Zero(),
    Matrix3d::Zero(), T;

    jacobian = Ta.inverse() * jacobian;
    return jacobian;
  }

  vector<Vector3d> get_joints_positions(Matrix<double, 6, 1> joints)
  {
    Matrix<double, 4, 4> T10 = rot(0, 0, D[0], joints[0]);
    Matrix<double, 4, 4> T21 = rot(pi / 2, 0, 0, joints[1]);
    Matrix<double, 4, 4> T32 = rot(0, A[1], 0, joints[2]);
    Matrix<double, 4, 4> T43 = rot(0, A[2], D[3], joints[3]);
    Matrix<double, 4, 4> T54 = rot(pi / 2, 0, D[4], joints[4]);
    Matrix<double, 4, 4> T65 = rot(-pi / 2, 0, D[5], joints[5]);
    Vector4d homogeneous_zero;
    homogeneous_zero << 0, 0, 0, 1;

    vector<Vector3d> res;
    res.push_back((T10 * homogeneous_zero).head(3));
    res.push_back((T10 * T21 * homogeneous_zero).head(3));
    res.push_back((T10 * T21 * T32 * homogeneous_zero).head(3));
    res.push_back((T10 * T21 * T32 * T43 * homogeneous_zero).head(3));
    res.push_back((T10 * T21 * T32 * T43 * T54 * homogeneous_zero).head(3));
    res.push_back((T10 * T21 * T32 * T43 * T54 * T65 * homogeneous_zero).head(3));

    return res;
  }

  bool check_world_collisions(Vector3d robot_coords) {
    const double MURO = 0.1;
    const double BLOCCO = 0.2;
    const double TAVOLO_ALTO = 1.75;
    const double TAVOLO_BASSO = 0.87;
    const double ALTEZZA_MURO = 1;
    Vector3d world_coords;
    world_coords = fromUr5ToWorld(robot_coords);

    // collisione contro il muro, contro il blocchetto, contro il tavolo in basso, contro il tavolo in alto
    if(world_coords(1) < MURO || (world_coords(1) < MURO && world_coords(2) < ALTEZZA_MURO) || world_coords(2) < TAVOLO_BASSO || world_coords(2) > TAVOLO_ALTO) return true;

    return false;
  }

};
