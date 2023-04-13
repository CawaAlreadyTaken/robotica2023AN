#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace std;
using namespace Eigen;
Matrix<double, 6, 1> A;
Matrix<double, 6, 1> D;

Matrix<double, 6, 6> jacobian(Matrix<double, 6, 1> q) {
    //[ok]

  A << 0, -0.425, -0.3922, 0, 0, 0;
  D << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;


  Matrix<double, 6, 1> J1;
  Matrix<double, 6, 1> J2;
  Matrix<double, 6, 1> J3;
  Matrix<double, 6, 1> J4;
  Matrix<double, 6, 1> J5;
  Matrix<double, 6, 1> J6;
  Matrix<double, 6, 6> J;

  J1 <<
  D(4)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) + D(2)*cos(q(0)) + D(3)*cos(q(0)) - A(2)*cos(q(1) + q(2))*sin(q(0)) - A(1)*cos(q(1))*sin(q(0)) - D(4)*sin(q(1) + q(2) + q(3))*sin(q(0)),
  D(4)*(cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4))) + D(2)*sin(q(0)) + D(3)*sin(q(0)) + A(2)*cos(q(1) + q(2))*cos(q(0)) + A(1)*cos(q(0))*cos(q(1)) + D(4)*sin(q(1) + q(2) + q(3))*cos(q(0)),
  0,
  0,
  0,
  1;
  J2 <<
  -cos(q(0))*(A(2)*sin(q(1) + q(2)) + A(1)*sin(q(1)) + D(4)*(sin(q(1) + q(2))*sin(q(3)) - cos(q(1) + q(2))*cos(q(3))) - D(4)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3)))),
  -sin(q(0))*(A(2)*sin(q(1) + q(2)) + A(1)*sin(q(1)) + D(4)*(sin(q(1) + q(2))*sin(q(3)) - cos(q(1) + q(2))*cos(q(3))) - D(4)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3)))),
  A(2)*cos(q(1) + q(2)) - (D(4)*sin(q(1) + q(2) + q(3) + q(4)))/2 + A(1)*cos(q(1)) + (D(4)*sin(q(1) + q(2) + q(3) - q(4)))/2 + D(4)*sin(q(1) + q(2) + q(3)),
  sin(q(0)),
  -cos(q(0)),
  0;
  J3 <<
  cos(q(0))*(D(4)*cos(q(1) + q(2) + q(3)) - A(2)*sin(q(1) + q(2)) + D(4)*sin(q(1) + q(2) + q(3))*sin(q(4))),
  sin(q(0))*(D(4)*cos(q(1) + q(2) + q(3)) - A(2)*sin(q(1) + q(2)) + D(4)*sin(q(1) + q(2) + q(3))*sin(q(4))),
  A(2)*cos(q(1) + q(2)) - (D(4)*sin(q(1) + q(2) + q(3) + q(4)))/2 + (D(4)*sin(q(1) + q(2) + q(3) - q(4)))/2 + D(4)*sin(q(1) + q(2) + q(3)),
  sin(q(0)),
  -cos(q(0)),
  0;
  J4 <<
  D(4)*cos(q(0))*(cos(q(1) + q(2) + q(3)) + sin(q(1) + q(2) + q(3))*sin(q(4))),
  D(4)*sin(q(0))*(cos(q(1) + q(2) + q(3)) + sin(q(1) + q(2) + q(3))*sin(q(4))),
  D(4)*(sin(q(1) + q(2) + q(3) - q(4))/2 + sin(q(1) + q(2) + q(3)) - sin(q(1) + q(2) + q(3) + q(4))/2),
  sin(q(0)),
  -cos(q(0)),
  0;
  J5 <<
  -D(4)*sin(q(0))*sin(q(4)) - D(4)*cos(q(1) + q(2) + q(3))*cos(q(0))*cos(q(4)),
  D(4)*cos(q(0))*sin(q(4)) - D(4)*cos(q(1) + q(2) + q(3))*cos(q(4))*sin(q(0)),
  -D(4)*(sin(q(1) + q(2) + q(3) - q(4))/2 + sin(q(1) + q(2) + q(3) + q(4))/2),
  sin(q(1) + q(2) + q(3))*cos(q(0)),
  sin(q(1) + q(2) + q(3))*sin(q(0)),
  -cos(q(1) + q(2) + q(3));
  J6 <<
  0,
  0,
  0,
  cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4)),
  -cos(q(0))*cos(q(4)) - cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4)),
  -sin(q(1) + q(2) + q(3))*sin(q(4));
  J << J1, J2, J3, J4, J5, J6;
  return J;

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


Matrix<double, 4, 4> ur5_direct(Matrix<double, 6, 1> joint_values)
{
  // [ok]
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

Matrix<double, 3, 3> getRotationMatrix(Vector3d rpy) { // input in radianti
  double roll = rpy(0);
  double pitch = rpy(1);
  double yaw = rpy(2);

  Matrix<double, 3, 3> rot_x;
  rot_x <<
  1, 0, 0,
  0, cos(roll), -sin(roll),
  0, sin(roll), cos(roll);

  Matrix<double, 3, 3> rot_y;
  rot_y <<
  cos(pitch), 0, sin(pitch),
  0, 1, 0,
  -sin(pitch), 0, cos(pitch);

  Matrix<double, 3, 3> rot_z;
  rot_z <<
  cos(yaw), -sin(yaw), 0,
  sin(yaw), cos(yaw), 0,
  0, 0, 1;

  return rot_z * rot_y * rot_x;
}

Vector3d fromRotToRPY(Matrix<double, 3, 3> R) // ritorna il risultato in radianti
{
  // [ok-ish]

  double roll, pitch, yaw;
  const double pi = 2 * acos(0.0);
  if(R(0, 0) == R(0, 1) && R(0, 0) == 0) {
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

Matrix<double, 6, 6> dampedJacobian(Matrix<double, 6, 6> jac, double damping_term)
{
  // [ok]
  Matrix<double, 6, 6> dampedJac;
  Matrix<double, 6, 6> damping_matrix;

  damping_matrix = Matrix<double, 6, 6>::Identity() * damping_term * damping_term;
  dampedJac = jac.transpose() * (jac * jac.transpose() + damping_matrix);
  return dampedJac;
}

Matrix<double, 6, 1> Qdot(Matrix<double, 6, 1> joint_states, Vector3d desired_pos, Vector3d desired_pos_vel, Vector3d desired_RPY, Vector3d desired_RPY_velocities, Matrix<double, 3, 3> Kq, Matrix<double, 3, 3> KRPY)
{
// [not ok]
  Matrix<double, 6, 1> Qdot;
  Vector3d current_end_eff_pos; // relative to robot's frame
  Vector3d current_RPY;         // relative to robot's frame
  Matrix<double, 4, 4> rototranslation_matrix;
  Matrix<double, 3, 3> rotation_matrix;
  Matrix<double, 6, 6> jac;

  rototranslation_matrix = ur5_direct(joint_states);                                            // matrice 4, 4 rispetto al robot
  rotation_matrix << rototranslation_matrix.row(0).head(3), rototranslation_matrix.row(1).head(3), rototranslation_matrix.row(2).head(3); // matrice di rotazione del robot rispetto al frame del robot
  current_end_eff_pos = rototranslation_matrix.col(3).head(3);                                  // RPY rispetto al frame del robot
  current_RPY = fromRotToRPY(rotation_matrix); // ok

  jac = jacobian(joint_states);
  jac = geometric_to_analytical(jac, current_RPY);

  EigenSolver<Matrix<double, 6, 6>> es;
  es.compute(jac, false);
  Matrix<double, 6, 1> eigenvalues = es.eigenvalues().real();
  double min_eigenvalue;
  min_eigenvalue = eigenvalues.minCoeff();
  if (abs(min_eigenvalue) < 0.1)
  {
    cout << "d" << endl;
    jac = dampedJacobian(jac, 0.1);
  }

  Eigen::FullPivLU<Matrix<double, 6, 6>> lu(jac);
  if (lu.isInvertible())
  {
    Matrix<double, 6, 1> t;
    t << desired_pos_vel + Kq * (desired_pos - current_end_eff_pos), desired_RPY_velocities + KRPY * (desired_RPY - current_RPY);
    cout << "t:" << endl << t << endl;
    Qdot = jac.inverse() * t;
    return Qdot;
  }
  else
  {
    Matrix<double, 6, 1> stop;
    stop << 0, 0, 0, 0, 0, 0;
    return Matrix<double, 6, 1>::Zero();
  }
}

/*
  TODO:
  - fix velocity

  

  - rewrite assignments
*/
