#include <custom_joint_publisher.h>
#include <math.h>
#include <vision/custMsg.h>

#include "inverseKin.cpp"
#include "differentialKin.cpp"

InverseKinematic ik; 
DifferentialKinematic dk;

Matrix<double, 4, 4> rot(double alpha, double a, double d, double theta)
{
  Matrix<double, 4, 4> T;

  T << 
  cos(theta), -sin(theta), 0, a,
  sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d,
  sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), cos(alpha) * d,
  0, 0, 0, 1;

  return T;
}

// void state(Matrix<double, 6, 1> joints) // ritorna le posizioni dei giunti
// {

//   const double pi = 2 * acos(0.0);

//   Matrix<double, 6, 1> A;
//   Matrix<double, 6, 1> D;
//   A << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0;
//   D << 0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996;

//   Matrix<double, 4, 4> T10 = rot(0, 0, D[0], joints(0)); 
//   Matrix<double, 4, 4> T21 = rot(pi / 2, 0, 0, joints(1));
//   Matrix<double, 4, 4> T32 = rot(0, A[1], 0, joints(2));
//   Matrix<double, 4, 4> T43 = rot(0, A[2], D[3], joints[3]);
//   Matrix<double, 4, 4> T54 = rot(pi / 2, 0, D[4], joints[4]);
//   Matrix<double, 4, 4> T65 = rot(-pi / 2, 0, D[5], joints[5]);
//   Vector4d unit;
//   unit << 0, 0, 0, 1;
//   cout << "j0:" << endl << fromUr5ToWorld((T10 * unit).head(3)) << endl << endl;
//   cout << "j1" << endl << fromUr5ToWorld((T10 * T21 * unit).head(3)) << endl << endl;
//   cout << "j2" << endl << fromUr5ToWorld((T10 * T21 * T32 * unit).head(3)) << endl << endl;
//   cout << "j3" << endl << fromUr5ToWorld((T10 * T21 * T32 * T43 * unit).head(3)) << endl << endl;
//   cout << "j4" << endl << fromUr5ToWorld((T10 * T21 * T32 * T43 * T54 * unit).head(3)) << endl << endl;
//   cout << "j5" << fromUr5ToWorld((T10 * T21 * T32 * T43 * T54 * T65 * unit).head(3)) << endl << endl;
// }

Vector3d fromWorldToUr5(Vector3d p)
{
  Vector3d Ur5Coords(0.501, 0.352, 1.754);
  Vector3d result = p - Ur5Coords;
  Vector4d homogeneous_p; homogeneous_p << p, 1;
  Matrix<double, 4, 4> r;
  r << 
  1, 0, 0, -Ur5Coords(0),
  0, -1, 0, Ur5Coords(1),
  0, 0, -1, Ur5Coords(2),
  0, 0, 0, 1;

  result(0) = result(0);
  result(1) = -result(1);
  result(2) = -result(2);

  return (r * homogeneous_p).head(3);
    // std::cout << "ToUrd5Coords: " << p-Urd5Coords << std::endl;
    // double a, b, c; std::cin >> a >> b >> c;
    // return Vector3d(a, b, c);
  return result;
}

Vector3d fromUr5ToWorld(Vector3d p) 
{
  Vector4d homogeneous_p;
  homogeneous_p << p, 1;
  Vector3d Urd5Coords(0.501, 0.352, 1.754);
  Matrix<double, 4, 4> r;

  r << 
  1, 0,  0, Urd5Coords(0),
  0, -1, 0, Urd5Coords(1),
  0, 0, -1, Urd5Coords(2),
  0, 0, 0, 1;
  return (r * homogeneous_p).head(3);
}

int main() {
  const double pi = 2 * acos(0.0);

  Vector3d start; start << 0, 4, 1;

  cout << fromUr5ToWorld(fromWorldToUr5(start)) - start << endl;

  return 0;
}