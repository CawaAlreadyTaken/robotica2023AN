#include <custom_joint_publisher.h>
#include <math.h>
#include <vision/custMsg.h>
#include <time.h>

#include "diffKinLibrary.cpp"

using namespace std;
using namespace Eigen;

const double pi = 2 * acos(0.0);

void test_standard_angles() {
  const double pi_3 = (double)1/3;
  const double pi_4 = (double)1/4;
  const double pi_6 = (double)1/6;
  Vector3d standard_angles; standard_angles << pi_3, pi_4, pi_6;

  Vector3d angles;
  Vector3d returned_angles;
  for(int i = 0; i < 3; i++) {
    angles = Vector3d(standard_angles(i), standard_angles((i + 1)%3), standard_angles((i + 2)%3));
    cout << "diff:" << endl << fromRotToRPY(getRotationMatrix(angles))- angles << endl;
  }
}

void test_random_angles() {
  Vector3d angles;
  Vector3d returned_angles;

  const double max = 2;
  const double min = 0;

  for(int i = 0; i < 3; i++) {
    angles = Vector3d(((double)rand()/numeric_limits<int>::max()) * (max - min) + min, ((double)rand()/numeric_limits<int>::max()) * (max - min) + min, ((double)rand()/numeric_limits<int>::max()) * (max - min) + min);
    cout << "angles:" << endl << angles << endl;
    cout << "returned_angles:" << endl << fromRotToRPY(getRotationMatrix(angles)) << endl;
    cout << "diff:" << endl << fromRotToRPY(getRotationMatrix(angles))- angles << endl;

    cout << "matrix diff:" << endl << getRotationMatrix(fromRotToRPY(getRotationMatrix(angles))) - getRotationMatrix(angles) << endl;
  }
}

int main() {
  srand(time(NULL));
  Matrix<double, 6, 1> q;
  q << 2, 0.6, 0.7, 1.8, 1.5, 0.5;
  Matrix<double, 3, 1> xe; xe << 1, 2, 3;

  Matrix<double, 3, 1> xd; xd << 1, 2, 3;

  Matrix<double, 3, 1> vd; vd << 1, 2, 3;

  Matrix<double, 3, 1> phie; phie << (double)1/3, (double)1/6, 0;

  Matrix<double, 3, 1> phid; phid << 1, 2, 3;

  Matrix<double, 3, 1> phiddot; phiddot << 1, 2, 3;

  Matrix<double, 3, 3> Kq;
  Kq << 0.1, 0, 0,
        0, 0.1, 0,
        0, 0, 0.1;

  Matrix<double, 3, 3> Kphi;
  Kphi << 0.1, 0, 0,
          0, 0.1, 0,
          0, 0, 0.1;
  Matrix<double, 6, 6> jac;
  jac = jacobian(q);

  jac = geometric_to_analytical(jac, phie);
  cout << jac << endl;

  Matrix<double, 3, 3> rot;
  rot = getRotationMatrix(phie);

  Vector3d rpy;
  rpy = fromRotToRPY(rot);

  cout << "angles from rot difference:" << endl << phie - rpy << endl;

  Matrix<double, 6, 1> matlab;
  matlab <<
  4.7801,
  -12.2232,
   22.8140,
   -9.0552,
   -2.1950,
   -1.8870;

  // Matrix<double, 6, 1> Qdot(Matrix<double, 6, 1> joint_states, Vector3d desired_pos, Vector3d desired_pos_vel, Vector3d desired_RPY, Vector3d desired_RPY_velocities, Matrix<double, 3, 3> Kq, Matrix<double, 3, 3> KRPY)
  cout << "matlab phie" << endl << "-1.6180, 0.4966, 0.3358" << endl;

  cout << "diffs:" << endl << matlab - Qdot(q, xd, vd, phid, phiddot, Kq, Kphi) << endl;



  return 0;
}
