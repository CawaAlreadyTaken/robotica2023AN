#include <iostream>
#include <cmath>
#include "eigen-git-mirror/Eigen/Core"
#include "eigen-git-mirror/Eigen/Geometry"

using namespace std;
using namespace Eigen;

/*!
 * Classe per computare la soluzione analitica dell'inverse kinematic del braccio UR5, adattata dal paper: https://smartech.gatech.edu/bitstream/handle/1853/50782/ur_kin_tech_report_1.pdf?sequence=1
 */

class InverseKinematic
{

	Matrix<double, 3, 3> R;
	Vector3d point;
	Matrix<double, 4, 4> T;
	double pi;
	Matrix<double, 6, 1> A;
	Matrix<double, 6, 1> D;
	double _gripper;

public:
	/*!
	 * Inizializza alcuni parametri noti, come le lunghezze del robot.
	 */
	InverseKinematic()
	{
		pi = 2 * acos(0.0);
		A << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0;
		D << 0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996;
	}

	/*!
	 * Inizializza attributi della classi quali: punto da raggiungere, matrice di rotazione e valore del gripper.
	 */
	void setDestinationPoint(Vector3d p, Vector3d m, double g)
	{
		_gripper = g;
		R = getRotationMatrix(m);
		point = p;
	}

	int scelta(Matrix<double, 6, 1> q0, Matrix<double, 8, 6> joint_configurations)
	{
		if(!check_collisions(q0, joint_configurations.row(5))); return 5;
		for (int i = 1; i < 7; i++)
		{
			if (!check_collisions(q0, joint_configurations.row(i)))
			{
				return i;
			}
		}

		return -1;
	}

	bool check_collisions(Matrix<double, 6, 1> q0, Matrix<double, 6, 1> q1)
	{
		int i = 0;
		double ts[100];
		Matrix<double, 6, 1> samples[100];
		for (int i = 0; i < 100; i++)
		{
			ts[i] = 0.01 * i;
		}

		// get 100 samples
		for (int i = 0; i < 100; i++)
		{
			samples[i] = line(q0, q1, ts[i]);
		}

		for (int i = 0; i < 100; i++)
		{
			// for each sample
			Matrix<double, 4, 4> T10 = rot(0, 0, D[0], samples[i][0]);
			Matrix<double, 4, 4> T21 = rot(pi / 2, 0, 0, samples[i][1]);
			Matrix<double, 4, 4> T32 = rot(0, A[1], 0, samples[i][2]);
			Matrix<double, 4, 4> T43 = rot(0, A[2], D[3], samples[i][3]);
			Matrix<double, 4, 4> T54 = rot(pi / 2, 0, D[4], samples[i][4]);
			Matrix<double, 4, 4> T65 = rot(-pi / 2, 0, D[5], samples[i][5]);

			Vector3d joints[6];
			Vector4d unit;
			unit << 0, 0, 0, 1;
			joints[0] = (T10 * unit).head(3);
			joints[1] = (T10 * T21 * unit).head(3);
			joints[2] = (T10 * T21 * T32 * unit).head(3);
			joints[3] = (T10 * T21 * T32 * T43 * unit).head(3);
			joints[4] = (T10 * T21 * T32 * T43 * T54 * unit).head(3);
			joints[5] = (T10 * T21 * T32 * T43 * T54 * T65 * unit).head(3);

			for (int j = 0; j < 6; j++)
			{

				if (joints[j][0] < -0.85 || joints[j][0] > 0.85)
				{
					return true;
				}

				if (joints[j][1] < -0.9 || joints[j][1] > 0.2)
				{
					return true;
				}

				if (joints[j][2] < 0 || joints[j][2] > 0.7)
				{
					return true;
				}
			}
		}

		return false;
	}

	/*!
	 * Ritorna la prima soluzione dell'inverse kinematics che non collide con il workspace.
	 */
	void getJointsPositions(Eigen::Matrix<double, 9, 1> &q_des0)
	{
		Matrix<double, 8, 6> result = inverse_kin(point, R);
		int choice = scelta(q_des0.head(6), result);
		q_des0 << result(choice, 0), result(choice, 1), result(choice, 2), result(choice, 3), result(choice, 4), result(choice, 5), _gripper, _gripper, _gripper;
	}

	/*!
	 * Ritorna coordinate secondo il frame del robot date coordinate secondo il frame del mondo.
	 */
	Vector3d fromWorldToUrd5(Vector3d p)
	{
		std::cout << "p: " << p << std::endl;
		Vector3d Urd5Coords(0.501, 0.352, 1.754);
		Vector3d result = p - Urd5Coords;
		result(0) = result(0);
		result(1) = -result(1);
		result(2) = -result(2);
		// std::cout << "ToUrd5Coords: " << p-Urd5Coords << std::endl;
		// double a, b, c; std::cin >> a >> b >> c;
		// return Vector3d(a, b, c);
		std::cout << result << endl;
		return result;
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
	Matrix<double, 6, 1> line(Matrix<double, 6, 1> q0, Matrix<double, 6, 1> q1, double t)
	{
		return (1 - t) * q0 + t * q1;
	}

private:
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

	/*!
	 * Computa la matrice di rototranslazione che rappresenta la cinematica diretta del braccio UR5
	 */
	Matrix<double, 4, 1> ur5_direct(Matrix<double, 6, 1> joint_values)
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
			0, 0, 1, -D(4),
			sin(th5), cos(th5), 0, 0,
			0, 0, 0, 1;
		// sixth matrix
		Matrix<double, 4, 4> T65;
		T65 << cos(th6), -sin(th6), 0, 0,
			0, 0, 1, D(5),
			-sin(th6), -cos(th6), 0, 0,
			0, 0, 0, 1;

		T = T10 * T21 * T32 * T43 * T54 * T65;

		Matrix<double, 4, 1> unit(0, 0, 0, 1);

		return T * unit;
	}

	/*!
	 * Computa le 8 possibili configurazioni dei giunti tali per cui l'end effector del braccio si troverà nel punto specificato, con orientamento dato dalla matrice di rotazione.
	 */
	Matrix<double, 8, 6> inverse_kin(Vector3d p60, Matrix<double, 3, 3> mat)
	{
		// p60 = posizione corrente
		// mat = matrice di rotazione

		// TH1
		Matrix<double, 4, 4> T60;

		T60 << mat(0, 0), mat(0, 1), mat(0, 2), p60(0),
			mat(1, 0), mat(1, 1), mat(1, 2), p60(1),
			mat(2, 0), mat(2, 1), mat(2, 2), p60(2),
			0, 0, 0, 1;

		Matrix<double, 4, 1> p50 = T60 * Vector4d(0, 0, -D(5), 1);

		double th1_1 = atan2(p50(1), p50(0)) + acos(D(3) / hypot(p50(1), p50(0))) + pi / 2;
		double th1_2 = atan2(p50(1), p50(0)) - acos(D(3) / hypot(p50(1), p50(0))) + pi / 2;

		// TH5
		Matrix<double, 4, 4> T10;

		T10 = rot(0, 0, D(0), th1_1);

		double th5_1 = acos(((p60(0) * sin(th1_1) - p60(1) * cos(th1_1) - D(3)) / D(5)));
		double th5_2 = -acos(((p60(0) * sin(th1_1) - p60(1) * cos(th1_1) - D(3)) / D(5)));
		double th5_3 = acos(((p60(0) * sin(th1_2) - p60(1) * cos(th1_2) - D(3)) / D(5)));
		double th5_4 = -acos(((p60(0) * sin(th1_2) - p60(1) * cos(th1_2) - D(3)) / D(5)));

		// TH6
		Matrix<double, 4, 4> T06 = T60.inverse();
		Vector3d Xhat(T06(0, 0), T06(1, 0), T06(2, 0));
		Vector3d Yhat(T06(0, 1), T06(1, 1), T06(2, 1));

		double th6_1 = atan2(((-Xhat(1) * sin(th1_1) + Yhat(1) * cos(th1_1))) / sin(th5_1), ((Xhat(0) * sin(th1_1) - Yhat(0) * cos(th1_1))) / sin(th5_1));
		double th6_2 = atan2(((-Xhat(1) * sin(th1_1) + Yhat(1) * cos(th1_1)) / sin(th5_2)), ((Xhat(0) * sin(th1_1) - Yhat(0) * cos(th1_1)) / sin(th5_2)));
		double th6_3 = atan2(((-Xhat(1) * sin(th1_2) + Yhat(1) * cos(th1_2)) / sin(th5_3)), ((Xhat(0) * sin(th1_2) - Yhat(0) * cos(th1_2)) / sin(th5_3)));
		double th6_4 = atan2(((-Xhat(1) * sin(th1_2) + Yhat(1) * cos(th1_2)) / sin(th5_4)), ((Xhat(0) * sin(th1_2) - Yhat(0) * cos(th1_2)) / sin(th5_4)));

		// TH3
		Matrix<double, 4, 4> T65;
		T65 = rot(-pi / 2, 0, D(5), th6_1);

		Matrix<double, 4, 4> T54;
		T54 = rot(pi / 2, 0, D(4), th5_1);
		Matrix<double, 4, 4> T41m;
		T41m = T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Vector3d p41_1(T41m(0, 3), T41m(1, 3), T41m(2, 3));
		double p41xz_1 = hypot(p41_1(0), p41_1(2));
		double th3_1 = acos((p41xz_1 * p41xz_1 - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2)));

		T10 = rot(0, 0, D(0), th1_1);
		T65 = rot(-pi / 2, 0, D(5), th6_2);
		T54 = rot(pi / 2, 0, D(4), th5_2);

		T41m = T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Vector3d p41_2(T41m(0, 3), T41m(1, 3), T41m(2, 3));
		double p41xz_2 = hypot(p41_2(0), p41_2(2));
		double th3_2 = (acos((p41xz_2 * p41xz_2 - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2))));

		T10 = rot(0, 0, D(0), th1_2);
		T65 = rot(-pi / 2, 0, D(5), th6_3);
		T54 = rot(pi / 2, 0, D(4), th5_3);
		T41m = T10.inverse() * T60 * T65.inverse() * T54.inverse();

		Vector3d p41_3(T41m(0, 3), T41m(1, 3), T41m(2, 3));
		double p41xz_3 = hypot(p41_3(0), p41_3(2));
		double th3_3 = (acos((p41xz_3 * p41xz_3 - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2))));

		T10 = rot(0, 0, D(0), th1_2);
		T65 = rot(-pi / 2, 0, D(5), th6_4);
		T54 = rot(pi / 2, 0, D(4), th5_4);
		T41m = T10.inverse() * T60 * T65.inverse() * T54.inverse();

		Vector3d p41_4(T41m(0, 3), T41m(1, 3), T41m(2, 3));
		double p41xz_4 = hypot(p41_4(0), p41_4(2));
		double th3_4 = (acos((p41xz_4 * p41xz_4 - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2))));

		double th3_5 = -th3_1;
		double th3_6 = -th3_2;
		double th3_7 = -th3_3;
		double th3_8 = -th3_4;

		// CALCOLANDO TH2
		double th2_1 = (atan2(-p41_1(2), -p41_1(0)) - asin((-A(2) * sin(th3_1)) / p41xz_1));
		double th2_2 = (atan2(-p41_2(2), -p41_2(0)) - asin((-A(2) * sin(th3_2)) / p41xz_2));
		double th2_3 = (atan2(-p41_3(2), -p41_3(0)) - asin((-A(2) * sin(th3_3)) / p41xz_3));
		double th2_4 = (atan2(-p41_4(2), -p41_4(0)) - asin((-A(2) * sin(th3_4)) / p41xz_4));
		double th2_5 = (atan2(-p41_1(2), -p41_1(0)) - asin((A(2) * sin(th3_1)) / p41xz_1));
		double th2_6 = (atan2(-p41_2(2), -p41_2(0)) - asin((A(2) * sin(th3_2)) / p41xz_2));
		double th2_7 = (atan2(-p41_3(2), -p41_3(0)) - asin((A(2) * sin(th3_3)) / p41xz_3));
		double th2_8 = (atan2(-p41_4(2), -p41_4(0)) - asin((A(2) * sin(th3_4)) / p41xz_4));

		// CALCOLANDO TH4
		Matrix<double, 4, 4> T32;
		T32 = rot(0, A(1), 0, th3_1);
		Matrix<double, 4, 4> T21;
		T21 = rot(pi / 2, 0, 0, th2_1);
		Matrix<double, 4, 4> T43;
		T10 = rot(0, 0, D(0), th1_1);
		T65 = rot(-pi / 2, 0, D(5), th6_1);
		T54 = rot(pi / 2, 0, D(4), th5_1);
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();

		Vector3d Xhat43(T43(0, 0), T43(1, 0), T43(2, 0));
		double th4_1 = (atan2(Xhat43(1), Xhat43(0)));

		T32 = rot(0, A(1), 0, th3_2);
		T21 = rot(pi / 2, 0, 0, th2_2);
		T10 = rot(0, 0, D(0), th1_1);
		T65 = rot(-pi / 2, 0, D(5), th6_2);
		T54 = rot(pi / 2, 0, D(4), th5_2);
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Xhat43 = Vector3d(T43(0, 0), T43(1, 0), T43(2, 0));
		double th4_2 = (atan2(Xhat43(1), Xhat43(0)));

		T32 = rot(0, A(1), 0, th3_3);
		T21 = rot(pi / 2, 0, 0, th2_3);
		T10 = rot(0, 0, D(0), th1_2);
		T65 = rot(-pi / 2, 0, D(5), th6_3);
		T54 = rot(pi / 2, 0, D(4), th5_3);
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Xhat43 = Vector3d(T43(0, 0), T43(1, 0), T43(2, 0));
		double th4_3 = (atan2(Xhat43(1), Xhat43(0)));

		T32 = rot(0, A(1), 0, th3_4);
		T21 = rot(pi / 2, 0, 0, th2_4);
		T10 = rot(0, 0, D(0), th1_2);
		T65 = rot(-pi / 2, 0, D(5), th6_4);
		T54 = rot(pi / 2, 0, D(4), th5_4);
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Xhat43 = Vector3d(T43(0, 0), T43(1, 0), T43(2, 0));
		double th4_4 = (atan2(Xhat43(1), Xhat43(0)));

		T32 = rot(0, A(1), 0, th3_5);
		T21 = rot(pi / 2, 0, 0, th2_5);
		T10 = rot(0, 0, D(0), th1_1);
		T65 = rot(-pi / 2, 0, D(5), th6_1);
		T54 = rot(pi / 2, 0, D(4), th5_1);
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Xhat43 = Vector3d(T43(0, 0), T43(1, 0), T43(2, 0));
		double th4_5 = (atan2(Xhat43(1), Xhat43(0)));

		T32 = rot(0, A(1), 0, th3_6);
		T21 = rot(pi / 2, 0, 0, th2_6);
		T10 = rot(0, 0, D(0), th1_1);
		T65 = rot(-pi / 2, 0, D(5), th6_2);
		T54 = rot(pi / 2, 0, D(4), th5_2);
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Xhat43 = Vector3d(T43(0, 0), T43(1, 0), T43(2, 0));
		double th4_6 = (atan2(Xhat43(1), Xhat43(0)));

		T32 = rot(0, A(1), 0, th3_7);
		T21 = rot(pi / 2, 0, 0, th2_7);
		T10 = rot(0, 0, D(0), th1_2);
		T65 = rot(-pi / 2, 0, D(5), th6_3);
		T54 = rot(pi / 2, 0, D(4), th5_3);
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Xhat43 = Vector3d(T43(0, 0), T43(1, 0), T43(2, 0));
		double th4_7 = (atan2(Xhat43(1), Xhat43(0)));

		T32 = rot(0, A(1), 0, th3_8);
		T21 = rot(pi / 2, 0, 0, th2_8);
		T10 = rot(0, 0, D(0), th1_2);
		T65 = rot(-pi / 2, 0, D(5), th6_4);
		T54 = rot(pi / 2, 0, D(4), th5_4);
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Xhat43 = Vector3d(T43(0, 0), T43(1, 0), T43(2, 0));
		double th4_8 = (atan2(Xhat43(1), Xhat43(0)));

		Matrix<double, 8, 6> joint_states;
		joint_states << th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
			th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
			th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
			th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
			th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
			th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
			th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
			th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;

		return joint_states;
	}
};
