#include <iostream>
#include <cmath>
#include "Eigen/Core"
#include "Eigen/Geometry"

using namespace std;
using namespace Eigen;
const float pi = 2 * acos(0.0);

Matrix<float, 6, 1> A;
Matrix<float, 6, 1> D;

Matrix<float, 4, 4> rot(float alpha, float a, float d, float theta) {
		Matrix<float, 4, 4> T;

	 	T << 
		  cos(theta), -sin(theta), 0, a,
		  sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha),  -sin(alpha) * d,
		  sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d,
		  0, 0, 0, 1;

		return T;
}

Matrix<float, 8, 6> inverse_kin(Vector3f p60, Matrix<float, 3, 3>mat) {
		// p60 = posizione corrente
		// mat = matrice di rotazione

		//TH1
		Matrix<float, 4, 4> T60;

		T60 << mat(0, 0), mat(0, 1), mat(0, 2), p60(0),
			mat(1, 0), mat(1,1),  mat(1,2),  p60(1),
			mat(2,0),  mat(2,1),  mat(2,2),  p60(2),
			0, 0, 0, 1;

		Matrix<float, 4, 1> p50 = T60 * Vector4f(0, 0, -D(5), 1);

		float th1_1 = atan2(p50(1), p50(0)) + acos(D(3)/hypot(p50(1), p50(0)))+pi/2;
		float th1_2 = atan2(p50(1), p50(0)) - acos(D(3)/hypot(p50(1), p50(0)))+pi/2;

		//TH5
		Matrix<float, 4, 4> T10;

		T10 = rot(0, 0, D(0), th1_1);

		float th5_1 = acos(((p60(0)*sin(th1_1) - p60(1)*cos(th1_1)-D(3))/D(5)));
		float th5_2 = -acos(((p60(0)*sin(th1_1) - p60(1)*cos(th1_1)-D(3))/D(5)));
		float th5_3 = acos(((p60(0)*sin(th1_2) - p60(1)*cos(th1_2)-D(3))/D(5)));
		float th5_4 = -acos(((p60(0)*sin(th1_2) - p60(1)*cos(th1_2)-D(3))/D(5)));


		//TH6
		Matrix<float, 4, 4> T06 = T60.inverse();
		Vector3f Xhat(T06(0, 0), T06(1, 0), T06(2, 0));
		Vector3f Yhat(T06(0, 1), T06(1, 1), T06(2, 1));


		//change: th6, atan2's first argument was: ((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1)))/sin(th5_1)
		//adopted from matlab as: th6_1 = ((-Xhat(2)*sin(th1_1)+Yhat(2)*cos(th1_1)))/sin(th5_1)
		float th6_1 = atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1)))/sin(th5_1), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1)))/sin(th5_1));
		float th6_2 = atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1))/sin(th5_2)), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1))/sin(th5_2)));
		float th6_3 = atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2))/sin(th5_3)), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2))/sin(th5_3)));
		float th6_4 = atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2))/sin(th5_4)), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2))/sin(th5_4)));

		//TH3
		Matrix<float, 4, 4> T65;
		T65 = rot(-pi/2, 0, D(5), th6_1);

		Matrix<float, 4, 4> T54;
		T54 =  rot(pi/2, 0, D(4), th5_1);

		Matrix<float, 4, 4> T41m;

		T41m = T10.inverse() * T60 * T65.inverse() * T54.inverse();
		//cout << "T41m, used for p41_1" << endl << T41m << endl << endl;
		cout << "fattori: " << endl << "T10" << endl << T10.inverse() << endl << endl << "T60:" << T60 << endl << endl << "T65.inv" << endl << T65.inverse() << endl << endl << "T54.inv" << endl << T54.inverse() << endl << endl; 
		cout << "T41m" << T41m << endl << endl;
		Vector3f p41_1(T41m(0, 3), T41m(1, 3), T41m(2, 3));
		float p41xz_1 = hypot(p41_1(0), p41_1(2));
		float th3_1 = acos((p41xz_1 * p41xz_1 - A(1) * A(1) - A(2) * A(2))/(2*A(1)*A(2)));

		T10 = rot(0, 0, D(0), th1_1); T65 = rot(-pi/2, 0, D(5), th6_2); T54 = rot(pi/2, 0, D(4), th5_2);

		T41m = T10.inverse() * T60 * T65.inverse() * T54.inverse();
		//cout << "T41m, used for p41_2" << endl << T41m << endl << endl;
		Vector3f p41_2(T41m(0, 3), T41m(1, 3), T41m(2, 3));
		float p41xz_2 = hypot(p41_2(0), p41_2(2));
		float th3_2 = (acos((p41xz_2 * p41xz_2 - A(1) * A(1) - A(2) * A(2))/(2*A(1)*A(2))));

		T10 = rot(0, 0, D(0), th1_1); T65 = rot(-pi/2, 0, D(5), th6_3); T54 = rot(pi/2, 0, D(4), th5_3);


		T41m = T10.inverse() * T60 * T65.inverse() * T54.inverse();
		//cout << "T41m, used for p41_3" << endl << T41m << endl << endl;
		
		Vector3f p41_3(T41m(0, 3), T41m(1, 3), T41m(2, 3));
		float p41xz_3 = hypot(p41_2(0), p41_2(2));
		float th3_3 = (acos((p41xz_3 * p41xz_3 - A(1) * A(1) - A(2) * A(2))/(2*A(1)*A(2))));

		T10 = rot(0, 0, D(0), th1_2); T65 = rot(-pi/2, 0, D(5), th6_4); T54 = rot(pi/2, 0, D(4), th5_4);
		T41m = T10.inverse() * T60 * T65.inverse() * T54.inverse();

		cout << "fattori: " << endl << "T10" << endl << T10.inverse() << endl << endl << "T60:" << T60 << endl << endl << "T65.inv" << endl << T65.inverse() << endl << endl << "T54.inv" << endl << T54.inverse() << endl << endl; 
		cout << "T41m" << T41m << endl << endl;
		Vector3f p41_4(T41m(0, 3), T41m(1, 3), T41m(2, 3));
		//cout << "T41m, used for p41_4" << T41m << endl << endl;
		float p41xz_4 = hypot(p41_2(0), p41_2(2));
		float th3_4 = (acos((p41xz_4 * p41xz_4 - A(1) * A(1) - A(2) * A(2))/(2*A(1)*A(2))));


		float th3_5 = -th3_1;
		float th3_6 = -th3_2;
		float th3_7 = -th3_3;
		float th3_8 = -th3_4;

		//CALCOLANDO TH2
		float th2_1 = (atan2(-p41_1(2), -p41_1(0))-asin((-A(2)*sin(th3_1))/p41xz_1));
		float th2_2 =  (atan2(-p41_2(2), -p41_2(0))-asin((-A(2)*sin(th3_2))/p41xz_2));
		//argomenti: p41_2, th3_2(corretto), p41xz_2 -> p41_2
		float th2_3 = (atan2(-p41_3(2), -p41_3(0))-asin((-A(2)*sin(th3_3))/p41xz_3));
		float th2_4 = (atan2(-p41_4(2), -p41_4(0))-asin((-A(2)*sin(th3_4))/p41xz_4));
		float th2_5 = (atan2(-p41_1(2), -p41_1(0))-asin((A(2)*sin(th3_1))/p41xz_1));
		float th2_6 = (atan2(-p41_2(2), -p41_2(0))-asin((A(2)*sin(th3_2))/p41xz_2));
		float th2_7 = (atan2(-p41_3(2), -p41_3(0))-asin((A(2)*sin(th3_3))/p41xz_3));
		float th2_8 = (atan2(-p41_4(2), -p41_4(0))-asin((A(2)*sin(th3_4))/p41xz_4));

		//CALCOLANDO TH4
		Matrix<float, 4, 4> T32;
		T32 = rot(0, A(1), 0, th3_1);
		Matrix<float, 4, 4> T21;
		T21 = rot(pi/2, 0, 0, th2_1);
		Matrix<float, 4, 4> T43;
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();

		Vector3f Xhat43(T43(0, 0), T43(1, 0), T43(2, 0));
		float th4_1 = (atan2(Xhat43(1), Xhat43(0)));

		T32 = rot(0, A(1), 0, th3_2);
		T21 = rot(pi/2, 0, 0, th2_2);
		T10 = rot(0, 0, D(0), th1_1);
		T65 = rot(-pi/2, 0, D(5), th6_2);
		T54 = rot(pi/2, 0, D(4), th5_2);
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Xhat43 = Vector3f(T43(0, 0), T43(1, 0), T43(2, 0));
		float th4_2 = (atan2(Xhat43(1), Xhat43(0)));

		T32 = rot(0, A(1), 0, th3_3);
		T21 = rot(pi/2, 0, 0, th2_3);
		T10 = rot(0, 0, D(0), th1_2);
		T65 = rot(-pi/2, 0, D(5), th6_3);
		T54 = rot(pi/2, 0, D(4), th5_3);
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Xhat43 = Vector3f(T43(0, 0), T43(1, 0), T43(2, 0));
		float th4_3 = (atan2(Xhat43(1), Xhat43(0)));

		T32 = rot(0, A(1), 0, th3_4);
		T21 = rot(pi/2, 0, 0, th2_4);
		T10 = rot(0, 0, D(0), th1_2);
		T65 = rot(-pi/2, 0, D(5), th6_4);
		T54 = rot(pi/2, 0, D(4), th5_4);
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Xhat43 = Vector3f(T43(0, 0), T43(1, 0), T43(2, 0));
		float th4_4 = (atan2(Xhat43(1), Xhat43(0)));

		T32 = rot(0, A(1), 0, th3_5);
		T21 = rot(pi/2, 0, 0, th2_5);
		T10 = rot(0, 0, D(0), th1_1);
		T65 = rot(-pi/2, 0, D(5), th6_1);
		T54 = rot(pi/2, 0, D(4), th5_1);
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Xhat43 = Vector3f(T43(0, 0), T43(1, 0), T43(2, 0));
		float th4_5 = (atan2(Xhat43(1), Xhat43(0)));

		T32 = rot(0, A(1), 0, th3_6);
		T21 = rot(pi/2, 0, 0, th2_6);
		T10 = rot(0, 0, D(0), th1_1);
		T65 = rot(-pi/2, 0, D(5), th6_2);
		T54 = rot(pi/2, 0, D(4), th5_2);
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Xhat43 = Vector3f(T43(0, 0), T43(1, 0), T43(2, 0));
		float th4_6 = (atan2(Xhat43(1), Xhat43(0)));

		T32 = rot(0, A(1), 0, th3_7);
		T21 = rot(pi/2, 0, 0, th2_7);
		T10 = rot(0, 0, D(0), th1_2);
		T65 = rot(-pi/2, 0, D(5), th6_3);
		T54 = rot(pi/2, 0, D(4), th5_3);
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Xhat43 = Vector3f(T43(0, 0), T43(1, 0), T43(2, 0));
		float th4_7 = (atan2(Xhat43(1), Xhat43(0)));

		T32 = rot(0, A(1), 0, th3_8);
		T21 = rot(pi/2, 0, 0, th2_8);
		T10 = rot(0, 0, D(0), th1_2);
		T65 = rot(-pi/2, 0, D(5), th6_4);
		T54 = rot(pi/2, 0, D(4), th5_4);
		T43 = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
		Xhat43 = Vector3f(T43(0, 0), T43(1, 0), T43(2, 0));
		float th4_8 = (atan2(Xhat43(1), Xhat43(0)));

		Matrix<float, 8, 6> joint_states;
		joint_states <<
				th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
				th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
				th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
				th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
				th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
				th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
				th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
				th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;

		return joint_states;
}


int main() {
	/*
		angoli che non vanno:
		th2_2, 3, 4, 6, 7, 8
		th3_2, 3, 4, 6, 7, 8
		th4_2, 3, 4, 6, 7, 8
	*/	
	A << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0;
	D << 0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996;
	Matrix<float, 3, 3> R;
	Vector3f point;
	point << 0.2, 0.3, 0.;
		R << 0.0175,    0.9702,    0.2416,
		  -0.9960,    0.0381,   -0.0809,
		  -0.0877,   -0.2392,    0.9670;

		(0.1518,
		 -0.1908,
		 0.4550);

		cout << "joint_states: " << endl << inverse_kin(point, R) << endl;

		// th2_2, th2_3, th2_4, th2_7, th2_8
		// th3 praticamente tutti
		// th4: 1, 3, 4, 6, 7, 8

		Matrix<float, 8, 6> expected_results;
		expected_results <<
				1.6000 ,  -0.2795 ,   0.5000 ,   2.3695 ,   1.1000 ,   1.2500,
				1.6000 ,  -0.7674 ,   1.0088 ,  -0.7930 ,  -1.1000 ,  -1.8916,
				-1.1847,   -3.3786,    1.0779,    3.0022,    2.3326,   -1.6358,
				-1.1847,   -3.2623,    0.3559,    0.4664,   -2.3326,    1.5058,
				1.6000 ,   0.2000 ,  -0.5000 ,   2.8900 ,   1.1000 ,   1.2500,
				1.6000 ,   0.1971 ,  -1.0088 ,   0.2601 ,  -1.1000 ,  -1.8916,
				-1.1847,   -2.3486,   -1.0779,   -2.1550,    2.3326,   -1.6358,
				-1.1847,   -2.9208,   -0.3559,    0.8367,   -2.3326,    1.5058;

		expected_results << 
			3.8083,   -0.2441,    2.2983,    2.8017,    1.7854,    2.5187,
    		3.8083,   -0.2829,    1.8044,    0.1929,   -1.7854,   -0.6229,
    		1.4371,    1.7422,    1.8333,    1.1864,    1.3179,   -1.4151,
    		1.4371,    1.2405,    2.2614,   -1.8815,   -1.3179,    1.7265,
    		3.8083,    1.8758,   -2.2983,   -1.0046,    1.7854,    2.5187,
    		3.8083,    1.4200,   -1.8044,    2.0989,   -1.7854,   -0.6229,
    		1.4371,    3.4709,   -1.8333,    3.1244,    1.3179,   -1.4151,
    		1.4371,    3.3318,   -2.2614,    0.5499,   -1.3179,    1.7265;

		// expected_results << 
		//  3.8083,    0.3194,    2.6150,   2.4853,    1.7854,    2.5187,
     	//  3.8083,   -0.0599,    1.0548,   1.2833,   -1.7854,   -0.6229,
     	//  1.4371,    1.2258,    2.2698,   1.3467,    1.3179,   -1.4151,
     	//  1.4371,    1.7536,    1.8203,  -1.8731,   -1.3179,    1.7265,
     	//  3.8083,    2.6388,   -2.6150,  -0.8872,    1.7854,    2.5187,
     	//  3.8083,    0.9481,   -1.0548,   2.3848,   -1.7854,   -0.6229,
     	//  1.4371,    3.3237,   -2.2698,  -2.4948,    1.3179,   -1.4151,
     	//  1.4371,    3.4706,   -1.8203,   0.0504,   -1.3179,    1.7265;



		Matrix<float, 8, 6> differences;
		differences = inverse_kin(point, R) - expected_results;

		cout << "differences: " << endl << differences << endl;

		return 0;
}


