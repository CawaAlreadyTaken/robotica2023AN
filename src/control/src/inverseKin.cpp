#include <iostream>
#include <cmath>
#include "eigen-git-mirror/Eigen/Core"
#include "eigen-git-mirror/Eigen/Geometry"

using namespace std;
using namespace Eigen;

class InverseKinematic {

        Matrix<float, 3, 3> R;
        Vector3f point;
        Matrix<float, 4, 4> T;
        double pi;
        Matrix<float, 6, 1> A;
        Matrix<float, 6, 1> D;
        float _gripper;

        public: 
        InverseKinematic() {
                pi = 2 * acos(0.0);
                A << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0;
                D << 0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996;
        }

        void setDestinationPoint(Vector3f p,Vector3f m, float g){
                _gripper = g;
                R = getRotationMatrix(m);
                point = p;
        }

        void getJointsPositions(Eigen::Matrix<double, 8, 1> & q_des0) {
                Matrix<float, 8, 6> result = inverse_kin(point, R);
                // TODO: Far la scelta tra le 8 opzioni
                int scelta = 7;
                q_des0 << result(scelta, 0), result(scelta, 1), result(scelta, 2), result(scelta, 3), result(scelta, 4), result(scelta, 5), _gripper, _gripper; 
        }

        Vector3f fromWorldToUrd5(Vector3f p){
                std::cout << "p: " << p << std::endl;
                Vector3f Urd5Coords(0.501, 0.352, 1.754);
                Vector3f result = p-Urd5Coords;
                result(0) = result(0);
                result(1) = -result(1);
                result(2) = -result(2);
                // std::cout << "ToUrd5Coords: " << p-Urd5Coords << std::endl;
                // float a, b, c; std::cin >> a >> b >> c;
                // return Vector3f(a, b, c);
                std::cout << result << endl;
                return result;
        }

        Matrix<float, 3, 3> getRotationMatrix(Vector3f m) {
                Matrix<float, 3, 3> x;
                Matrix<float, 3, 3> y;
                Matrix<float, 3, 3> z;
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

        private:
        Matrix<float, 4, 4> rot(float alpha, float a, float d, float theta) {
                Matrix<float, 4, 4> T;

                T << cos(theta), -sin(theta), 0, a,
                        sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha),  -sin(alpha) * d,
                        sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d,
                        0, 0, 0, 1;

                return T;
        }

        Matrix<float, 4, 1> ur5_direct(Matrix<float, 6, 1> joint_values) {
                float th1 = joint_values(0);
                float th2 = joint_values(1);
                float th3 = joint_values(2);
                float th4 = joint_values(3);
                float th5 = joint_values(4);
                float th6 = joint_values(5);

                //first matrix, rotation around z and translation about z
                Matrix<float, 4, 4> T10;
                T10 <<  cos(th1), -sin(th1), 0, 0,
                        sin(th1), cos(th1), 0, 0,
                        0, 0, 1, D(0),
                        0, 0, 0, 1;

                // cout << "T10: " << endl << T10 << "T10_rot: " << endl << rot(0, 0, D(0), th1) << endl;
                // cout << "diff10: " << endl << T10 - rot(0, 0, D(0), th1) << endl;

                //second matrix, variable rotation around z and -pi/2 rotation around x
                Matrix<float, 4, 4> T21;
                T21 <<  cos(th2), -sin(th2), 0, 0,
                        0, 0, -1, 0,
                        sin(th2), cos(th2), 0, 0,
                        0, 0, 0, 1;

                // cout << "T21: " << endl << T21 << "T21_rot: " << endl << rot(pi/2, 0, 0, th2) << endl;
                // cout << "diff21: " << endl << T21 - rot(pi/2, 0, 0, th2) << endl;

                //third matrix
                Matrix<float, 4, 4> T32;
                T32 <<  cos(th3), -sin(th3), 0, A(1),
                        sin(th3), cos(th3), 0, 0,
                        0, 0, 1, D(2),
                        0, 0, 0, 1;

                // cout << "T32: " << endl << T32 << "T32_rot: " << endl << rot(0, A(1), 0, th3) << endl;
                // cout << "diff32: " << endl << T32 - rot(0, A(1), 0, th3) << endl;

                //fourth matrix
                Matrix<float, 4, 4> T43;
                T43 <<  cos(th4), -sin(th4), 0, A(2),
                        sin(th4), cos(th4), 0, 0,
                        0, 0, 1, D(3),
                        0, 0, 0, 1;
                // cout << "T43: " << endl << T43 << "T43_rot: " << endl << rot(0, A(2), D(3), th4) << endl;
                // cout << "diff43: " << endl << T43 - rot(0, A(2), D(3), th4) << endl;
                //fifth matrix
                Matrix<float, 4, 4> T54;
                T54 <<  cos(th5), -sin(th5), 0, 0,
                        0, 0, 1, -D(4),
                        sin(th5), cos(th5), 0, 0,
                        0, 0, 0, 1;
                // cout << "T54: " << endl << T54 << "T10_rot: " << endl << rot(pi/2, 0, D(4), th5) << endl;
                // cout << "diff54: " << endl << T54 - rot(pi/2, 0, D(4), th5) << endl;

                //sixth matrix
                Matrix<float, 4, 4> T65;
                T65 <<  cos(th6), -sin(th6), 0, 0,
                        0, 0, 1, D(5),
                        -sin(th6), -cos(th6), 0, 0,
                        0, 0, 0, 1;
                // `  cout << "T65: " << endl << T65 <<"T10_rot: " << endl << rot(-pi/2, 0, D(5), th6) << endl;
                //   cout << "diff65: " << endl << T65 - rot(-pi/2, 0, D(5), th6) << endl;`

                T = T10 * T21 * T32 * T43 * T54 * T65;

                Matrix<float, 4, 1> unit(0, 0, 0, 1);

                return T * unit;
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
                Vector3f p41_1(T41m(0, 3), T41m(1, 3), T41m(2, 3));
                float p41xz_1 = hypot(p41_1(0), p41_1(2));
                float th3_1 = acos((p41xz_1 * p41xz_1 - A(1) * A(1) - A(2) * A(2))/(2*A(1)*A(2)));

                T10 = rot(0, 0, D(0), th1_1); T65 = rot(pi/2, 0, D(4), th6_2); T54 = rot(-pi/2, 0, D(4), th5_2);

                T41m = T10.inverse() * T60 * T65.inverse() * T54.inverse();
                Vector3f p41_2(T41m(0, 3), T41m(1, 3), T41m(2, 3));
                float p41xz_2 = hypot(p41_2(0), p41_2(2));
                float th3_2 = (acos((p41xz_2 * p41xz_2 - A(1) * A(1) - A(2) * A(2))/(2*A(1)*A(2))));

                T10 = rot(0, 0, D(0), th1_1); T65 = rot(pi/2, 0, D(4), th6_3); T54 = rot(-pi/2, 0, D(5), th5_3);


                T41m = T10.inverse() * T60 * T65.inverse() * T54.inverse();

                Vector3f p41_3(T41m(0, 3), T41m(1, 3), T41m(2, 3));
                float p41xz_3 = hypot(p41_2(0), p41_2(2));
                float th3_3 = (acos((p41xz_3 * p41xz_3 - A(1) * A(1) - A(2) * A(2))/(2*A(1)*A(2))));

                T10 = rot(0, 0, D(0), th1_2); T65 = rot(pi/2, 0, D(4), th6_4); T54 = rot(-pi/2, 0, D(5), th5_4);
                T41m = T10.inverse() * T60 * T65.inverse() * T54.inverse();

                Vector3f p41_4(T41m(0, 3), T41m(1, 3), T41m(2, 3));
                float p41xz_4 = hypot(p41_2(0), p41_2(2));
                float th3_4 = (acos((p41xz_4 * p41xz_4 - A(1) * A(1) - A(2) * A(2))/(2*A(1)*A(2))));


                float th3_5 = -th3_1;
                float th3_6 = -th3_2;
                float th3_7 = -th3_3;
                float th3_8 = -th3_4;

                //CALCOLANDO TH2
                float th2_1 = (atan2(-p41_1(2), -p41_1(0))-asin((-A(2)*sin(th3_1))/p41xz_1));
                float th2_2 =  (atan2(-p41_2(2), -p41_2(0))-asin((-A(2)*sin(th3_2))/p41xz_2));
                float th2_3 = (atan2(-p41_3(2), -p41_3(0))-asin((-A(2)*sin(th3_3))/p41xz_3));
                float th2_4 = (atan2(-p41_4(2), -p41_4(0))-asin((-A(2)*sin(th3_4))/p41xz_4));
                float th2_5 = (atan2(-p41_1(2), -p41_1(0))-asin((A(2)*sin(th3_1))/p41xz_1));
                float th2_6 = (atan2(-p41_2(2), -p41_2(0))-asin((A(2)*sin(th3_2))/p41xz_2));
                float th2_7 = (atan2(-p41_3(2), -p41_3(0))-asin((A(2)*sin(th3_3))/p41xz_3));
                float th2_8 = (atan2(-p41_4(2), -p41_4(0))-asin((A(2)*sin(th3_4))/p41xz_4));

                //cout << p41_1 << endl << p41_2 << endl << p41_3 << endl << p41_4 << endl;

                // p41_2 è sbagliato
                // p41_3 è sbagliato


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

        /*
        int main() {
        
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

                        /*
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


                Matrix<float, 8, 6> differences;
                differences = inverse_kin(point, R) - expected_results;

                cout << "differences: " << endl << differences << endl;
        
                return 0;
        }*/

};