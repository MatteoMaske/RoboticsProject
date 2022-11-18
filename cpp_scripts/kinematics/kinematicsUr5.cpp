#include <iostream>
#include <Eigen>
#include <cmath>

using namespace std;
using Eigen::MatrixXf;
using Eigen::Vector4d;

//distance vectors
const float A[6] = {0, -0.425, -0.3922, 0, 0, 0};
const float D[6] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};

MatrixXf Re(3,3);

//rotation matrix for each joint
MatrixXf A10(4,4);
MatrixXf A21(4, 4);
MatrixXf A32(4, 4);
MatrixXf A43(4, 4);
MatrixXf A54(4, 4);
MatrixXf A65(4, 4);
MatrixXf A60(4, 4);

void fwKin(float Th[6], float endEffectorPos[3]); // This function will calculate the forward kinematics of the robot and return the position of the end effector
void invKin(float endEffectorPos[3]); // This function will calculate the inverse kinematics of the robot and return the joint angles

//calculates rotation matrix for each joint
void calcA10(float th0);
void calcA21(float th1);
void calcA32(float th2);
void calcA43(float th3);
void calcA54(float th4);
void calcA65(float th5);

int main(int argc, char** argv){

    float Th[6] = {1.6, 0.2, -0.5, 2.89, 1.1, 1.25};
    float endEffectorPos[3] = {0, 0, 0}; // This will be the position of the end effector

    fwKin(Th, endEffectorPos);

    invKin(endEffectorPos);

    //cout << "The end effector is at: " << endEffectorPos[0] << ", " << endEffectorPos[1] << ", " << endEffectorPos[2] << endl;

    return 0;
}

void calcA10(float Th0){
    A10 <<  cos(Th0), -sin(Th0), 0, 0,
            sin(Th0), cos(Th0), 0, 0,
            0, 0, 1, D[0],
            0, 0, 0, 1;
}

void calcA21(float Th1){
    A21 <<  cos(Th1), -sin(Th1), 0, 0,
            0, 0, -1, 0,
            sin(Th1), cos(Th1), 0, 0,
            0, 0, 0, 1;
}

void calcA32(float Th2){
    A32 <<  cos(Th2), -sin(Th2), 0, A[1],
            sin(Th2), cos(Th2), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
}

void calcA43(float Th3){
    A43 <<  cos(Th3), -sin(Th3), 0, A[2],
            sin(Th3), cos(Th3), 0, 0,
            0, 0, 1, D[3],
            0, 0, 0, 1;
}

void calcA54(float Th4){
    A54 <<  cos(Th4), -sin(Th4), 0, 0,
            0, 0, -1, -D[4],
            sin(Th4), cos(Th4), 0, 0,
            0, 0, 0, 1;
}

void calcA65(float Th5){
    A65 <<  cos(Th5), -sin(Th5), 0, 0,
            0, 0, 1, D[5],
            -sin(Th5), -cos(Th5), 0, 0,
            0, 0, 0, 1;
}

void fwKin(float Th[6], float endEffectorPos[3]){

    MatrixXf Pe(1,4);

    calcA10(Th[0]);
    calcA21(Th[1]);
    calcA32(Th[2]);
    calcA43(Th[3]);
    calcA54(Th[4]);
    calcA65(Th[5]);

    A60 = A10*A21*A32*A43*A54*A65;

    Pe = A60.block(0,3,3,1);
    Re = A60.block(0,0,3,3);

    //cout << Pe << endl;
    //cout << Re << endl;

    endEffectorPos[0] = Pe(0, 0);
    endEffectorPos[1] = Pe(1, 0);
    endEffectorPos[2] = Pe(2, 0);
}

void invKin(float endEffectorPos[3]){

    MatrixXf T60(4, 4);

    T60 << Re(0, 0), Re(0, 1), Re(0, 2), endEffectorPos[0],
        Re(1, 0), Re(1, 1), Re(1, 2), endEffectorPos[1],
        Re(2, 0), Re(2, 1), Re(2, 2), endEffectorPos[2],
        0, 0, 0, 1;

    // finding th1
    MatrixXf p50(1, 4);
    MatrixXf temp(4, 1);
    temp << 0, 0, -D[5], 1;

    p50 = T60 * temp;
    float th1_1 = atan2(p50(1, 0), p50(0, 0)) + acos(D[3] / hypot(p50(1, 0), p50(0, 0))) + M_PI_2;
    float th1_2 = atan2(p50(1, 0), p50(0, 0)) - acos(D[3] / hypot(p50(1, 0), p50(0, 0))) + M_PI_2;

    float th5_1 = acos((endEffectorPos[0] * sin(th1_1) - endEffectorPos[1] * cos(th1_1) - D[3]) / D[5]);
    float th5_2 = -acos((endEffectorPos[0] * sin(th1_1) - endEffectorPos[1] * cos(th1_1) - D[3]) / D[5]);
    float th5_3 = acos((endEffectorPos[0] * sin(th1_2) - endEffectorPos[1] * cos(th1_2) - D[3]) / D[5]);
    float th5_4 = -acos((endEffectorPos[0] * sin(th1_2) - endEffectorPos[1] * cos(th1_2) - D[3]) / D[5]);

    // related to th11 a th51
    MatrixXf T06(4, 4);
    MatrixXf Xhat(3, 1);
    MatrixXf Yhat(3, 1);

    T06 = T60.inverse();
    Xhat = T06.block(0, 0, 3, 1);
    Yhat = T06.block(0, 1, 3, 1);

    float th6_1 = atan2(((-Xhat(1) * sin(th1_1) + Yhat(1) * cos(th1_1))) / sin(th5_1), ((Xhat(0) * sin(th1_1) - Yhat(0) * cos(th1_1))) / sin(th5_1));
    // related to th11 a th52
    float th6_2 = atan2(((-Xhat(1) * sin(th1_1) + Yhat(1) * cos(th1_1))) / sin(th5_2), ((Xhat(0) * sin(th1_1) - Yhat(0) * cos(th1_1))) / sin(th5_2));
    // related to th12 a th53
    float th6_3 = atan2(((-Xhat(1) * sin(th1_2) + Yhat(1) * cos(th1_2))) / sin(th5_3), ((Xhat(0) * sin(th1_2) - Yhat(0) * cos(th1_2))) / sin(th5_3));
    // related to th12 a th54
    float th6_4 = atan2(((-Xhat(1) * sin(th1_2) + Yhat(1) * cos(th1_2))) / sin(th5_4), ((Xhat(0) * sin(th1_2) - Yhat(0) * cos(th1_2))) / sin(th5_4));

    MatrixXf T41m(4, 4);
    MatrixXf p41_1(1, 4);
    MatrixXf p41_2(1, 4);
    MatrixXf p41_3(1, 4);
    MatrixXf p41_4(1, 4);

    calcA10(th1_1); calcA65(th6_1); calcA54(th5_1);
    T41m = A10.inverse() * T60 * A65.inverse() * A54.inverse();
    p41_1 = T41m.block(0, 3, 3, 1);
    float p41xz_1 = hypot(p41_1(0), p41_1(2));

    calcA10(th1_1); calcA65(th6_2); calcA54(th5_2);
    T41m = A10.inverse() * T60 * A65.inverse() * A54.inverse();
    p41_2 = T41m.block(0, 3, 3, 1);
    float p41xz_2 = hypot(p41_2(0), p41_2(2));

    calcA10(th1_2); calcA65(th6_3); calcA54(th5_3);
    T41m = A10.inverse() * T60 * A65.inverse() * A54.inverse();
    p41_3 = T41m.block(0, 3, 3, 1);
    float p41xz_3 = hypot(p41_3(0), p41_3(2));

    calcA10(th1_2); calcA65(th6_4); calcA54(th5_4);
    T41m = A10.inverse() * T60 * A65.inverse() * A54.inverse();
    p41_4 = T41m.block(0, 3, 3, 1);
    float p41xz_4 = hypot(p41_4(0), p41_4(2));

    //Computation of the 8 possible values for th3
    float th3_1 = acos((pow(p41xz_1, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2]));
    float th3_2 = acos((pow(p41xz_2, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2]));
    float th3_3 = acos((pow(p41xz_3, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2]));
    float th3_4 = acos((pow(p41xz_4, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2]));

    float th3_5 = -th3_1;
    float th3_6 = -th3_2;
    float th3_7 = -th3_3;
    float th3_8 = -th3_4;

    //Computation of eight possible value for th2
    float th2_1 = atan2(-p41_1(2), -p41_1(0)) - asin((-A[2] * sin(th3_1)) / p41xz_1);
    float th2_2 = atan2(-p41_2(2), -p41_2(0)) - asin((-A[2] * sin(th3_2)) / p41xz_2);
    float th2_3 = atan2(-p41_3(2), -p41_3(0)) - asin((-A[2] * sin(th3_3)) / p41xz_3);
    float th2_4 = atan2(-p41_4(2), -p41_4(0)) - asin((-A[2] * sin(th3_4)) / p41xz_4);

    float th2_5 = atan2(-p41_1(2), -p41_1(0)) - asin((A[2] * sin(th3_1)) / p41xz_1);
    float th2_6 = atan2(-p41_2(2), -p41_2(0)) - asin((A[2] * sin(th3_2)) / p41xz_2);
    float th2_7 = atan2(-p41_3(2), -p41_3(0)) - asin((A[2] * sin(th3_3)) / p41xz_3);
    float th2_8 = atan2(-p41_4(2), -p41_4(0)) - asin((A[2] * sin(th3_4)) / p41xz_4);

    //riga 119 matlab
}
