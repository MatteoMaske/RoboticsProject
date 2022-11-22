#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>

using namespace std;
using Eigen::MatrixXf;

//distance vectors
const float A[6] = {0, -0.425, -0.3922, 0, 0, 0};
const float D[6] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};
struct EEPose{
    MatrixXf Pe;
    MatrixXf Re;
};

EEPose fwKin(MatrixXf Th); // This function will calculate the forward kinematics of the robot and return the position of the end effector
MatrixXf invKin(EEPose eePose); // This function will calculate the inverse kinematics of the robot and return the joint angles

//calculates rotation matrix for each joint
MatrixXf calcA10(float th0);
MatrixXf calcA21(float th1);
MatrixXf calcA32(float th2);
MatrixXf calcA43(float th3);
MatrixXf calcA54(float th4);
MatrixXf calcA65(float th5);

/*int main(int argc, char** argv){
    cout.setf(ios::fixed);

    //float Th[6] = {1.6, 0.2, -0.5, 2.89, 1.1, 1.25};
    MatrixXf Th(1,6); Th << 1.6, 0.2, -0.5, 2.89, 1.1, 1.25;
    MatrixXf ThInv(8,6);
    EEPose eePose; EEPose eePose1;

    eePose = fwKin(Th); //calculates forward kinematics
    ThInv = invKin(eePose); //calculates inverse kinematics

    cout << "The end effector is at: " << eePose.Pe(0) << ", " << eePose.Pe(1) << ", " << eePose.Pe(2) << endl;

    // for(int i=0; i<8; i++){
    //     eePose1 = fwKin(ThInv.row(i));
    //     cout << "pe[" << i << "]: " << setprecision(2) << (eePose1.Pe - eePose.Pe) << ", Re[" << i << "]: " << setprecision(6) << (eePose1.Re - eePose.Re) << endl;
    // }

    return 0;
}*/

/**
 * @brief calculates the rotation matrix A10
 * 
 * @param Th0 
 * @return MatrixXf 
 */
MatrixXf calcA10(float Th0){
    MatrixXf A10(4,4);

    A10 <<  cos(Th0), -sin(Th0), 0, 0,
            sin(Th0), cos(Th0), 0, 0,
            0, 0, 1, D[0],
            0, 0, 0, 1;

    return A10;
}

/**
 * @brief calculates the rotation matrix A21
 * 
 * @param Th1 
 * @return MatrixXf 
 */
MatrixXf calcA21(float Th1){
    MatrixXf A21(4,4);

    A21 <<  cos(Th1), -sin(Th1), 0, 0,
            0, 0, -1, 0,
            sin(Th1), cos(Th1), 0, 0,
            0, 0, 0, 1;

    return A21;
}

/**
 * @brief calculates the rotation matrix A32
 * 
 * @param Th2 
 * @return MatrixXf 
 */
MatrixXf calcA32(float Th2){
    MatrixXf A32(4,4);

    A32 <<  cos(Th2), -sin(Th2), 0, A[1],
            sin(Th2), cos(Th2), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return A32;
}

/**
 * @brief calculates the rotation matrix A43
 * 
 * @param Th3 
 * @return MatrixXf 
 */
MatrixXf calcA43(float Th3){
    MatrixXf A43(4,4);

    A43 <<  cos(Th3), -sin(Th3), 0, A[2],
            sin(Th3), cos(Th3), 0, 0,
            0, 0, 1, D[3],
            0, 0, 0, 1;

    return A43;
}

/**
 * @brief calculates the rotation matrix A54
 * 
 * @param Th4 
 * @return MatrixXf 
 */
MatrixXf calcA54(float Th4){
    MatrixXf A54(4,4);

    A54 <<  cos(Th4), -sin(Th4), 0, 0,
            0, 0, -1, -D[4],
            sin(Th4), cos(Th4), 0, 0,
            0, 0, 0, 1;

    return A54;
}

/**
 * @brief calculates the rotation matrix A65
 * 
 * @param Th5 
 * @return MatrixXf 
 */
MatrixXf calcA65(float Th5){
    MatrixXf A65(4,4);

    A65 <<  cos(Th5), -sin(Th5), 0, 0,
            0, 0, 1, D[5],
            -sin(Th5), -cos(Th5), 0, 0,
            0, 0, 0, 1;

    return A65;
}

/**
 * @brief This function will calculate the forward kinematics of the robot and return the position of the end effector
 * 
 * @param Th The joint angles of the robot
 * @return EEPose The position and orientation of the end effector
 */
EEPose fwKin(MatrixXf Th){

    MatrixXf A60(4, 4);
    MatrixXf Re(4,4);
    MatrixXf Pe(1,4);

    A60 = calcA10(Th(0)) * calcA21(Th(1)) * calcA32(Th(2)) * calcA43(Th(3)) * calcA54(Th(4)) * calcA65(Th(5));

    Pe = A60.block(0,3,3,1);
    Re = A60.block(0,0,3,3);

    //cout << Pe << endl;
    //cout << Re << endl;

    EEPose eePose;
    eePose.Pe = Pe;
    eePose.Re = Re;

    return eePose;
}

/**
 * @brief This function will calculate the inverse kinematics of the robot and return the joint angles
 * 
 * @param eePose The position and orientation of the end effector
 * @return MatrixXf The joint angles of the robot
 */
MatrixXf invKin(EEPose eePose){

    MatrixXf Re = eePose.Re;
    MatrixXf T60(4, 4);
    float endEffectorPos[3] = {eePose.Pe(0), eePose.Pe(1), eePose.Pe(2)};

    T60 << Re(0, 0), Re(0, 1), Re(0, 2), endEffectorPos[0],
        Re(1, 0), Re(1, 1), Re(1, 2), endEffectorPos[1],
        Re(2, 0), Re(2, 1), Re(2, 2), endEffectorPos[2],
        0, 0, 0, 1;

    //Computation values for th1
    MatrixXf p50(1, 4);
    MatrixXf temp(4, 1);
    temp << 0, 0, -D[5], 1;

    p50 = T60 * temp;
    float th1_1 = atan2(p50(1, 0), p50(0, 0)) + acos(D[3] / hypot(p50(1, 0), p50(0, 0))) + M_PI_2;
    float th1_2 = atan2(p50(1, 0), p50(0, 0)) - acos(D[3] / hypot(p50(1, 0), p50(0, 0))) + M_PI_2;

    //Computation values for th5
    float th5_1 = acos((endEffectorPos[0] * sin(th1_1) - endEffectorPos[1] * cos(th1_1) - D[3]) / D[5]);
    float th5_2 = -acos((endEffectorPos[0] * sin(th1_1) - endEffectorPos[1] * cos(th1_1) - D[3]) / D[5]);
    float th5_3 = acos((endEffectorPos[0] * sin(th1_2) - endEffectorPos[1] * cos(th1_2) - D[3]) / D[5]);
    float th5_4 = -acos((endEffectorPos[0] * sin(th1_2) - endEffectorPos[1] * cos(th1_2) - D[3]) / D[5]);

    //Computation values for th6
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

    //------------------------
    //cout << calcA10(th1_1) << endl;

    T41m = calcA10(th1_1).inverse() * T60 * calcA65(th6_1).inverse() * calcA54(th5_1).inverse();
    p41_1 = T41m.block(0, 3, 3, 1);
    float p41xz_1 = hypot(p41_1(0), p41_1(2));

    T41m = calcA10(th1_1).inverse() * T60 * calcA65(th6_2).inverse() * calcA54(th5_2).inverse();
    p41_2 = T41m.block(0, 3, 3, 1);
    float p41xz_2 = hypot(p41_2(0), p41_2(2));

    T41m = calcA10(th1_2).inverse() * T60 * calcA65(th6_3).inverse() * calcA54(th5_3).inverse();
    p41_3 = T41m.block(0, 3, 3, 1);
    float p41xz_3 = hypot(p41_3(0), p41_3(2));

    T41m = calcA10(th1_2).inverse() * T60 * calcA65(th6_4).inverse() * calcA54(th5_4).inverse();
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

    //Computation of the 8 possible value for th2
    float th2_1 = atan2(-p41_1(2), -p41_1(0)) - asin((-A[2] * sin(th3_1)) / p41xz_1);
    float th2_2 = atan2(-p41_2(2), -p41_2(0)) - asin((-A[2] * sin(th3_2)) / p41xz_2);
    float th2_3 = atan2(-p41_3(2), -p41_3(0)) - asin((-A[2] * sin(th3_3)) / p41xz_3);
    float th2_4 = atan2(-p41_4(2), -p41_4(0)) - asin((-A[2] * sin(th3_4)) / p41xz_4);

    float th2_5 = atan2(-p41_1(2), -p41_1(0)) - asin((A[2] * sin(th3_1)) / p41xz_1);
    float th2_6 = atan2(-p41_2(2), -p41_2(0)) - asin((A[2] * sin(th3_2)) / p41xz_2);
    float th2_7 = atan2(-p41_3(2), -p41_3(0)) - asin((A[2] * sin(th3_3)) / p41xz_3);
    float th2_8 = atan2(-p41_4(2), -p41_4(0)) - asin((A[2] * sin(th3_4)) / p41xz_4);

    //Computation of the 8 possible value for th4
    MatrixXf T43m(4,4);
    MatrixXf Xhat43(1,4);

    T43m = calcA32(th3_1).inverse() * calcA21(th2_1).inverse() * calcA10(th1_1).inverse() * T60 * calcA65(th6_1).inverse() * calcA54(th5_1).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_1 = atan2(Xhat43(1), Xhat43(0));

    T43m = calcA32(th3_2).inverse() * calcA21(th2_2).inverse() * calcA10(th1_1).inverse() * T60 * calcA65(th6_2).inverse() * calcA54(th5_2).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_2 = atan2(Xhat43(1), Xhat43(0));

    T43m = calcA32(th3_3).inverse() * calcA21(th2_3).inverse() * calcA10(th1_2).inverse() * T60 * calcA65(th6_3).inverse() * calcA54(th5_3).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_3 = atan2(Xhat43(1), Xhat43(0));

    T43m = calcA32(th3_4).inverse() * calcA21(th2_4).inverse() * calcA10(th1_2).inverse() * T60 * calcA65(th6_4).inverse() * calcA54(th5_4).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_4 = atan2(Xhat43(1), Xhat43(0));

    T43m = calcA32(th3_5).inverse() * calcA21(th2_5).inverse() * calcA10(th1_1).inverse() * T60 * calcA65(th6_1).inverse() * calcA54(th5_1).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_5 = atan2(Xhat43(1), Xhat43(0));

    T43m = calcA32(th3_6).inverse() * calcA21(th2_6).inverse() * calcA10(th1_1).inverse() * T60 * calcA65(th6_2).inverse() * calcA54(th5_2).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_6 = atan2(Xhat43(1), Xhat43(0));

    T43m = calcA32(th3_7).inverse() * calcA21(th2_7).inverse() * calcA10(th1_2).inverse() * T60 * calcA65(th6_3).inverse() * calcA54(th5_3).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_7 = atan2(Xhat43(1), Xhat43(0));

    T43m = calcA32(th3_8).inverse() * calcA21(th2_8).inverse() * calcA10(th1_2).inverse() * T60 * calcA65(th6_4).inverse() * calcA54(th5_4).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_8 = atan2(Xhat43(1), Xhat43(0));

    //Reuslt of the inverse kinematics
    MatrixXf Th(8,6);

    Th << th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
          th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
          th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
          th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
          th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
          th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
          th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
          th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;

    //cout << Th << endl;

    return Th;
}
