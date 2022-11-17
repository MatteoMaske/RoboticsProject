#include <iostream>
#include <Eigen>

using namespace std;
using Eigen::MatrixXf;

const float A[6] = {0, -0.425, -0.3922, 0, 0, 0};
const float D[6] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};

void fwKin(float Th[6], float endEffectorPos[3]); // This function will calculate the forward kinematics of the robot and return the position of the end effector

int main(int argc, char** argv){

    float Th[6] = {1.6, 0.2, -0.5, 2.89, 1.1, 1.25};
    float endEffectorPos[3] = {0, 0, 0}; // This will be the position of the end effector

    fwKin(Th, endEffectorPos);

    cout << "The end effector is at: " << endEffectorPos[0] << ", " << endEffectorPos[1] << ", " << endEffectorPos[2] << endl;

    return 0;
}

void fwKin(float Th[6], float endEffectorPos[3]){

    MatrixXf A10(4,4);
    MatrixXf A21(4,4);
    MatrixXf A32(4,4);
    MatrixXf A43(4,4);
    MatrixXf A54(4,4);
    MatrixXf A65(4,4);
    MatrixXf A60(4,4);

    MatrixXf Pe(1,4);
    MatrixXf Re(3,3);

    A10 <<  cos(Th[0]), -sin(Th[0]), 0, 0,
            sin(Th[0]), cos(Th[0]), 0, 0,
            0, 0, 1, D[0],
            0, 0, 0, 1;

    A21 <<  cos(Th[1]), -sin(Th[1]), 0, 0,
            0, 0, -1, 0,
            sin(Th[1]), cos(Th[1]), 0, 0,
            0, 0, 0, 1;

    A32 <<  cos(Th[2]), -sin(Th[2]), 0, A[1],
            sin(Th[2]), cos(Th[2]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    A43 <<  cos(Th[3]), -sin(Th[3]), 0, A[2],
            sin(Th[3]), cos(Th[3]), 0, 0,
            0, 0, 1, D[3],
            0, 0, 0, 1;

    A54 <<  cos(Th[4]), -sin(Th[4]), 0, 0,
            0, 0, -1, -D[4],
            sin(Th[4]), cos(Th[4]), 0, 0,
            0, 0, 0, 1;

    A65 <<  cos(Th[5]), -sin(Th[ 5]), 0, 0,
            0, 0, 1, D[5],
            -sin(Th[5]), -cos(Th[5]), 0, 0,
            0, 0, 0, 1;

    A60 = A10*A21*A32*A43*A54*A65;

    Pe = A60.block(0,3,3,1);
    Re = A60.block(0,0,3,3);

    //cout << Pe << endl;
    //cout << Re << endl;

    endEffectorPos[0] = Pe(0,0);
    endEffectorPos[1] = Pe(1,0);
    endEffectorPos[2] = Pe(2,0);
}
