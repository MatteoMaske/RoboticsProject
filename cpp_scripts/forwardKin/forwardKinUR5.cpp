#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <Eigen>

using namespace std;
using Eigen::MatrixXf;
using Eigen::VectorXf;

#define scaleFactor 10

struct robotParams{
    float A[6] = {0, -0.425, -0.3922, 0, 0, 0};
    float D[6] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};
    float Th[6] = {0,0,0,0,0,0};
} robotParams;

void fwKin(struct robotParams, float endEffectorPos[3]); // This function will calculate the forward kinematics of the robot and return the position of the end effector

int main(int argc, char** argv){

    float endEffectorPos[3] = {0, 0, 0}; // This will be the position of the end effector

    for(int i=0; i<6; i++){
        robotParams.A[i] *= scaleFactor;
        robotParams.D[i] *= scaleFactor;
    }

    fwKin(robotParams, endEffectorPos);

    cout << "The end effector is at: " << endEffectorPos[0] << ", " << endEffectorPos[1] << ", " << endEffectorPos[2] << endl;

    return 0;
}

void fwKin(struct robotParams, float endEffectorPos[3]){
    
    float A[6] = {robotParams.A[0], robotParams.A[1], robotParams.A[2], robotParams.A[3], robotParams.A[4], robotParams.A[5]};
    float D[6] = {robotParams.D[0], robotParams.D[1], robotParams.D[2], robotParams.D[3], robotParams.D[4], robotParams.D[5]};
    float Th[6] = {robotParams.Th[0], robotParams.Th[1], robotParams.Th[2], robotParams.Th[3], robotParams.Th[4], robotParams.Th[5]};

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

    // cout << Pe << endl;
    // cout << Re << endl;

    endEffectorPos[0] = Pe(0,0);
    endEffectorPos[1] = Pe(1,0);
    endEffectorPos[2] = Pe(2,0);
}
