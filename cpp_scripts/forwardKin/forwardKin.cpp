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

//#define pi 3.14159265358979323846

struct robotParams{
    int A[3] = {3, 3, 3};
    float Th[3] = {M_PI_4, M_PI_2, M_PI/3};
} robotParams;

void fwKin(int A[3], float Th[3]); // This function will calculate the forward kinematics of the robot and return the position of the end effector


int main(int argc, char** argv){

    fwKin(robotParams.A, robotParams.Th);

    return 0;
}

void fwKin(int A[3], float Th[3]){
    // This function will calculate the forward kinematics of the robot and return the position of the end effector
    // The inputs are the link lengths and the joint angles
    // The output is the position of the end effector

    Eigen::MatrixXf A10(4,4);
    Eigen::MatrixXf A21(4,4);
    Eigen::MatrixXf A32(4,4);
    Eigen::MatrixXf A30(4,4);

    A10 << cos(Th[0]), 0, sin(Th[0]), 0,
            sin(Th[0]), 0, -cos(Th[0]), 0,
            0, 1, 0, 0,
            0, 0, 0, 1;

    A21 << cos(Th[1]), -sin(Th[1]), 0, A[1]*cos(Th[1]),
            sin(Th[1]), cos(Th[1]), 0, A[1]*sin(Th[1]),
            0, 0, 1, 0,
            0, 0, 0, 1;

    A32 << cos(Th[2]), -sin(Th[2]), 0, A[2]*cos(Th[2]),
            sin(Th[2]), cos(Th[2]), 0, A[2]*sin(Th[2]),
            0, 0, 1, 0,
            0, 0, 0, 1;

    A30 = A10*A21*A32;

    cout << A30 << endl;
    
    // for(int i=0; i<4; i++){
    //     for(int j=0; j<4; j++){
    //         cout << A30[i][j] << " ";
    //     }
    //     cout << endl;
    // }
}