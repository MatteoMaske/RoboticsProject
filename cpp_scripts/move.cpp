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

using namespace ros;

struct joint{
    double link_length;
    double joint_angle;
};

struct anthropomorphic_arm{
    std::vector<double> joint_angles;
};

int main(){



    return 0;
}

void inverseKinematics(){

}

void forwardKinematics(){

}

void moveUr5(){
    // Moves the ur5 from an initial pose to a target frame represnting an object
    // The target frame is defined by a 4x4 matrix
    // The initial pose is defined by a 4x4 matrix
    
    // compute the jacobian
    // velocity = jacobian * joint_angles_derived
    

}