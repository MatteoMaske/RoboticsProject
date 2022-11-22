#include <iostream>
#include <Eigen>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

typedef float orientation_t; // position is represented in radiants
typedef Eigen::Matrix<double, 3, 1> position_t; // position is represented in meters ?
typedef Eigen::Matrix<double, 3, 1> velocity_t; // velocity is represented in meters/second
position_t endEffectorDesired;
position_t endEffectorCurrent;

// TODO(): check if this is correct
typedef struct robotParams{
    float armLength[6]= {0, -0.425, -0.3922, 0, 0, 0};
    float wristPos[3] = {0.1978, -0.6579, 0.2324};
} robotParams;


// Goal is to reduce to 0 the following expression
// e(t) = endEffectorDesired - endEffectorCurrent
// After a derivation step we obtain the following expression
// e'(t) = d'(t) - J(q(t)) * dq(t)
// x_e -> desired Pos 
// x_d -> current Pos
// q -> joint angles
// dq -> joint velocities


int main(int argc, char** argv){
    Eigen::Matrix <float, 6, 6> J;
    return 0;
}

/**
 * @brief Computes the Jacobian matrix
 * 
 * @return Eigen::Matrix3f
 */
Eigen::Matrix3f computeJacobian(){
    
    // TODO
    Eigen::Matrix3f J;
    J <<    0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0;
    return J;
}

/**
 * @brief This function computes the distance between the desired and current end effector position
 * 
 */
void computeError(){
    // TODO
}


/**
 * @brief This function computes the controlled value for the inverse kinematics
 * 
 * @param A robot parameters
 * @param computeJacobian function that takes join positions and robot parameters and returns the Jacobian matrix
 * @param jointPositions vector of joint positions described in radiants
 * @param endEffectorCurrent current position of the end effector
 * @param endEffectorDesired desired position of the end effector
 * @param desiredVelocityEndEffector desired velocity of the end effector
 * @param K Positive definite matrix used to reduce the error
 * 
 * @return Eigen::Matrix3f
 */
Eigen::Matrix3f computeControlledVal(robotParams* A, Eigen::Matrix3f (*computeJacobian)(orientation_t*, robotParams*),
                          orientation_t* jointOrientation, position_t endEffectorCurrent,
                          position_t endEffectorDesired, velocity_t desiredVelocityEndEffector,
                            Eigen::Matrix3f K){

    //double** J = // call computeJacobian();
    Eigen::Matrix3f J = computeJacobian(jointOrientation, A);
    Eigen::Matrix3f dot_Q = J.inverse() * (desiredVelocityEndEffector - K * (endEffectorDesired - endEffectorCurrent));

    return dot_Q;
}

/**
 * @brief Computes the so called condition, indicating whether the robot is approaching a singularity or not
 * 
 * @return float 
 */
float condition(){
    // TODO
}

/**
 * @brief This function contains the main loop moving the robot
 * 
 */
void moveRobot(robotParams* A, Eigen::Matrix3f (*computeJacobian)(position_t*, robotParams*), orientation_t* jointStartingOrientation,
                position_t endEffectorDesired, float minT, float maxT, float dt, Eigen::Matrix3f K){

    orientation_t* jointOrientation = jointStartingOrientation;
    for(float i = minT; i < maxT; i+=dt){
        // TODO 
        // == RESUME FROM HERE ==
    }
}
