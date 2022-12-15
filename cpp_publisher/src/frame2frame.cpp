#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

typedef double radiants;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Matrix3d RotationMatrix;

// TODO TEST THE FUNCTION
int main(){
    // call function in here
    // do testing in here
}

Matrix4d transformPoint(Vector3d pointA, Vector3d pointB, std::vector<radiants> theta){
    Vector3d translation = pointB - pointA;
    return transformPoint(translation, theta);
}

Matrix4d transformPoint(Vector3d translation, std::vector<radiants> theta){
    RotationMatrix Rx;
    Rx << 1, 0, 0,
          0, cos(theta[0]), -sin(theta[0]),
          0, sin(theta[0]), cos(theta[0]);
    RotationMatrix Ry;
    Ry << cos(theta[1]), 0, sin(theta[1]),
          0, 1, 0,
          -sin(theta[1]), 0, cos(theta[1]);
    RotationMatrix Rz;
    Rz << cos(theta[2]), -sin(theta[2]), 0,
          sin(theta[2]), cos(theta[2]), 0,
          0, 0, 1;

    RotationMatrix R = Rz * Ry * Rx;

    Matrix4d transformPoint;
    transformPoint << R(0,0), R(0,1), R(0,2), translation(0),
                      R(1,0), R(1,1), R(1,2), translation(1),
                      R(2,0), R(2,1), R(2,2), translation(2),
                      0, 0, 0, 1;
    return transformPoint;
}