#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

using namespace std;

typedef double radiants;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Matrix3d RotationMatrix;

Vector3d fromWorldToBase(Vector3d point);
Matrix4d transformPoint(Vector3d pointA, Vector3d pointB, vector<radiants> theta);
Matrix4d transformPointAux(Vector3d translation, vector<radiants> theta);

// TODO TEST THE FUNCTION
int main(){

    Vector3d point;
    point << 0.8,0.45,0.9;
    cout << fromWorldToBase(point) << endl;


    return 0;
    
}

Vector3d fromWorldToBase(Vector3d point){

    //taken in input a point in the world frame, it returns the point in the base frame knowing 
    //the position of the base frame in the world frame and the orientation of the base frame in the world frame
    //is 0.5, 0.35, 1.75 , pi, 0, 0

    Vector3d pointA;
    pointA << 0.5, 0.35, 1.75;
    vector<radiants> theta = {0, 0, M_PI};

    Matrix4d transformPointMat = transformPoint(pointA, point, theta);
    Vector3d pointB;
    pointB << transformPointMat(0,3), transformPointMat(1,3), transformPointMat(2,3);
    return pointB;
}

Matrix4d transformPoint(Vector3d pointA, Vector3d pointB, vector<radiants> theta){
    Vector3d translation = pointB - pointA;
    return transformPointAux(translation, theta);
}

Matrix4d transformPointAux(Vector3d translation, vector<radiants> theta){
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