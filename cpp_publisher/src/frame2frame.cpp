#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

/* typedef double radiants;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Matrix3d RotationMatrix; */

/* Vector3d fromWorldToBase(Vector3d point);
Matrix4d transformPoint(Vector3d pointA, Vector3d pointB, vector<radiants> theta);
Matrix4d transformPointAux(Vector3d translation, vector<radiants> theta); */

Eigen::Vector3d transformationWorldToBase(Eigen::Vector3d pointInWorldFrame);

int main(){

    //Define the point in the world frame
    Eigen::Vector3d pointInWorldFrame(0.616, 0.739, 0.87);

    Eigen::Vector3d pointInBaseFrame = transformationWorldToBase(pointInWorldFrame);
    std::cout << "Point in base frame: " << pointInBaseFrame.transpose() << std::endl;

    return 0;
}

/**
 * @brief takes a position in the world frame and returns the position in the base frame
 * 
 * @param pointInWorldFrame 
 * @return Eigen::Vector3d 
 */
Eigen::Vector3d transformationWorldToBase(Eigen::Vector3d pointInWorldFrame){

    //Define traslation and rotation
    Eigen::Vector3d t(0.5, 0.35, 1.75);
    Eigen::AngleAxisd r(M_PI, Eigen::Vector3d::UnitX());

    //Create affine transformation from traslation and rotation
    Eigen::Affine3d transformation(r);
    transformation.pretranslate(t);

    //Calculate coordinates of nwe frame
    Eigen::Vector3d pointInBaseFrame = transformation * pointInWorldFrame;

    //Some corrections
    pointInBaseFrame[0] = pointInBaseFrame[0] - 1;

    return pointInBaseFrame;
}

/* Vector3d fromWorldToBase(Vector3d point){

    //taken in input a point in the world frame, it returns the point in the base frame knowing 
    //the position of the base frame in the world frame and the orientation of the base frame in the world frame
    //is 0.5, 0.35, 1.75 , pi, 0, 0

    Vector3d pointA;
    Vector3d pointB;
    pointA << 0, 0, 0;
    pointB << 0.5, 0.35, 1.75;
    vector<radiants> theta = {-M_PI, 0, 0};

    Matrix4d transformPointMat = transformPoint(pointA, pointB, theta);
    cout << transformPointMat << endl;

    Vector3d pointDes;
    // cout << "block<3,3>(0,0)" << endl;
    // cout << transformPointMat.block<3,3>(0,0) << endl;
    // cout << "block<3,1>(0,3)" << endl;
    // cout << transformPointMat.block<3,1>(0,3) << endl;

    cout << transformPointMat.block<3,1>(0,3) << endl;
    cout << point << endl;
    //pointDes = transformPointMat.block<3,1>(0,3) * point; //rottissimo
    //pointDes << transformPointMat(0,3), transformPointMat(1,3), transformPointMat(2,3);
    return pointDes;
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
} */