#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

Eigen::Vector3d transformationWorldToBase(Eigen::Vector3d pointInWorldFrame);

/**
 * @brief takes a position in the world frame and returns the position in the base frame
 * 
 * @param pointInWorldFrame 
 * @return Eigen::Vector3d 
 */
Eigen::Vector3d transformationWorldToBase(Eigen::Vector3d pointInWorldFrame){

    //Define traslation and rotation
    Eigen::Vector3d t(-0.5, 0.35, 1.75);
    Eigen::AngleAxisd r(M_PI, Eigen::Vector3d::UnitX());

    //Create affine transformation from traslation and rotation
    Eigen::Affine3d transformation(r);
    transformation.pretranslate(t);

    //Calculate coordinates of new frame
    Eigen::Vector3d pointInBaseFrame = transformation * pointInWorldFrame;

    return pointInBaseFrame;
}
