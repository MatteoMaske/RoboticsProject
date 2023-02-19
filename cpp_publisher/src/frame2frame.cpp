/**
 * @file frame2frame.cpp
 * @author Amir Gheser
 * @brief File that contains the function to transform a point from the world frame to the base frame
 * @version 1.0
 * @date 2023-02-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

Eigen::Vector3f transformationWorldToBase(Eigen::Vector3f pointInWorldFrame);

/**
 * @brief takes a position in the world frame and returns the position in the base frame
 * 
 * @param pointInWorldFrame 
 * @return Eigen::Vector3f 
 */
Eigen::Vector3f transformationWorldToBase(Eigen::Vector3f pointInWorldFrame){

    //Define traslation and rotation
    Eigen::Vector3f t(-0.5, 0.35, 1.75);
    Eigen::AngleAxisf r(M_PI, Eigen::Vector3f::UnitX());

    //Create affine transformation from traslation and rotation
    Eigen::Affine3f transformation(r);
    transformation.pretranslate(t);

    //Calculate coordinates of new frame
    Eigen::Vector3f pointInBaseFrame = transformation * pointInWorldFrame;

    return pointInBaseFrame;
}
