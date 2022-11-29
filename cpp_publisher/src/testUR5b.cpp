#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <std_msgs/Float64MultiArray.h>
#include "ros/ros.h"
#include "kinematicsUr5.cpp"

#define LOOPRATE 1000 //rate of ros loop

using namespace std;
using Eigen::MatrixXf;

//=======GLOBAL VARIABLES=======
ros::Publisher pub_des_jstate; //publish desired joint state

//=======FUNCTION DECLARATION=======
MatrixXf xe(float t, MatrixXf xef, MatrixXf xe0); //linear interpolation of the position
MatrixXf phie(float t, MatrixXf phief, MatrixXf phie0); //linear interpolation of the orientation
Eigen::MatrixXf toRotationMatrix(Eigen::MatrixXf euler); //convert euler angles to rotation matrix
void movingProcedure(float dt, float vDes, MatrixXf qDes, MatrixXf qRef, ros::Publisher pub_des_jstate); //moving procedure
void publish(Eigen::MatrixXf publishPos, ros::Publisher pub_des_jstate); //publish the joint angles
void computeMovement(MatrixXf Th0, MatrixXf targetPosition, MatrixXf targetOrientation, ros::Publisher pub_des_jstate);//compute the movement
void closeGripper(MatrixXf currentPos);
void openGripper(MatrixXf currentPos);

//=======MAIN FUNCTION=======
int main(int argc, char **argv){

    //ROS initialization
    ros::init(argc, argv, "testUR5b");
    ros::NodeHandle node;
    pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1); //publisher for desired joint state

    /*initial joint angles*/
    MatrixXf Th0(1,6);
    Th0 << -0.322, -0.7805, -2.5675, -1.634, -1.571, -1.0017; //homing procedure joint angles

    /*target position and orientation*/
    MatrixXf xef(1,3);
    MatrixXf phief(1,3); 
    xef << 0.35, -0.15, 0.70;// 0.5, -0.5, 0.5;
    phief << 0, 0, 0;

    computeMovement(Th0, xef, phief, pub_des_jstate);

    return 0;
}

/**
 * @brief compute the movement without controlling the velocity
 * 
 * @param Th0 
 * @param targetPosition 
 * @param targetOrientation 
 * @param pub_des_jstate 
 */
void computeMovement(MatrixXf Th0, MatrixXf targetPosition, MatrixXf targetOrientation, ros::Publisher pub_des_jstate){

    EEPose eePose;

    /*calc initial end effector pose*/
    eePose = fwKin(Th0);

    /*calc distance between initial and target position*/
    float targetDist = sqrt(pow( targetPosition(0,0) - eePose.Pe(0,0), 2 ) + pow( targetPosition(0,1) - eePose.Pe(1,0), 2 ) + pow( targetPosition(0,2) - eePose.Pe(2,0), 2 ));
    float step = targetDist / 100; //step size

    /*from rotation matrix to euler angles*/
    MatrixXf phie0;
    Eigen::Matrix3f tmp;
    tmp = eePose.Re;
    phie0 = tmp.eulerAngles(0,1,2);

    MatrixXf x(1,3); //position
    MatrixXf phi(1,3); //orientation
    Eigen::Matrix3f rotM; //rotation matrix
    MatrixXf TH(8,6); //joint angles
    MatrixXf currentPos(1,6); //current joint angles
    EEPose eePose1;

    /*current position equals to homing procedure position*/
    currentPos = Th0;

    /*create message to publish*/
    std_msgs::Float64MultiArray msg;
    msg.data.resize(9); //6 joint angles + 3 end effector joints
    msg.data.assign(9,0); //empty the msg

    cout << "Moving to target -> " << targetPosition << endl;

    /*loop to calculate and publish the joint angles*/
    for(float t=0; t<=1; t+=step){

        x = xe(t, targetPosition, eePose.Pe); //linear interpolation of the position
        phi = phie(t, targetOrientation, phie0); //linear intepolation of the orientation
        x.transposeInPlace();
        
        /*from euler angles to rotation matrix*/
        rotM = toRotationMatrix(phi);

        /*save the result in the end effector struct*/
        eePose1.Pe = x;
        eePose1.Re = rotM;
        
        /*inverse kinematics*/
        TH = invKin(eePose1);

        /*check nan and minimum distance*/
        MatrixXf minSol(1,6);
        bool use = true;
        int minDistRow= 0;
        float dist = 0.;
        float minDist= 10000.;

        for(int i=0; i<8; i++){
            
            use = true;
            for(int j=0; j<6; j++){
                if(isnan(TH(i,j))){
                    use = false;
                    break;
                }
            }
            if(use){
                minSol = TH.row(i) - currentPos;

                /*calculate the average distance between the joints*/
                for(int j=0; j<6; j++){
                    dist += abs(minSol(0,j));
                }
                dist = dist/6;
                if(dist < minDist){
                    minDist = dist;
                    minDistRow = i;
                }
            }
        }

        /*save in current position the best solution of inverse kinemtics*/
        currentPos = TH.row(minDistRow);

        /*ROS loop*/
        publish(currentPos, pub_des_jstate);
    }

    /*close the gripper when in position*/
    closeGripper(currentPos);
}

/**
 * @brief From euler angles to rotation matrix
 * 
 * @param euler 
 * @return Eigen::MatrixXf 
 */
Eigen::MatrixXf toRotationMatrix(Eigen::MatrixXf euler){
    Eigen::Matrix3f m;
    m = Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitX());
    return m;
}

/**
 * @brief using movingProcedure of ur5generic.py template to get to a desired position
 * 
 * @param dt 
 * @param vDes 
 * @param qDes 
 * @param qRef
 * @return Eigen::MatrixXf 
 */
void movingProcedure(float dt, float vDes, MatrixXf qDes, MatrixXf qRef, ros::Publisher pub_des_jstate){

    cout << "MOVING PROCEDURE STARTED" << endl;

    ros::Rate loop_rate(LOOPRATE); //rate of the loop

    Eigen::MatrixXf error (1,6);
    float errorNorm = 0.0;
    float vRef = 0.0;

    do{            
        error = qDes - qRef;
        errorNorm = error.norm();
        vRef += 0.005*(vDes-vRef);
        qRef += dt*vRef*error/errorNorm;
        publish(qRef, pub_des_jstate);
        loop_rate.sleep();
        ros::spinOnce(); 
    }while(errorNorm > 0.001);

    cout << "HOMING PROCEDURE COMPLETED" << endl;
}

/**
 * @brief publish the desired joint state
 * 
 * @param publishPos 
 * @param pub_des_jstate 
 */
void publish(MatrixXf publishPos, ros::Publisher pub_des_jstate){

    std_msgs::Float64MultiArray msg;
    msg.data.resize(9); //6 joint angles + 3 end effector joints
    msg.data.assign(9,0); //empty the msg
    ros::Rate loop_rate(LOOPRATE);

    for (int i = 0; i < publishPos.cols(); i++){
        msg.data[i] = publishPos(0, i);
    }

    pub_des_jstate.publish(msg); // publish the message

    ros::spinOnce();   // allow data update from callback
    loop_rate.sleep(); // sleep for the time remaining to let us hit our 1000Hz publish rate
}

/**
 * @brief close the gripper
 * 
 * @param currentPos 
 */
void closeGripper(MatrixXf currentPos){

    std_msgs::Float64MultiArray msg;
    msg.data.resize(9); //6 joint angles + 3 end effector joints
    msg.data.assign(9,0); //empty the msg

    for(int i=0; i<6; i++){
        msg.data[i] = currentPos(0,i); //keep same joint angles
    }
    msg.data[6] = 2; msg.data[7] = 2.; msg.data[8] = 2.; //close the gripper

    pub_des_jstate.publish(msg); //publish the message
}

/**
 * @brief open the gripper
 * 
 * @param currentPos 
 */
void openGripper(MatrixXf currentPos){

    std_msgs::Float64MultiArray msg;
    msg.data.resize(9); //6 joint angles + 3 end effector joints
    msg.data.assign(9,0); //empty the msg

    for(int i=0; i<6; i++){
        msg.data[i] = currentPos(0,i); //keep same joint angles
    }
    msg.data[6] = 0.; msg.data[7] = 0.; msg.data[8] = 0.; //open the gripper

    pub_des_jstate.publish(msg); //publish the message
}

/**
 * @brief linear interpolation of the position
 * 
 * @param t
 * @param xef
 * @param xe0
 * @return MatrixXf(1,3) 
 */
MatrixXf xe(float t, MatrixXf xef, MatrixXf xe0){
    xe0.transposeInPlace();
    MatrixXf x(1,3);
    x = t * xef + (1-t) * xe0;
    return x;
}

/**
 * @brief linear interpolation of the orientation
 * 
 * @param t 
 * @param phief 
 * @param phie0 
 * @return MatrixXf(1,3) 
 */
MatrixXf phie(float t, MatrixXf phief, MatrixXf phie0){
    phie0.transposeInPlace();
    MatrixXf phi(1,3);
    phi = t * phief + (1-t) * phie0;
    return phi;
}
