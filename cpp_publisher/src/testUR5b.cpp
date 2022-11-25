#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <std_msgs/Float64MultiArray.h>
#include "ros/ros.h"
#include "kinematicsUr5.cpp"

using namespace std;
using Eigen::MatrixXf;

MatrixXf xe(float t, MatrixXf xef, MatrixXf xe0); //linear interpolation of the position
MatrixXf phie(float t, MatrixXf phief, MatrixXf phie0); //linear interpolation of the orientation
void movingProcedure(float dt, float vDes, MatrixXf qDes, MatrixXf qRef, ros::Rate publish_rate, ros::Publisher pub_des_jstate); //moving procedure
void publish(Eigen::MatrixXf publishPos, ros::Publisher pub_des_jstate, ros::Rate loop_rate); //publish the joint angles
Eigen::MatrixXf toRotationMatrix(Eigen::MatrixXf euler); //convert euler angles to rotation matrix
void computeMovement(MatrixXf Th0, MatrixXf targetPosition, MatrixXf targetOrientation, ros::Publisher pub_des_jstate, ros::Rate loop_rate);//compute the movement

int main(int argc, char **argv){

    //ROS initialization
    ros::init(argc, argv, "testUR5b");
    ros::NodeHandle node;
    ros::Publisher pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    ros::Rate loop_rate(1000);

    /*initial joint angles*/
    MatrixXf Th0(1,6);
    Th0 << -0.322, -0.7805, -2.5675, -1.634, -1.571, -1.0017; //homing procedure joint angles

    /*target position and orientation*/
    MatrixXf xef(1,3);
    MatrixXf phief(1,3); 
    xef << 0.35, -0.15, 0.70;// 0.5, -0.5, 0.5;
    phief << 0, 0, 0;

    computeMovement(Th0, xef, phief, pub_des_jstate, loop_rate);

    return 0;
}

/**
 * @brief compute the movement without controlling the velocity
 * 
 * @param Th0 
 * @param targetPosition 
 * @param targetOrientation 
 * @param pub_des_jstate 
 * @param loop_rate 
 */

void computeMovement(MatrixXf Th0, MatrixXf targetPosition, MatrixXf targetOrientation, ros::Publisher pub_des_jstate, ros::Rate loop_rate){

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
        publish(currentPos, pub_des_jstate, loop_rate);
    }

    MatrixXf closeGripper(1,9);
    closeGripper << currentPos(0,0), currentPos(0,1), currentPos(0,2), currentPos(0,3), currentPos(0,4), currentPos(0,5), 2.0, 2.0, 2.0;
    publish(closeGripper, pub_des_jstate, loop_rate);

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
 * @param rate 
 * @return Eigen::MatrixXf 
 */
void movingProcedure(float dt, float vDes, MatrixXf qDes, MatrixXf qRef, ros::Rate publish_rate, ros::Publisher pub_des_jstate){

    cout << "MOVING PROCEDURE STARTED" << endl;

    Eigen::MatrixXf error (1,6);
    float errorNorm = 0.0;
    float vRef = 0.0;

    do{            
        error = qDes - qRef;
        errorNorm = error.norm();
        vRef += 0.005*(vDes-vRef);
        qRef += dt*vRef*error/errorNorm;
        publish(qRef, pub_des_jstate, publish_rate);
        publish_rate.sleep();
        ros::spinOnce(); 
    }while(errorNorm > 0.001);

    //cout << "HOMING PROCEDURE COMPLETED" << endl;
}

/**
 * @brief publish the desired joint state
 * 
 * @param publishPos 
 * @param pub_des_jstate 
 */
void publish(MatrixXf publishPos, ros::Publisher pub_des_jstate, ros::Rate loop_rate){

    std_msgs::Float64MultiArray msg;
    msg.data.resize(9); //6 joint angles + 3 end effector joints
    msg.data.assign(9,0); //empty the msg

    while(ros::ok){

        for(int i=0; i<publishPos.cols(); i++){
            msg.data[i] = publishPos(0,i);
        }

        pub_des_jstate.publish(msg); //publish the message

        ros::spinOnce();   // allow data update from callback
        loop_rate.sleep(); // sleep for the time remaining to let us hit our 1000Hz publish rate

        break;
    }
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
