#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include "ros/ros.h"
#include "kinematicsUr5.cpp"

#define LOOPRATE 1000 //rate of ros loop
#define ROBOT_JOINTS 6 //number of joints of the robot
#define EE_JOINTS 2 //number of joints of the end effector

using namespace std;
using Eigen::MatrixXf;

//=======GLOBAL VARIABLES=======
ros::Publisher pub_des_jstate; //publish desired joint state
MatrixXf currentPos(1,6); //current joint angles
MatrixXf currentGripper(1,2); //current gripper pos

//=======FUNCTION DECLARATION=======
MatrixXf xe(float t, MatrixXf xef, MatrixXf xe0); //linear interpolation of the position
MatrixXf phie(float t, MatrixXf phief, MatrixXf phie0); //linear interpolation of the orientation
MatrixXf toRotationMatrix(MatrixXf euler); //convert euler angles to rotation matrix
void homingProcedure(float dt, float vDes, MatrixXf qDes, MatrixXf qRef); //moving procedure
void publish(MatrixXf publishPos); //publish the joint angles
MatrixXf computeMovementInverse(MatrixXf Th0, MatrixXf targetPosition, MatrixXf targetOrientation);//compute the movement
MatrixXf computeMovementDifferential(MatrixXf Th0, MatrixXf targetPosition, MatrixXf targetOrientation,float dt);//compute the movement
MatrixXf invDiffKinematiControlComplete(MatrixXf q, MatrixXf xe, MatrixXf xd, MatrixXf vd, MatrixXf phie, MatrixXf phid, MatrixXf phiddot, MatrixXf kp, MatrixXf kphi);
MatrixXf jacobian(MatrixXf Th);
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

void changeGripper(float firstVal, float secondVal);
//=======MAIN FUNCTION=======
int main(int argc, char **argv){

    //ROS initialization
    ros::init(argc, argv, "move");
    ros::NodeHandle node,node1;
    pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1); //publisher for desired joint state

    ros::Subscriber sub = node1.subscribe("/ur5/joint_states", 1, jointStateCallback); //subscriber for joint state
    /*initial joint angles*/
    MatrixXf Th0(1,6);
    Th0 << -0.322, -0.7805, -2.5675, -1.634, -1.571, -1.0017; //homing procedure joint angles

    /*initial gripper pos*/
    currentPos = Th0;
    currentGripper << 0.0, 0.0;

    /*target position and orientation*/
    MatrixXf xef(1,3);
    MatrixXf phief(1,3); 
    /* xef << 0.35, -0.15, 0.70;// 0.5, -0.5, 0.5;
    phief << 0, 0, 0; */

    //currentPos = computeMovement(Th0, xef, phief, pub_des_jstate); //compute the movement to the first brick in tavolo_brick.world
    int input;
    while(1){
        do{
            cout << "[1] for moving to a point with differential kinematics" << endl;
            cout << "[2] for coming back in homing procedure" << endl;
            cout << "[3] for getting current ee pos" << endl;
            cout << "[4] for getting current joint state" << endl;
            cout << "[5] for moving to a point with inverse kinematics" << endl;
            cout << "[0] to exit" << endl;
            cin >> input;

        }while(input != 0 && input != 1 && input != 2 && input != 3 && input != 4 && input != 5);

        if(input == 1){

            cout << "Insert the posistion coordinate: " << endl;
            cin >> xef(0,0) >> xef(0,1) >> xef(0,2);
            // cout << "Insert the orientation coordinate: " << endl;
            // cin >> phief(0,0) >> phief(0,1) >> phief(0,2);
            phief << 0, 0, 0;
            currentPos = computeMovementDifferential(currentPos, xef, phief,0.01); //compute the movement to the first brick in tavolo_brick.world
        }
        else if(input == 2){
            homingProcedure(0.001,0.6, Th0, currentPos);
        }else if(input==3){
            EEPose eePose;
            eePose = fwKin(currentPos);
            cout << "Current ee position: " << endl;
            cout << eePose.Pe.transpose() << endl;
        }else if(input == 4){
            ros::spinOnce();
        }else if(input == 5){
            cout << "Insert the posistion coordinate: " << endl;
            cin >> xef(0,0) >> xef(0,1) >> xef(0,2);
            phief << 0, 0, 0;
            computeMovementInverse(currentPos, xef, phief);
        }else{
            break;
        }
    }

    return 0;
}

/**
 * @brief compute the movement without controlling the velocity
 * 
 * @param Th0 
 * @param targetPosition 
 * @param targetOrientation 
 * @param pub_des_jstate
 * @return MatrixXf
 */
MatrixXf computeMovementInverse(MatrixXf Th0, MatrixXf targetPosition, MatrixXf targetOrientation){

    EEPose eePose;

    /*calc initial end effector pose*/
    eePose = fwKin(Th0);

    /*calc distance between initial and target position*/
    float targetDist = sqrt(pow( targetPosition(0,0) - eePose.Pe(0,0), 2 ) + pow( targetPosition(0,1) - eePose.Pe(1,0), 2 ) + pow( targetPosition(0,2) - eePose.Pe(2,0), 2 ));
    float step = targetDist / 1000; //step size

    /*from rotation matrix to euler angles*/
    MatrixXf phie0;
    Eigen::Matrix3f tmp;
    tmp = eePose.Re;
    phie0 = tmp.eulerAngles(0,1,2);

    MatrixXf x(1,3); //position
    MatrixXf phi(1,3); //orientation
    Eigen::Matrix3f rotM; //rotation matrix
    MatrixXf TH(8,6); //joint angles
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
        publish(currentPos);
    }

    /*close the gripper when in position*/
    //changeGripper(0,0);

    return currentPos;
}
MatrixXf computeMovementDifferential(MatrixXf Th0, MatrixXf targetPosition, MatrixXf targetOrientation,float dt){
    
    cout << "Moving to target with differential kinematics-> " << targetPosition << endl;

    /*calc initial end effector pose*/
    EEPose eePose;
    eePose = fwKin(Th0);

    /*calc x0 and phie0*/
    MatrixXf phie0,x0;
    x0 = eePose.Pe;
    phie0 = eePose.Re.eulerAngles(0,1,2);

    /*current position equals to homing procedure position*/
    currentPos = Th0;

    /*Matrix to coorect the trajectory which otehrwise is bad approximated*/
    MatrixXf kp(3,3);
    kp = Eigen::Matrix3f::Identity(3,3)*100;
    MatrixXf kphi(3,3);
    kphi = Eigen::Matrix3f::Identity(3,3)*100;

    /*parameters for the loop*/
    MatrixXf x(1,3); //position
    MatrixXf phi(1,3); //orientation
    EEPose eePose1;

    MatrixXf qk(1,6);
    MatrixXf qk1(1,6);
    MatrixXf vd(1,3);
    MatrixXf phiddot(1,3);
    MatrixXf dotqk(6,6);
    MatrixXf xArg(1,3);
    MatrixXf phiArg(1,3);

    qk = currentPos; //initialize qk

    for(float t=dt; t<=1; t+=dt){

        eePose1 = fwKin(qk);
        x = eePose1.Pe;
        phi = eePose1.Re.eulerAngles(0,1,2);
        vd = (xe(t,targetPosition,x0)-xe(t-dt,targetPosition,x0)) / dt;
        phiddot = (phie(t,targetOrientation,phie0)-phie(t-dt,targetOrientation,phie0)) / dt;
        xArg = xe(t,targetPosition,x0);
        phiArg = phie(t,targetOrientation,phie0);

        dotqk = invDiffKinematiControlComplete(qk,x,xArg.transpose(),vd.transpose(),phi,phiArg.transpose(),phiddot.transpose(),kp,kphi);
        //cout << "dotqk -> " << dotqk << endl;
        qk1 = qk + dotqk.transpose()*dt;
        qk = qk1;
        publish(qk1);        
    }
    MatrixXf positionReached = fwKin(qk1).Pe.transpose();
    cout << "Target reached -> " << positionReached << endl;
    return qk1;
}

MatrixXf invDiffKinematiControlComplete(MatrixXf q, MatrixXf xe, MatrixXf xd, MatrixXf vd, MatrixXf phie, MatrixXf phid, MatrixXf phiddot, MatrixXf kp, MatrixXf kphi){
    
    MatrixXf J(6,6);
    J = jacobian(q);
    float alpha = phie(2);
    float beta = phie(1);
    float gamma = phie(0);

    MatrixXf T(3,3);
    T << cos(beta)*cos(gamma), -sin(gamma), 0,
        cos(beta)*sin(gamma), cos(gamma), 0,
        -sin(beta), 0, 1;
    
    MatrixXf Ta(6,6);
    Ta << MatrixXf::Identity(3,3), MatrixXf::Zero(3,3),
        MatrixXf::Zero(3,3), T;
    
    /*dumped least squares inverse of J*/
    MatrixXf Ja = Ta.inverse()*J;
    MatrixXf dotQ(6,1);
    MatrixXf ve(6,1);
    MatrixXf Js(6,6);

    float k = pow(10,-6); //dumping factor

    ve << (vd+kp*(xd-xe)),
    (phiddot+kphi*(phid-phie));
    Js = Ja.transpose()*(Ja*Ja.transpose() + pow(k,2)*MatrixXf::Identity(6,6)).inverse();
    dotQ = Js*ve;

    /*limit the velocity of the joints*/
    for(int i = 0; i < 6; i++){
        if(dotQ(i,0) > 2.5){
            dotQ(i,0) = 2.5;
        }
        if(dotQ(i,0) < -2.5){
            dotQ(i,0) = -2.5;
        }
    }

    return dotQ;

}
MatrixXf jacobian(MatrixXf Th){
    MatrixXf A(1,6);
    MatrixXf D(1,6) ;
    A << 0,-0.425,-0.3922,0,0,0;
    D << 0.1625,0,0,0.1333,0.0997,0.0996;

    MatrixXf J1(6,1);    
    J1 << D(4)*(cos(Th(0))*cos(Th(4)) + cos(Th(1)+Th(2)+Th(3))*sin(Th(0))*sin(Th(4))) + D(2)*cos(Th(0)) + D(3)*cos(Th(0)) - A(2)*cos(Th(1))*sin(Th(0)) - A(1)*cos(Th(1))*sin(Th(0)) - D(4)*sin(Th(1)+Th(2)+Th(3))*sin(Th(0)),
        D(4)*(cos(Th(4))*sin(Th(0)) - cos(Th(1)+Th(2)+Th(3))*cos(Th(0))*sin(Th(4))) + D(2)*sin(Th(0)) + D(3)*sin(Th(0)) + A(2)*cos(Th(0))*cos(Th(1)) + A(1)*cos(Th(0))*cos(Th(1)) + D(4)*sin(Th(1)+Th(2)+Th(3))*cos(Th(0)),
        0,
        0,
        0,
        1;
    

    MatrixXf J2(6,1);
    J2 << -cos(Th(0))*(A(2)*sin(Th(1)+Th(2)) + A(1)*sin(Th(1)) + D(4)*(sin(Th(2)+Th(3))*sin(Th(3)) - cos(Th(1)+Th(2))*cos(Th(3))) - D(4)*sin(Th(4))*(cos(Th(1)+Th(2))*sin(Th(3)) + sin(Th(1)+Th(2))*cos(Th(3)))),
        -sin(Th(0))*(A(2)*sin(Th(1)+Th(2)) + A(1)*sin(Th(1)) + D(4)*(sin(Th(1)+Th(2))*sin(Th(3)) - cos(Th(1)+Th(2))*cos(Th(3))) - D(4)*sin(Th(4))*(cos(Th(1)+Th(2))*sin(Th(3)) + sin(Th(1)+Th(2))*cos(Th(3)))),
        A(2)*cos(Th(1)+Th(2)) - (D(4)*sin(Th(1)+Th(2)+Th(3)+Th(4)))/2 + A(1)*cos(Th(1)) + (D(4)*sin(Th(1)+Th(2)+Th(3)-Th(4)))/2 + D(4)*sin(Th(1)+Th(2)+Th(3)),
        sin(Th(0)),
        -cos(Th(0)),
        0;


    MatrixXf J3(6,1);
    J3 << cos(Th(0))*(D(4)*cos(Th(1)+Th(2)+Th(3)) - A(2)*sin(Th(1)+Th(2)) + D(4)*sin(Th(1)+Th(2)+Th(3))*sin(Th(4))),
        sin(Th(0))*(D(4)*cos(Th(1)+Th(2)+Th(3)) - A(2)*sin(Th(1)+Th(2)) + D(4)*sin(Th(1)+Th(2)+Th(3))*sin(Th(4))),
        A(2)*cos(Th(1)+Th(2)) - (D(4)*sin(Th(1)+Th(2)+Th(3)+Th(4)))/2 + (D(4)*sin(Th(1)+Th(2)+Th(3)-Th(4)))/2 + D(4)*sin(Th(1)+Th(2)+Th(3)),
        sin(Th(0)),
        -cos(Th(0)),
        0;
    
    MatrixXf J4(6,1);
    J4 << D(4)*cos(Th(0))*(cos(Th(1)+Th(2)+Th(3)) + sin(Th(1)+Th(2)+Th(3))*sin(Th(4))),
        D(4)*sin(Th(0))*(cos(Th(1)+Th(2)+Th(3)) + sin(Th(1)+Th(2)+Th(3))*sin(Th(4))),
        D(4)*(sin(Th(1)+Th(2)+Th(3)-Th(4))/2 + sin(Th(1)+Th(2)+Th(3)) - sin(Th(1)+Th(2)+Th(3)+Th(4))/2),
        sin(Th(0)),
        -cos(Th(0)),
        0;

    MatrixXf J5(6,1);
    J5 << -D(4)*sin(Th(0))*sin(Th(4)) - D(4)*cos(Th(1)+Th(2)+Th(3))*cos(Th(0))*cos(Th(4)),
        D(4)*cos(Th(0))*sin(Th(4)) - D(4)*cos(Th(1)+Th(2)+Th(3))*cos(Th(4))*sin(Th(0)),
        -D(4)*(sin(Th(1)+Th(2)+Th(3)-Th(4))/2 + sin(Th(1)+Th(2)+Th(3)+Th(4))/2),
        sin(Th(1)+Th(2)+Th(3))*cos(Th(0)),
        sin(Th(1)+Th(2)+Th(3))*sin(Th(0)),
        -cos(Th(1)+Th(2)+Th(3));

    MatrixXf J6(6,1);
    J6 << 0,
        0,
        0,
        cos(Th(4))*sin(Th(0)) - cos(Th(1)+Th(2)+Th(3))*cos(Th(0))*sin(Th(4)),
        -cos(Th(0))*cos(Th(4)) - cos(Th(1)+Th(2)+Th(3))*sin(Th(0))*sin(Th(4)),
        -sin(Th(1)+Th(2)+Th(3))*sin(Th(4));

    MatrixXf J(6,6);
    J << J1, J2, J3, J4, J5, J6;
    
    return J;
}

/**
 * @brief From euler angles to rotation matrix
 * 
 * @param euler 
 * @return Eigen::MatrixXf 
 */
MatrixXf toRotationMatrix(MatrixXf euler){
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
void homingProcedure(float dt, float vDes, MatrixXf qDes, MatrixXf qRef){

    cout << "MOVING PROCEDURE STARTED" << endl;

    ros::Rate loop_rate(LOOPRATE); //rate of the loop

    MatrixXf error (1,6);
    float errorNorm = 0.0;
    float vRef = 0.0;

    do{            
        error = qDes - qRef;
        errorNorm = error.norm();
        vRef += 0.005*(vDes-vRef);
        qRef += dt*vRef*error/errorNorm;
        publish(qRef);
        loop_rate.sleep();
        ros::spinOnce(); 
    }while(errorNorm > 0.001);

    currentPos = qRef;

    cout << "HOMING PROCEDURE COMPLETED" << endl;
}

/**
 * @brief publish the desired joint state
 * 
 * @param publishPos 
 * @param pub_des_jstate 
 */
void publish(MatrixXf publishPos){

    std_msgs::Float64MultiArray msg;
    msg.data.resize(ROBOT_JOINTS); //6 joint angles
    msg.data.assign(ROBOT_JOINTS,0); //empty the msg
    ros::Rate loop_rate(LOOPRATE);

    for (int i = 0; i < ROBOT_JOINTS; i++){
        msg.data[i] = publishPos(0, i);
    }

    pub_des_jstate.publish(msg); // publish the message

    loop_rate.sleep(); // sleep for the time remaining to let us hit our 1000Hz publish rate
}

/**
 * @brief close the gripper
 * 
 * @param currentPos 
 */
void changeGripper(float firstVal,float secondVal){

    ros::Rate loop_rate(LOOPRATE);

    std_msgs::Float64MultiArray msg;
    msg.data.resize(EE_JOINTS); //6 joint angles + 3 end effector joints
    msg.data.assign(EE_JOINTS,0); //empty the msg

    msg.data[0] = firstVal;
    msg.data[1] = secondVal;

    //pub_des_jstate.publish(msg); //to do -> change publisher with new topic

    loop_rate.sleep(); // sleep for the time remaining to let us hit our 1000Hz publish rate
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

/**
 * @brief callback for the joint state subscriber to print the current joint state
 * 
 * @param msg 
 */

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){

    MatrixXf lastPos(1,6);
    cout << "Last joint position -> " << endl;
    for (int i = 0; i < ROBOT_JOINTS; i++){
        cout << msg->position[i] << " ";
        lastPos(i) = msg->position[i];
    }
    cout << endl;

    EEPose eepose = fwKin(lastPos);
    cout << "Last ee position -> " << endl;
    cout << eepose.Pe.transpose() << endl;

}
