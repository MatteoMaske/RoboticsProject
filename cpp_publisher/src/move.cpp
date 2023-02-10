#include <iostream>
#include <Eigen/Dense>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Byte.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <cpp_publisher/Coordinates.h>
#include <cpp_publisher/MoveOperation.h>

#include "kinematicsUr5.cpp"
#include "frame2frame.cpp"

#define DEBUG 0 //debug mode

#define LOOPRATE 1000 //rate of ros loop
#define ROBOT_JOINTS 6 //number of joints of the robot
#define EE_SOFT_JOINTS 2 //number of joints of the end effector
#define EE_HARD_JOINTS 3 //number of joints of the end effector

#define HARD_GRIPPER 1

using namespace std;
using Eigen::MatrixXf;
using Eigen::Vector3f;

//=======GLOBAL VARIABLES=======
ros::Publisher pub_des_jstate; //publish desired joint state
ros::Publisher pub_move_operation; //publish move operation
MatrixXf currentJoint(1,6); //current joint angles
MatrixXf currentGripper; //current gripper joint angles

//=======FUNCTION DECLARATION=======
Vector3f xe(float t, Vector3f xef, Vector3f xe0); //linear interpolation of the position
Vector3f phie(float t, Vector3f phief, Vector3f phie0); //linear interpolation of the orientation
MatrixXf toRotationMatrix(Vector3f euler); //convert euler angles to rotation matrix
MatrixXf computeMovementInverse(MatrixXf Th0, Vector3f targetPosition, Vector3f targetOrientation);//compute the movement
void computeMovementDifferential(Vector3f targetPosition, Vector3f targetOrientation,float dt);//compute the movement
MatrixXf invDiffKinematiControlComplete(MatrixXf q, MatrixXf xe, MatrixXf xd, MatrixXf vd, MatrixXf re, MatrixXf phif, MatrixXf kp, MatrixXf kphi);
MatrixXf computeOrientationError(MatrixXf wRe, MatrixXf wRd);
MatrixXf jacobian(MatrixXf Th);

void coordinateCallback(const cpp_publisher::Coordinates::ConstPtr& msg);
void publishJoint(MatrixXf publishPos); //publish the joint angles
void publishMoveOperation(int blockId, bool success); //publish the joint angles
void changeSoftGripper(float firstVal, float secondVal);
void changeHardGripper(Vector3f ee_joints);
Vector3f mapToGripperJoints(float diameter);

void moveObject(Vector3f pos, Vector3f ori, Vector3f targetPos);
void moveDown(float distance);
void moveUp(float distance);

//=======MAIN FUNCTION=======
int main(int argc, char **argv){

    //ROS initialization
    ros::init(argc, argv, "move");
    ros::NodeHandle node;
    pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1); //publisher for desired joint state

    pub_move_operation = node.advertise<cpp_publisher::MoveOperation>("/move/movement_results", 1); //publisher for desired joint state

    ros::Subscriber coordinateSubscriber = node.subscribe("/planner/position", 1, coordinateCallback); //subscriber for block position
    

    MatrixXf homingJoint(1,6);
    homingJoint << -0.322, -0.7805, -2.5675, -1.634, -1.571, -1.0017; //homing procedure joint angles

    MatrixXf customHomingJoint(1,6);
    customHomingJoint <<   -2.7907,-0.78, -2.56,-1.63, -1.57, 3.49;

    /*initial gripper pos*/
    currentJoint = customHomingJoint;
    if(HARD_GRIPPER) {
        currentGripper.resize(1,3);
        currentGripper << 0.0, 0.0, 0.0;
    }else currentGripper.resize(1,2);

    if(!DEBUG){
        while(ros::ok()){
            ros::spinOnce();
        }
    }

    if(DEBUG){
        int input;
        while(1){

            do{
                cout << "[1] for moving to a point with differential kinematics" << endl;
                cout << "[2] for coming back in homing procedure" << endl;
                cout << "[3] for getting current ee pos" << endl;
                cout << "[4] for getting current joint state" << endl;
                cout << "[5] for moving to a point with inverse kinematics" << endl;
                cout << "[6] for moving the gripper" << endl;
                cout << "[7] for moving up" << endl;
                cout << "[8] for moving down" << endl;
                cout << "[0] to exit" << endl;
                cin >> input;

            }while(input != 0 && input != 1 && input != 2 && input != 3 && input != 4 && input != 5 && input != 6 && input != 7 && input != 8);

            if(input == 1){

                Vector3f pos, ori;
                cout << "Insert the posistion coordinate: " << endl;
                cin >> pos(0) >> pos(1) >> pos(2);
                cout <<"Insert the orientation coordinate: " << endl;
                cin >> ori(0) >> ori(1) >> ori(2);

                cout <<"Choose the reference frame [0] world [1] end effector: " << endl;
                int refFrame;
                cin >> refFrame;

                if(refFrame == 0) {
                    pos(2) += 0.01;
                    pos = transformationWorldToBase(pos);
                }

                computeMovementDifferential(pos, ori ,0.001); //compute the movement to the first brick in tavolo_brick.world
            }
            else if(input == 2){
                publishJoint(customHomingJoint);
                currentJoint = customHomingJoint;
            }else if(input==3){
                EEPose eePose;
                eePose = fwKin(currentJoint);
                cout << "Current ee position: " << endl;
                cout << eePose.Pe.transpose() << endl;
            }else if(input == 4){
                cout << "Current joint state: " << endl;
                cout << currentJoint << endl;
            }else if(input == 5){

                Vector3f pos, ori;
                cout << "Insert the posistion coordinate: " << endl;
                cin >> pos(0) >> pos(1) >> pos(2);
                cout <<"Insert the orientation coordinate: " << endl;
                cin >> ori(0) >> ori(1) >> ori(2);

                cout <<"Choose the reference frame [0] world [1] end effector: " << endl;
                int refFrame;
                cin >> refFrame;
                if(refFrame == 0) pos = transformationWorldToBase(pos);
                //computeMovementInverse(currentJoint, pos, ori);

            }else if(input == 6){

                if(HARD_GRIPPER){
                    float diameter;
                    Vector3f ee_joints = Vector3f::Ones(3);
                    cout << "Insert the value of the gripper joints:" << endl;
                    cin >> diameter;
                    ee_joints*=diameter;
                    changeHardGripper(ee_joints);
                }else{
                    cout << "Insert the value of the gripper joints:" << endl;
                    float value,value2;
                    cin >> value >> value2;
                }
            }else if(input == 7){

                moveUp(0.1);

            }else if(input == 8){
                
                moveDown(0.1);

            }else{
                break;
            }
        }
    }

    return 0;
}

/**
 * @brief compute the movement with inverse kinematics without controlling the speed
 * 
 * @param Th0 
 * @param targetPosition 
 * @param targetOrientation 
 * @param pub_des_jstate
 * @return MatrixXf
 */
MatrixXf computeMovementInverse(MatrixXf Th0, Vector3f targetPosition, Vector3f targetOrientation){

    EEPose eePose;

    /*calc initial end effector pose*/
    eePose = fwKin(Th0);

    /*calc distance between initial and target position*/
    float targetDist = sqrt(pow( targetPosition(0) - eePose.Pe(0), 2 ) + pow( targetPosition(1) - eePose.Pe(1), 2 ) + pow( targetPosition(2) - eePose.Pe(2), 2 ));
    float step = targetDist / 100; //step size

    /*from rotation matrix to euler angles*/
    Vector3f phie0;
    phie0 = eePose.Re.eulerAngles(2,1,0);

    Vector3f x; //position
    Vector3f phi; //orientation
    MatrixXf rotM; //rotation matrix
    MatrixXf TH(8,6); //joint angles
    EEPose eePose1;

    /*current position equals to homing procedure position*/
    currentJoint = Th0;

    /*create message to publish*/
    std_msgs::Float64MultiArray msg;
    msg.data.resize(9); //6 joint angles + 3 end effector joints
    msg.data.assign(9,0); //empty the msg

    cout << "Moving to target -> " << targetPosition << endl;

    /*loop to calculate and publish the joint angles*/
    for(float t=0; t<=1; t+=step){

        x = xe(t, targetPosition, eePose.Pe); //linear interpolation of the position
        phi = phie(t, targetOrientation, phie0); //linear intepolation of the orientation
        
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
                minSol = TH.row(i) - currentJoint;

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
        currentJoint = TH.row(minDistRow);

        /*ROS loop*/
        publishJoint(currentJoint);
    }

    /*close the gripper when in position*/
    //changeGripper(0,0);

    return currentJoint;
}

/**
 * @brief Compute the movement using the differential kinematics
 *
 * @param targetPosition 
 * @param targetOrientation 
 * @param dt
 */
void computeMovementDifferential(Vector3f targetPosition, Vector3f targetOrientation,float dt){

    /*calc initial end effector pose*/
    EEPose eePose;
    eePose = fwKin(currentJoint);

    /*calc x0 and phie0*/
    Vector3f x0,phie0;
    x0 = eePose.Pe;
    phie0 = eePose.Re.eulerAngles(2,1,0);

    /*Matrix to coorect the trajectory which otherwise is badly approximated*/
    MatrixXf kp(3,3);
    kp = MatrixXf::Identity(3,3)*40;
    MatrixXf kphi(3,3);
    kphi = MatrixXf::Identity(3,3)*5;

    /*parameters for the loop*/
    Vector3f x; //position
    MatrixXf re(3,3); //rotation matrix
    EEPose eePose1;

    MatrixXf qk(1,6);
    MatrixXf qk1(1,6);
    Vector3f vd;
    MatrixXf dotqk(6,6);
    Vector3f xArg;

    qk = currentJoint; //initialize qk

    for(float t=dt; t<=1; t+=dt){

        eePose1 = fwKin(qk);
        x = eePose1.Pe;
        re = eePose1.Re;

        vd = (xe(t,targetPosition,x0)-xe(t-dt,targetPosition,x0)) / dt;
        xArg = xe(t,targetPosition,x0);

        dotqk = invDiffKinematiControlComplete(qk,x,xArg,vd,re,targetOrientation,kp,kphi);
        qk1 = qk + dotqk.transpose()*dt; 
        qk = qk1;
       
        publishJoint(qk1);        
    }
    currentJoint = qk1;
}

/**
 * @brief 
 * 
 * @param q 
 * @param xe 
 * @param xd 
 * @param vd
 * @param re
 * @param kp 
 * @param kphi 
 * @return MatrixXf 
 */
MatrixXf invDiffKinematiControlComplete(MatrixXf q, MatrixXf xe, MatrixXf xd, MatrixXf vd, MatrixXf re, MatrixXf phif, MatrixXf kp, MatrixXf kphi){
    
    MatrixXf wRd(6,6);
    wRd = toRotationMatrix(phif);

    MatrixXf errorVector(3,1);
    errorVector = computeOrientationError(re,wRd);

    MatrixXf J(6,6);
    J = jacobian(q);
    
    /*dumped least squares inverse of J*/
    MatrixXf dotQ(6,1);
    MatrixXf ve(6,1);

    float k = pow(10,-6); //dumping factor

    if(errorVector.norm() > 0.1){
        errorVector = 0.1*errorVector.normalized();
    }

    ve << (vd+kp*(xd-xe)),
    (kphi*errorVector);
    
    dotQ = (J+MatrixXf::Identity(6,6)*k).inverse()*ve;

    /*limit the velocity of the joints*/
    for(int i = 0; i < 6; i++){
        if(dotQ(i,0) > M_PI){
            dotQ(i,0) = 3.;
        }
        if(dotQ(i,0) < -M_PI){
            dotQ(i,0) = -3.;
        }
    }

    return dotQ;

}

/**
 * @brief Compute the orientation error
 * 
 * @param wRe 
 * @param wRd 
 * @return MatrixXf 
 */

MatrixXf computeOrientationError(MatrixXf wRe, MatrixXf wRd){
    
    MatrixXf relativeOrientation(3,3);
    relativeOrientation = wRe.transpose()*wRd;

    //compute the delta angle
    float cosDTheta = (relativeOrientation(0,0)+relativeOrientation(1,1)+relativeOrientation(2,2)-1)/2;
    MatrixXf tmp(3,1);
    tmp << relativeOrientation(2,1)-relativeOrientation(1,2),
    relativeOrientation(0,2)-relativeOrientation(2,0),
    relativeOrientation(1,0)-relativeOrientation(0,1);
    float senDTheta = tmp.norm()/2;

    float dTheta = atan2(senDTheta,cosDTheta);

    if(dTheta == 0){
        return MatrixXf::Zero(3,1);
    }else{
        MatrixXf axis(3,1);
        axis = (1/(2*senDTheta))*tmp;
        return wRe * axis * dTheta;
    }
}

/**
 * @brief Calculate the jacobian
 * 
 * @param Th 
 * @return MatrixXf 
 */
MatrixXf jacobian(MatrixXf Th){
    MatrixXf A(1,6);
    MatrixXf D(1,6) ;
    A << 0,-0.425,-0.3922,0,0,0;
    D << 0.1625,0,0,0.1333,0.0997,0.0996+0.14;

    MatrixXf J1(6,1);    
    J1 << D(4)*(cos(Th(0))*cos(Th(4)) + cos(Th(1)+Th(2)+Th(3))*sin(Th(0))*sin(Th(4))) + D(2)*cos(Th(0)) + D(3)*cos(Th(0)) - A(2)*cos(Th(1)+Th(2))*sin(Th(0)) - A(1)*cos(Th(1))*sin(Th(0)) - D(4)*sin(Th(1)+Th(2)+Th(3))*sin(Th(0)),
        D(4)*(cos(Th(4))*sin(Th(0)) - cos(Th(1)+Th(2)+Th(3))*cos(Th(0))*sin(Th(4))) + D(2)*sin(Th(0)) + D(3)*sin(Th(0)) + A(2)*cos(Th(1)+Th(2))*cos(Th(0)) + A(1)*cos(Th(0))*cos(Th(1)) + D(4)*sin(Th(1)+Th(2)+Th(3))*cos(Th(0)),
        0,
        0,
        0,
        1;
    

    MatrixXf J2(6,1);
    J2 << -cos(Th(0))*(A(2)*sin(Th(1)+Th(2)) + A(1)*sin(Th(1)) + D(4)*(sin(Th(1)+Th(2))*sin(Th(3)) - cos(Th(1)+Th(2))*cos(Th(3))) - D(4)*sin(Th(4))*(cos(Th(1)+Th(2))*sin(Th(3)) + sin(Th(1)+Th(2))*cos(Th(3)))),
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
MatrixXf toRotationMatrix(Vector3f euler){
    Eigen::Matrix3f m;
    m = Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitX());
    return m;
}

/**
 * @brief publishJoint the desired joint state
 * 
 * @param publishPos
 */
void publishJoint(MatrixXf publishPos){

    std_msgs::Float64MultiArray msg;
    ros::Rate loop_rate(LOOPRATE);

    if(HARD_GRIPPER){
        msg.data.resize(ROBOT_JOINTS+EE_HARD_JOINTS); //6 joint angles + 3 hard gripper angles
        msg.data.assign(ROBOT_JOINTS+EE_HARD_JOINTS,0); //empty the msg
        

        for (int i = 0; i < ROBOT_JOINTS; i++){
            msg.data[i] = publishPos(0, i);
        }
        for(int i=0; i<EE_HARD_JOINTS; i++){
            msg.data[i+ROBOT_JOINTS] = currentGripper(i);
        }
    }else{
        msg.data.resize(ROBOT_JOINTS); //6 joint angles
        msg.data.assign(ROBOT_JOINTS,0); //empty the msg

        for (int i = 0; i < ROBOT_JOINTS; i++){
            msg.data[i] = publishPos(0, i);
        }
    }

    pub_des_jstate.publish(msg); // publish the message

    loop_rate.sleep(); // sleep for the time remaining to let us hit our 1000Hz publish rate
}

void publishMoveOperation(int blockId, bool success){

    cpp_publisher::MoveOperation msg;
    std_msgs::Byte byteMsg;
    std_msgs::String stringMsg;

    byteMsg.data = blockId;

    if(success){
        stringMsg.data = "success";
    }else{
        stringMsg.data = "fail - Something went wrong";
    }

    msg.blockId = byteMsg;
    msg.result = stringMsg;

    pub_move_operation.publish(msg);
}

/**
 * @brief changeSoftGripper
 * 
 * @param firstVal 
 * @param secondVal 
 */
void changeSoftGripper(float firstVal,float secondVal){

    ros::Rate loop_rate(LOOPRATE);

    std_msgs::Float64MultiArray msg;
    msg.data.resize(EE_SOFT_JOINTS); //6 joint angles + 3 end effector joints
    msg.data.assign(EE_SOFT_JOINTS,0); //empty the msg

    msg.data[0] = firstVal;
    msg.data[1] = secondVal;

    //pub_des_jstate.publish(msg); //to do -> change publisher with new topic

    loop_rate.sleep(); // sleep for the time remaining to let us hit our 1000Hz publish rate
}

/**
 * @brief close the gripper
 * 
 * @param currentJoint 
 */

void changeHardGripper(Vector3f ee_joints){

    ros::Rate loop_rate(LOOPRATE);

    std_msgs::Float64MultiArray msg;
    msg.data.resize(ROBOT_JOINTS+EE_HARD_JOINTS); //6 joint angles + 3 end effector joints
    msg.data.assign(ROBOT_JOINTS+EE_HARD_JOINTS,0); //empty the msg

    for(int i = 0; i < ROBOT_JOINTS; i++)
        msg.data[i] = currentJoint(0,i);

    msg.data[ROBOT_JOINTS+0] = ee_joints(0);
    msg.data[ROBOT_JOINTS+1] = ee_joints(1);
    msg.data[ROBOT_JOINTS+2] = ee_joints(2);

    currentGripper = ee_joints;

    pub_des_jstate.publish(msg); //to do -> change publisher with new topic

    loop_rate.sleep(); // sleep for the time remaining to let us hit our 1000Hz publish rate
}

/**
 * @brief This function computes the angles of the gripper joint based on the diameter given
 * 
 * @param diameter 
 * @return Vector3f 
 */
Vector3f mapToGripperJoints(float diameter){
    float alpha = (diameter - 22) / (130 - 22) * (-M_PI) + M_PI;
    Vector3f gripperJoints = Vector3f::Ones(3) * alpha;
    return gripperJoints;
}

/**
 * @brief The position to be reach at an instance t whilst moving from xe0 to xef (linear interpolation of the position)
 * 
 * @param t current time instance
 * @param xef final position
 * @param xe0 initial position
 * @return Vector3f reprensenting the position
 */
Vector3f xe(float t, Vector3f xef, Vector3f xe0){
    Vector3f x;
    x = t * xef + (1-t) * xe0;
    return x;
}

/**
 * @brief linear interpolation of the orientation
 * 
 * @param t
 * @param Rf
 * @param R0
 * @return Vector3f
 */
Vector3f phie(float t, Vector3f phief, Vector3f phie0){
    Vector3f phie;
    phie = t * phief + (1-t) * phie0;
    return phie;
}

/**
 * @brief callback for the joint state subscriber to print the current joint state
 * 
 * @param msg 
 */

void coordinateCallback(const cpp_publisher::Coordinates::ConstPtr& msg){

    cout << "Received coordinates" << endl;

    Vector3f pos,target;
    pos << msg->from.x, msg->from.y, msg->from.z;
    target << msg->to.x, msg->to.y, msg->to.z;

    Vector3f ori = Vector3f::Zero();

    //Adding 0.01 to the z coordinate to avoid collision with the table
    pos(2) += 0.01;
    target(2) += 0.01;

    pos = transformationWorldToBase(pos);
    target = transformationWorldToBase(target);

    cout << "Moving block " << msg->blockId.data << endl;

    moveObject(pos, ori, target);

    publishMoveOperation(msg->blockId.data, true);


}

void moveObject(Vector3f pos, Vector3f ori, Vector3f targetPos){

    EEPose eePose;

    cout << "Moving object from " << pos.transpose() << " to " << targetPos.transpose() << endl;

    // Kinematics
    cout << "Starting kinematics" << endl;

    //Moving in x,y
    cout << "Moving in x,y" << endl;
    Vector3f tmp = pos;
    tmp(2) -= 0.2;
    computeMovementDifferential(pos, ori, 0.001);

    //moving in z
    cout << "Moving in z" << endl;
    moveDown(0.2);

    // Grasping
    cout << "Grasping object" << endl;
    Vector3f gripperJoints;
    gripperJoints(0)=gripperJoints(1)=gripperJoints(2)=2.7;
    changeHardGripper(gripperJoints);
    sleep(1);

    //moving in z
    cout << "Moving in z" << endl;
    moveUp(0.2);
    if(DEBUG)sleep(2);

    //moving in x,y in a safer position
    cout << "Moving in x,y" << endl;
    eePose = fwKin(currentJoint);
    Vector3f tmp = eePose.Pe;
    //if the object is too close to the robot, move it away to avoid singularities
    if(tmp(1)>-0.4){
        tmp(1) = -0.5;
        computeMovementDifferential(tmp, Vector3f::Zero(), 0.001);
    }


    //moving in x,y
    tmp = targetPos;
    eePose = fwKin(currentJoint);
    tmp(2) = eePose.Pe(2);
    computeMovementDifferential(tmp, Vector3f::Zero(), 0.001);
    if(DEBUG)sleep(2);

    // Moving to target in z
    cout << "Moving to target" << endl;
    computeMovementDifferential(targetPos, Vector3f::Zero(), 0.001);
    if(DEBUG)sleep(2);

    // Releasing
    cout << "Releasing object" << endl;
    changeHardGripper(Vector3f::Ones()*1.8);
    sleep(1);

    // Moving up
    cout << "Moving up" << endl;
    moveUp(0.2);

    // Move in a safe position to take the next object
    cout << "Moving in a safe position, waiting for other objects" << endl;
    eePose = fwKin(currentJoint);
    Vector3f currentPos = eePose.Pe;
    if(currentPos(1)>-0.4){
        eePose.Pe(1) = -0.5;
        computeMovementDifferential(eePose.Pe, Vector3f::Zero(), 0.001);
    }
    

}

void moveUp(float distance){

    EEPose eepose = fwKin(currentJoint);
    Vector3f target = eepose.Pe;
    target(2) -= distance;

    computeMovementDifferential(target, eepose.Re.eulerAngles(2,1,0), 0.001);
}

void moveDown(float distance){

    EEPose eepose = fwKin(currentJoint);
    Vector3f target = eepose.Pe;
    target(2) += distance;

    computeMovementDifferential(target, eepose.Re.eulerAngles(2,1,0), 0.001);
}
