#include <iostream>
#include <Eigen/Dense>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <cpp_publisher/Coordinates.h>

#include "kinematicsUr5.cpp"
#include "frame2frame.cpp"

#define DEBUG 1 //debug mode

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
MatrixXf currentJoint(1,6); //current joint angles
MatrixXf currentGripper; //current gripper joint angles

//=======FUNCTION DECLARATION=======
Vector3f xe(float t, Vector3f xef, Vector3f xe0); //linear interpolation of the position
Vector3f phie(float t, Vector3f phief, Vector3f phie0); //linear interpolation of the orientation
MatrixXf toRotationMatrix(Vector3f euler); //convert euler angles to rotation matrix
MatrixXf computeMovementInverse(MatrixXf Th0, Vector3f targetPosition, Vector3f targetOrientation);//compute the movement
MatrixXf computeMovementDifferential(Vector3f targetPosition, Vector3f targetOrientation,float dt);//compute the movement
MatrixXf invDiffKinematiControlComplete(MatrixXf q, MatrixXf xe, MatrixXf xd, MatrixXf vd, MatrixXf re, MatrixXf phif, MatrixXf kp, MatrixXf kphi);
MatrixXf computeOrientationError(MatrixXf wRe, MatrixXf wRd);
MatrixXf jacobian(MatrixXf Th);

void coordinateCallback(const cpp_publisher::Coordinates::ConstPtr& msg);
void publish(MatrixXf publishPos); //publish the joint angles
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
void changeSoftGripper(float firstVal, float secondVal);
void changeHardGripper(MatrixXf joints, Vector3f ee_joints);
Vector3f mapToGripperJoints(float diameter);

//=======MAIN FUNCTION=======
int main(int argc, char **argv){

    //ROS initialization
    ros::init(argc, argv, "move");
    ros::NodeHandle node;
    pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1); //publisher for desired joint state

    ros::Subscriber jointSubscriber = node.subscribe("/ur5/joint_states", 1, jointStateCallback); //subscriber for joint state

    ros::Subscriber coordinateSubscriber = node.subscribe("/position", 1, coordinateCallback); //subscriber for block position
    
    /*initial joint angles*/
    MatrixXf Th0(1,6);
    Th0 << -0.322, -0.7805, -2.5675, -1.634, -1.571, -1.0017; //homing procedure joint angles

    /*initial gripper pos*/
    currentJoint = Th0;
    if(HARD_GRIPPER) currentGripper.resize(1,3);
    else currentGripper.resize(1,2);

    if(DEBUG){
        int input;
        while(1){
            ros::spinOnce();
            do{
                ros::spinOnce();

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
                if(refFrame == 0) pos = transformationWorldToBase(pos);

                computeMovementDifferential(pos, ori ,0.001); //compute the movement to the first brick in tavolo_brick.world
            }
            else if(input == 2){
                //homingProcedure(0.001,0.6, Th0, currentJoint);
                MatrixXf homePosition(1,6);
                homePosition << -0.322, -0.7805, -2.5675, -1.634, -1.571, -1.0017;
                publish(homePosition);
                currentJoint = homePosition;
            }else if(input==3){
                EEPose eePose;
                eePose = fwKin(currentJoint);
                cout << "Current ee position: " << endl;
                cout << eePose.Pe.transpose() << endl;
            }else if(input == 4){
                ros::spinOnce();
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
                    Vector3f ee_joints;
                    cout << "Insert the value of the gripper joints:" << endl;
                    cin >> diameter;
                    ee_joints(0) = ee_joints(1) = ee_joints(2) = diameter;
                    // cout << "Insert the diameter of the object in mm: " << endl;
                    // cin >> diameter;
                    // ee_joints = mapToGripperJoints(diameter);
                    // cout << "Gripper joints: " << endl;
                    // cout << ee_joints.transpose() << endl;
                    changeHardGripper(currentJoint,ee_joints);
                }else{
                    cout << "Insert the value of the gripper joints:" << endl;
                    float value,value2;
                    cin >> value >> value2;
                }
            }else if(input == 7){
                EEPose eePose;
                eePose = fwKin(currentJoint);
                Vector3f tmp = eePose.Pe;
                Vector3f phie = eePose.Re.eulerAngles(2,1,0);
                cout << "Current ee position: " << endl;
                cout << tmp.transpose() << endl;
                tmp(2) -= 0.1;
                cout << "New ee position: " << endl;
                cout << tmp.transpose() << endl;
                computeMovementDifferential(tmp, phie,0.001);
            }else if(input == 8){
                EEPose eePose;
                eePose = fwKin(currentJoint);
                Vector3f tmp = eePose.Pe;
                Vector3f phie = eePose.Re.eulerAngles(2,1,0);
                tmp(2)+= 0.1;
                computeMovementDifferential(tmp, phie ,0.001);   
            }else{
                break;
            }
            ros::spinOnce();
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
        publish(currentJoint);
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
 * @return MatrixXf 
 */
MatrixXf computeMovementDifferential(Vector3f targetPosition, Vector3f targetOrientation,float dt){
    
    cout << "Moving to target with differential kinematics-> " << targetPosition << endl;

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
       
        publish(qk1);        
    }
    currentJoint = qk1;
    MatrixXf positionReached = fwKin(qk1).Pe.transpose();
    MatrixXf orientationReached = fwKin(qk1).Re.eulerAngles(2,1,0).transpose();
    cout << "Target reached -> " << positionReached << endl;
    cout << "Target orientation reached -> " << orientationReached << endl;
    return qk1;
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
 * @brief publish the desired joint state
 * 
 * @param publishPos
 */
void publish(MatrixXf publishPos){

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

void changeHardGripper(MatrixXf joint, Vector3f ee_joints){

    ros::Rate loop_rate(LOOPRATE);

    std_msgs::Float64MultiArray msg;
    msg.data.resize(ROBOT_JOINTS+EE_HARD_JOINTS); //6 joint angles + 3 end effector joints
    msg.data.assign(ROBOT_JOINTS+EE_HARD_JOINTS,0); //empty the msg

    for(int i = 0; i < ROBOT_JOINTS; i++)
        msg.data[i] = joint(0,i);

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
 * @brief linear interpolation of the position
 * 
 * @param t
 * @param xef
 * @param xe0
 * @return Vector3f
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

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){

    // MatrixXf lastPos(1,6);
    // cout << "Last joint position -> " << endl;
    // for (int i = 0; i < ROBOT_JOINTS; i++){
    //     cout << msg->position[i] << " ";
    //     lastPos(i) = msg->position[i];
    // }
    // currentJoint = lastPos;
    // cout << endl;

    // EEPose eepose = fwKin(lastPos);
    // cout << "Last ee position -> " << endl;
    // cout << eepose.Pe.transpose() << endl;

}

void coordinateCallback(const cpp_publisher::Coordinates::ConstPtr& msg){

    cout << "porcoddio" << endl;

    if(DEBUG){
        cout << "I have just received from " << msg->from.x << " " << msg->from.y << " " << msg->from.z << endl;
        cout << "to " << msg->to.x << " " << msg->to.y << " " << msg->to.z << endl;
    }

    float x = (float)msg->to.x;
    float y = (float)msg->to.y;
    float z = (float)msg->to.z;

    Vector3f target;
    target << x,y,z;

    Vector3f orientation = Vector3f::Zero();

    target = transformationWorldToBase(target);

    computeMovementDifferential(target,orientation,0.001);
}


