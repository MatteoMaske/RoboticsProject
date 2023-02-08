#include <iostream>
#include <condition_variable>
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <cpp_publisher/Coordinates.h>
#include <cpp_publisher/BlockInfo.h>
#include <cpp_publisher/BlockDetected.h>
#include <cpp_publisher/MoveOperation.h>

#include <Eigen/Dense>

#include <vector>

#define DEBUG 1
#define BLOCK_CLASSES 10

using namespace std;
using Eigen::Vector3f;

void sendMoveOrder(Vector3f blockPos, int blockClass, int blockId);
void visionCallback(const cpp_publisher::BlockInfo::ConstPtr& msg);
void movementCallback(const cpp_publisher::MoveOperation::ConstPtr& msg);
Vector3f getTargetZone(int blockClass);

ros::Publisher movePublisher;
ros::Publisher visionPublisher;
vector<int> blockPerClass(BLOCK_CLASSES, 0);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle n;

    movePublisher = n.advertise<cpp_publisher::Coordinates>("/planner/position", 100);

    visionPublisher = n.advertise<std_msgs::Bool>("/planner/detection_request", 100);

    ros::Subscriber visionSubscriber = n.subscribe("/vision/vision_detection", 100, visionCallback);

    ros::Subscriber moveSubscriber = n.subscribe("/move/movement_results", 100, movementCallback);

    cout << "waiting for subscribers" << endl;
    while(ros::ok()){
        if(visionPublisher.getNumSubscribers() > 0){
            std_msgs::Bool msg;
            msg.data = true;
            if(DEBUG)cout << "Publishing detection request" << endl;
            visionPublisher.publish(msg);
            break;
        }
        ros::spinOnce();
    }

    ros::spin();

    return 0;
}

void sendMoveOrder(Vector3f blockPos, int blockClass, int blockId){

    if(DEBUG)cout << "Sending move order" << endl;
    
    while(ros::ok()){
        cout << "Waiting for subscribers" << endl;
        if(movePublisher.getNumSubscribers() > 0){
            cout << "Publishing" << endl;
            cpp_publisher::Coordinates msg;

            std_msgs::Byte byteMsg;
            byteMsg.data = blockId;
            msg.blockId = byteMsg;

            msg.from.x = blockPos(0);
            msg.from.y = blockPos(1);
            msg.from.z = blockPos(2);

            Vector3f target = getTargetZone(blockClass);

            msg.to.x = target(0);
            msg.to.y = target(1);
            msg.to.z = target(2);

            if(msg.from.x < 0.5)movePublisher.publish(msg);
            
            break;
        }
    }
}

Vector3f getTargetZone(int blockClass){

    Vector3f target;

    float offset = 0.07 * blockPerClass[blockClass-1]; // 0.05 is the offset between blocks of same class

    blockPerClass[blockClass-1]+=1; // Increment the number of blocks of this class

    switch (blockClass)
    {
    case 1:
        target << 0.9, 0.5+offset, 0.9;
        break;
    case 2:
        target << 0.9, 0.5+offset, 0.9;
        break;
    case 3:
        target << 0.9, 0.5+offset, 0.9;
        break;
    case 4:
        break;
    case 5:
        break;
    case 6:
        target << 0.9, 0.5+offset, 0.9;
        break;
    case 7:
        target << 0.9, 0.5+offset, 0.9;
        break;
    case 8:
        target << 0.9, 0.5+offset, 0.9;
        break;
    case 9:
        target << 0.9, 0.5+offset, 0.9;
        break;
    case 10:
        target << 0.8, 0.5+offset, 0.9;
        break;
    }

    return target;
}

void visionCallback(const cpp_publisher::BlockInfo::ConstPtr& msg){
    
    if(DEBUG){
        cout << "Received vision callback" << endl;
    }
    
    // Send move order
    Vector3f blockPos;
    blockPos << msg->blockPosition.x, msg->blockPosition.y, msg->blockPosition.z;
    int blockId = msg->blockId.data;
    int blockClass = msg->blockClass.data;

    sendMoveOrder(blockPos, blockClass, blockId);

}

void movementCallback(const cpp_publisher::MoveOperation::ConstPtr& msg){
    if(DEBUG)cout << "Received movement callback" << endl;

    cout << "Movement result: " << msg->result.data << endl;

    std_msgs::Bool boolMsg;
    boolMsg.data = true;
    visionPublisher.publish(msg->result);

}


