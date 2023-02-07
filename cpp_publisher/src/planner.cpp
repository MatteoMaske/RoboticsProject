#include <iostream>
#include <ros/ros.h>

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
void visionCallback(const cpp_publisher::BlockDetected::ConstPtr& msg);
void movementCallback(const cpp_publisher::MoveOperation::ConstPtr& msg);
Vector3f getTargetZone(int blockClass);

ros::Publisher pub;
vector<int> blockPerClass(BLOCK_CLASSES, 0);
bool movementSucceded = false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle n;

    pub = n.advertise<cpp_publisher::Coordinates>("/planner/position", 100);

    ros::Subscriber visionSubscriber = n.subscribe("vision/vision_detection", 100, visionCallback);

    ros::Subscriber moveSubscriber = n.subscribe("/move/movement_results", 100, movementCallback);

    ros::spin();

    return 0;
}

void sendMoveOrder(Vector3f blockPos, int blockClass, int blockId){

    if(DEBUG)cout << "Sending move order" << endl;
    
    while(ros::ok()){
        if(pub.getNumSubscribers() > 0){
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

            pub.publish(msg);
            
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
    }

    return target;
}

void visionCallback(const cpp_publisher::BlockDetected::ConstPtr& msg){
    
    if(DEBUG){
        cout << "Received vision callback" << endl;
        // cout << "Blocks detected:"<< endl;
        // for(int i = 0 ; i < msg->blockDetected.size() ; i++){
        //     cout << "Block " << msg->blockDetected[i].id << endl;
        //     cout << "Class: " << msg->blockDetected[i].objectClass << endl;
        //     cout << "Position: " << msg->blockDetected[i].position.x << " " << msg->blockDetected[i].position.y << " " << msg->blockDetected[i].position.z << endl;
        //     cout << endl;
        // }
    }

    for(int i = 0; i < msg->blockDetected.size(); i++){
        Vector3f blockPos;
        blockPos << msg->blockDetected[i].blockPosition.x, msg->blockDetected[i].blockPosition.y, msg->blockDetected[i].blockPosition.z;
        int objectClass = (msg->blockDetected[i].blockClass.data);
        int blockId = msg->blockDetected[i].blockId.data;
        sendMoveOrder(blockPos, objectClass, blockId);
        movementSucceded = false;
        while(ros::ok()){
            ros::spinOnce();
            if(movementSucceded)break;
        }
    }
}

void movementCallback(const cpp_publisher::MoveOperation::ConstPtr& msg){
    if(DEBUG)cout << "Received movement callback" << endl;
    
    cout << "Movement result: " << msg->result << endl;

    if(msg->result.data == "success")movementSucceded = true;
    
}


