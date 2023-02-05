#include <iostream>
#include <ros/ros.h>

#include <cpp_publisher/Coordinates.h>
#include <cpp_publisher/BlockInfo.h>
#include <cpp_publisher/BlockDetected.h>

#include <Eigen/Dense>

#define DEBUG 1

using namespace std;
using Eigen::Vector3f;

void sendMoveOrder(Vector3f blockPos, int blockClass);
void visionCallback(const cpp_publisher::BlockDetected::ConstPtr& msg);
Vector3f getTargetZone(int blockClass);

ros::Publisher pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle n;
    pub = n.advertise<cpp_publisher::Coordinates>("position", 100);
    ros::Subscriber sub = n.subscribe("vision/vision_detection", 100, visionCallback);

    return 0;
}

void sendMoveOrder(Vector3f blockPos, int blockClass){

    if(DEBUG)cout << "Sending move order" << endl;
    
    if(pub.getNumSubscribers() > 0){
        cout << "Publishing" << endl;
        cpp_publisher::Coordinates msg;
        msg.from.x = blockPos(0);
        msg.from.y = blockPos(1);
        msg.from.z = blockPos(2);

        Vector3f target = getTargetZone(blockClass);

        msg.to.x = target(0);
        msg.to.y = target(1);
        msg.to.z = target(2);

        pub.publish(msg);
    }
}

Vector3f getTargetZone(int blockClass){

    Vector3f target;

    switch (blockClass)
    {
    case 1:
        target << -0.3, 0.45, 0.9;
        break;
    }

    return target;
}

void visionCallback(const cpp_publisher::BlockDetected::ConstPtr& msg){
    
    if(DEBUG){
        cout << "Received vision callback" << endl;
        cout << "Blocks detected:"<< endl;
        for(int i = 0 ; i < msg->blockDetected.size() ; i++){
            cout << "Block " << msg->blockDetected[i].id << endl;
            cout << "Class: " << msg->blockDetected[i].objectClass << endl;
            cout << "Position: " << msg->blockDetected[i].position.x << " " << msg->blockDetected[i].position.y << " " << msg->blockDetected[i].position.z << endl;
            cout << endl;
        }
    }

    for(int i = 0; i < msg->blockDetected.size(); i++){
        Vector3f blockPos;
        blockPos << msg->blockDetected[i].position.x, msg->blockDetected[i].position.y, msg->blockDetected[i].position.z;
        int tmp = (msg->blockDetected[i].objectClass.data);
        sendMoveOrder(blockPos, tmp);
        sleep(10);
    }
}


