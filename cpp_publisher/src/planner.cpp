#include <iostream>
#include <ros/ros.h>
#include <cpp_publisher/Coordinates.h>

#define DEBUG 1

using namespace std;

void startTopic();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");

    startTopic();

    return 0;
}

void startTopic(){

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<cpp_publisher::Coordinates>("position", 100);
    ros::Rate loop_rate(0.5);

    if(DEBUG){
        cout << "Starting topic" << endl;
        cout << "Publishing first position" << endl;
        while(ros::ok()){
                cout << "Publishing" << endl;
                cpp_publisher::Coordinates msg;
                msg.from.x = 0.8;
                msg.from.y = 0.45;
                msg.from.z = 0.9;

                msg.to.x = 0.8;
                msg.to.y = 0.45;
                msg.to.z = 0.9;

                pub.publish(msg);
                loop_rate.sleep();
        }
    }

}


