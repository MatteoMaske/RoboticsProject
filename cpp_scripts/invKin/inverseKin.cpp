#include <iostream>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>

using namespace std;

struct robotParams{
    float armLenght[3];
    float wristPos[3];
}robotParams;

void inverseKin(float *, float *);

int main(int argc, char** argv){

    robotParams.armLenght[0] = 0.5;
    robotParams.armLenght[1] = 0.5;
    robotParams.armLenght[2] = 0.5;

    robotParams.wristPos[0] = 0.5;
    robotParams.wristPos[1] = 0.5;
    robotParams.wristPos[2] = 0.5;

    inverseKin(robotParams.armLenght, robotParams.wristPos);


    return 0;
}

// fun to compute inverse kinematics taking armlength and wrist position as input
void inverseKin(float * armLenght, float * wristPos){

    // saving wrist variables
    float wpx = wristPos[0];
    float wpy = wristPos[1];
    float wpz = wristPos[2];

    // saving arm variables
    float a0 = armLenght[0];
    float a1 = armLenght[1];
    float a2 = armLenght[2];

    float sin1, cos1, sin2, cos2, sin3, cos3;

    float th11, th12, th21, th22, th23, th24, th31, th32;

    cos3 = (pow(wpx,2) + pow(wpy,2) + pow(wpz,2) - pow(a0,2) - pow(a1,2) - pow(a2,2))/(2*a1*a2);
    sin3 = sqrt(1-pow(cos3,2));

    th31 = atan2(sin2, cos3);
    th32 = -th31;


    th21 = atan2((a1+a2*cos3)*wpz-a2*sin3*sqrt(pow(wpx,2)+pow(wpy,2)), (a1+a2*cos3)*sqrt(pow(wpx,2)+pow(wpy,2)+a2*sin3*wpx));
    /* th22 = atan2((a1+a2*cos3)*wpz+a2*sin3*sqrt(wpx^2+wpy^2), -(a1+a2*cos3)*sqrt(wpx^2+wpy^2)+a2*sin3*wpz);
    th23 = atan2((a1+a2*cos3)*wpz-a2*-sin3*sqrt(wpx^2+wpy^2), (a1+a2*cos3)*sqrt(wpx^2+wpy^2)+a2*-sin3*wpz);
    th24 = atan2((a1+a2*cos3)*wpz+a2*-sin3*sqrt(wpx^2+wpy^2), -(a1+a2*cos3)*sqrt(wpx^2+wpy^2)+a2*sin3*wpz);
 */
    th11 = atan2(wpy, wpx);
    th12 = atan2(-wpy, -wpx);

    float firstConf [4] = {th11, th21, th31};
    float secondConf [4] = {th11, th23, th32};
    float thirdConf [4] = {th12, th22, th31};
    float fourthConf [4] = {th12, th24, th32};

    cout << "First configuration: " << firstConf[0] << endl;


}