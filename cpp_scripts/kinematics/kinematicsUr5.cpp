#include <iostream>
#include <Eigen>
#include <cmath>

using namespace std;
using Eigen::MatrixXf;
using Eigen::Vector4d;

const float A[6] = {0, -0.425, -0.3922, 0, 0, 0};
const float D[6] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};

MatrixXf Re(3,3);

void fwKin(float Th[6], float endEffectorPos[3]); // This function will calculate the forward kinematics of the robot and return the position of the end effector
void invKin(float endEffectorPos[3]); // This function will calculate the inverse kinematics of the robot and return the joint angles

int main(int argc, char** argv){

        float Th[6] = {1.6, 0.2, -0.5, 2.89, 1.1, 1.25};
        float endEffectorPos[3] = {0, 0, 0}; // This will be the position of the end effector

        fwKin(Th, endEffectorPos);

        invKin(endEffectorPos);

        //cout << "The end effector is at: " << endEffectorPos[0] << ", " << endEffectorPos[1] << ", " << endEffectorPos[2] << endl;

        return 0;
}

void fwKin(float Th[6], float endEffectorPos[3]){

        MatrixXf A10(4,4);
        MatrixXf A21(4,4);
        MatrixXf A32(4,4);
        MatrixXf A43(4,4);
        MatrixXf A54(4,4);
        MatrixXf A65(4,4);
        MatrixXf A60(4,4);

        MatrixXf Pe(1,4);
        

        A10 <<  cos(Th[0]), -sin(Th[0]), 0, 0,
                sin(Th[0]), cos(Th[0]), 0, 0,
                0, 0, 1, D[0],
                0, 0, 0, 1;

        A21 <<  cos(Th[1]), -sin(Th[1]), 0, 0,
                0, 0, -1, 0,
                sin(Th[1]), cos(Th[1]), 0, 0,
                0, 0, 0, 1;

        A32 <<  cos(Th[2]), -sin(Th[2]), 0, A[1],
                sin(Th[2]), cos(Th[2]), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

        A43 <<  cos(Th[3]), -sin(Th[3]), 0, A[2],
                sin(Th[3]), cos(Th[3]), 0, 0,
                0, 0, 1, D[3],
                0, 0, 0, 1;

        A54 <<  cos(Th[4]), -sin(Th[4]), 0, 0,
                0, 0, -1, -D[4],
                sin(Th[4]), cos(Th[4]), 0, 0,
                0, 0, 0, 1;

        A65 <<  cos(Th[5]), -sin(Th[ 5]), 0, 0,
                0, 0, 1, D[5],
                -sin(Th[5]), -cos(Th[5]), 0, 0,
                0, 0, 0, 1;

        A60 = A10*A21*A32*A43*A54*A65;

        Pe = A60.block(0,3,3,1);
        Re = A60.block(0,0,3,3);

        //cout << Pe << endl;
        //cout << Re << endl;

        endEffectorPos[0] = Pe(0,0);
        endEffectorPos[1] = Pe(1,0);
        endEffectorPos[2] = Pe(2,0);
}

void invKin(float endEffectorPos[3]){
        
        MatrixXf T60 (4,4);

        cout << "Re" <<endl<< Re <<endl;

        T60 << Re(0,0), Re(0,1), Re(0,2), endEffectorPos[0],
               Re(1,0), Re(1,1), Re(1,2), endEffectorPos[1],
               Re(2,0), Re(2,1), Re(2,2), endEffectorPos[2],
               0, 0, 0, 1;

        //finding th1
        MatrixXf p50 (1,4);
        MatrixXf temp(4,1);
        temp << 0, 0, -D[5], 1;

        p50 = T60 * temp;
        float th1_1 = atan2(p50(1,0), p50(0,0))+acos(D[3]/hypot(p50(1,0), p50(0,0)))+M_PI_2;
        float th1_2 = atan2(p50(1,0), p50(0,0))-acos(D[3]/hypot(p50(1,0), p50(0,0)))+M_PI_2;

        float th5_1 = acos((endEffectorPos[0]*sin(th1_1) - endEffectorPos[1]*cos(th1_1)-D[3]) / D[5]);
        float th5_2 = -acos((endEffectorPos[0]*sin(th1_1) - endEffectorPos[1]*cos(th1_1)-D[3]) / D[5]);
        float th5_3 = acos((endEffectorPos[0]*sin(th1_2) - endEffectorPos[1]*cos(th1_2)-D[3]) / D[5]);
        float th5_4 = -acos((endEffectorPos[0]*sin(th1_2) - endEffectorPos[1]*cos(th1_2)-D[3]) / D[5]);

        //related to th11 a th51
        MatrixXf T06(4,4);
        MatrixXf Xhat (3,1);
        MatrixXf Yhat (3,1);

        T06 = T60.inverse();        
        Xhat = T06.block(0,0,3,1);        
        Yhat = T06.block(0,1,3,1);
        cout << "Xhat" <<endl<< Xhat <<endl;
        cout << "Yhat" <<endl<< Yhat <<endl;


        float th6_1 = atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1)))/sin(th5_1), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1)))/sin(th5_1));
        float th6_2 = atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1)))/sin(th5_2), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1)))/sin(th5_2));
        float th6_3 = atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2)))/sin(th5_3), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2)))/sin(th5_3));
        float th6_4 = atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2)))/sin(th5_4), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2)))/sin(th5_4));
        
        


        


}
