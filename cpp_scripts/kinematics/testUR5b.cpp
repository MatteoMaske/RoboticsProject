#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "kinematicsUr5.cpp"

using namespace std;
using Eigen::MatrixXf;

//struct model
/*struct EEPose{
    MatrixXf Pe;
    MatrixXf Re;
};*/

MatrixXf xe(float t, MatrixXf xef, MatrixXf xe0); //linear interpolation of the position
MatrixXf phie(float t, MatrixXf phief, MatrixXf phie0); //linear interpolation of the orientation
Eigen::MatrixXf homingProcedure(float dt, float vDes, MatrixXf qDes, MatrixXf qRef); //using homingProcedure template to get to a desired position

int main(int argc, char **argv){

    EEPose eePose;

    //initial joint angles
    MatrixXf Th0(1,6);
    Th0 << -0.322, -0.7805, -2.5675, -1.634, -1.571, -1.0017; //homing procedure

    //calc initial end effector pose
    eePose = fwKin(Th0);

    //from rotation matrix to euler angles
    MatrixXf phie0;
    Eigen::Matrix3f tmp;
    tmp = eePose.Re;
    phie0 = tmp.eulerAngles(0,1,2); //il risultato è un po' diverso da quello di matlab
    //phie0 << -0.01, -0.01, 1.5709; //risultato di matlab
    //cout << "phie0: " << endl << phie0 << endl;

    //target position
    MatrixXf xef(1,3); //postion
    MatrixXf phief(1,3); //orientation
    xef << 0.19, 0.21, -0.7;  //potential position of the brick relative to world frame
    phief << 0, 0, 0; 

 /*    MatrixXf x(1,3);
    MatrixXf phi(1,3);
    MatrixXf TH(8,6);
    MatrixXf Th(0,6);
    EEPose eePose1;

    for(float t=0; t<1.0101; t += 0.0101){ //t <= 1.0101
    
        x = xe(t, xef, eePose.Pe); //linear interpolation of the position
        phi = phie(t, phief, phie0); //linear intepolation of the orientation
        x.transposeInPlace();
        
        //from euler angles to rotation matrix
        Eigen::Matrix3f m;
        m = Eigen::AngleAxisf(phi(0), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(phi(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(phi(2), Eigen::Vector3f::UnitX());
        //cout << "phi: " << endl << phi << endl;
        //cout << "m: " << endl << m << endl;

        //save the result in the end effector struct
        eePose1.Pe = x;
        eePose1.Re = m;
        
        //inverse kinematics
        TH = invKin(eePose1);
        //cout << "TH: " << endl << TH << endl;

        //concatenate the first row of TH to Th
        /*prende il primo risultato della invKin, si può fare un controllo per scegliere il risultato migliore degli 8
        Th.conservativeResize(Th.rows()+1, Th.cols());
        Th.row(Th.rows()-1) = TH.row(0);
    } */

    //cout << "Th: " << Th << endl;

    /**********************************/
    // EEPose eePose2;

    // MatrixXf temp1(1,3);
    // temp1 << 0.5, 0.5, 0;
    // eePose2.Pe = temp1;

    // MatrixXf temp(3,3);
    // tmp <<  0, 0, 1,
    //         0, 1, 0,
    //         -1, 0, 0;
    // eePose2.Re = tmp;

    // TH = invKin(eePose2);

    // cout << "TH: " << endl << TH << endl;
    /**********************************/

    MatrixXf possibleDest(8,6);
    possibleDest = invKin(eePose);

    MatrixXf test(1,6);
    test << 0, 0, 0, 0, 0, 0;

    cout << "selected destination: " << endl << possibleDest.row(0) << endl;


    homingProcedure(0.001, 0.6, possibleDest.row(0), Th0);

    return 0;
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
 * @brief homingProcedure
 * 
 * @param dt 
 * @param vDes 
 * @param qDes 
 * @param qRef
 * @return Eigen::MatrixXf 
 */

Eigen::MatrixXf homingProcedure(float dt, float vDes, MatrixXf qDes, MatrixXf qRef){

    Eigen::MatrixXf Th(0,6);
    Eigen::MatrixXf error (1,6);
    float errorNorm = 0.0;
    float vRef = 0.0;

    cout << "hello" << endl;
    int i = 0;

    do{            
        error = qDes - qRef;
        //cout << error << endl;
        errorNorm = error.norm();
        //cout << errorNorm << endl;
        vRef += 0.005*(vDes-vRef);
        qRef += dt*vRef*error/errorNorm;
        if(i++ < 100)
            cout << qRef << endl;

        //Th.conservativeResize(Th.rows()+1, Th.cols());
        //Th.row(Th.rows()-1) = qDes;
    }while(errorNorm > 0.001);

    //cout << "Th: " << Th << endl;
    return Th;
}