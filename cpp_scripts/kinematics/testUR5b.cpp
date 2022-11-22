#include <iostream>
#include <Eigen>
#include <cmath>
#include <iomanip>
#include "kinematicsUr5.cpp"

#define SAMPLES 100

using namespace std;
using Eigen::MatrixXf;

//struct model
/*struct EEPose{
    MatrixXf Pe;
    MatrixXf Re;
};*/

MatrixXf xe(float t, MatrixXf xef, MatrixXf xe0); //linear interpolation of the position
MatrixXf phie(float t, MatrixXf phief, MatrixXf phie0); //linear interpolation of the orientation

int main(int argc, char **argv){

    EEPose eePose;

    //initial pose
    MatrixXf Th0(1,6);
    Th0 << 0, 0, 0, 0.01, 0.01, 0;

    eePose = fwKin(Th0);
    MatrixXf phie0;
    Eigen::Matrix3f tmp;
    tmp = eePose.Re;
    phie0 = tmp.eulerAngles(0,1,2); //non funziona
    cout << "phie0: " << endl << phie0 << endl;
    // phie0.transposeInPlace();
    // cout << "phie0: " << endl << phie0 << endl;

    //target position
    MatrixXf xef(1,3);
    MatrixXf phief(1,3);
    xef << 0.5, 0.5, 0;
    phief << M_PI, M_PI_4, 3*M_PI_4;

    //temporaneo finchè matteo non fa il suo lavoro
    //MatrixXf phie0(1,3);
    //phie0 << -0.01, -0.01, -1.5709;

    MatrixXf x(1,3);
    MatrixXf phi(1,3);
    MatrixXf TH(8,6);
    MatrixXf Th;
    EEPose eePose1;

    for(float t=0; t<0.0202; t += 0.0101){ //t <= 1.0101
        //cout << "t: " << t << endl; 
        x = xe(t, xef, eePose.Pe);
        phi = phie(t, phief, phie0);
        x.transposeInPlace();
        //phi.transposeInPlace();
        eePose1.Pe = x;
        eePose1.Re = phi; //dobbiamo usare eul2rotm se no non funziona
        //cout << "phi: " << phi << endl;
        //TH = invKin(eePose1); //core dump se non usaimo eul2rotm
        //Th = Th, TH.row(0);
        //cout << "Th: " << Th << endl;
    }

    return 0;
}

//linear interpolation of the position
MatrixXf xe(float t, MatrixXf xef, MatrixXf xe0){
    xe0.transposeInPlace();
    MatrixXf x(1,3);
    x = t * xef + (1-t) * xe0;
    return x;
}

//linear interpolation of the orientation
MatrixXf phie(float t, MatrixXf phief, MatrixXf phie0){
    phie0.transposeInPlace();
    MatrixXf phi(1,3);
    phi = t * phief + (1-t) * phie0;
    return phi;
}
