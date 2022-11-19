%Direct Kinematics of the UR5
%Th: six joint angles
%pe: cartesian position of the end effector
%Re: Rotation matrix of the end effecto
function [pe,Re] = ur5Direct(Th)
%Vector of the a distance (expressed in metres)
A = [0, -0.425, -0.3922, 0, 0, 0];
%Vector of the D distance (expressed in metres)
D = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];

alfa = [0, pi/2, 0, 0, pi/2, -pi/2];


T10f = @(th1)  [
    cos(th1), -sin(th1), 0, 0;
    sin(th1), cos(th1), 0, 0;
    0, 0, 1, D(1);
    0, 0, 0, 1
    ];
T21f = @(th2) [
    cos(th2), -sin(th2), 0, 0;
    0, 0, -1, 0;
    sin(th2), cos(th2), 0, 0;
    0, 0, 0, 1
    ];

T32f = @(th3) [
    cos(th3), -sin(th3), 0, A(2);
    sin(th3), cos(th3), 0, 0;
    0, 0, 1, D(3);
    0, 0, 0, 1
    ];

T43f = @(th4) [
    cos(th4), -sin(th4), 0, A(3);
    sin(th4), cos(th4), 0, 0;
    0, 0, 1, D(4);
    0, 0, 0, 1
    ];

T54f = @(th5) [
    cos(th5), -sin(th5), 0, 0;
    0, 0, -1, -D(5);
    sin(th5), cos(th5), 0, 0;
    0, 0, 0, 1
    ];
T65f =  @(th6) [
    cos(th6), -sin(th6), 0, 0;
    0, 0, 1, D(6);
    -sin(th6), -cos(th6), 0, 0;
    0, 0, 0, 1
    ];





T10m = T10f(Th(1));

T21m = T21f(Th(2));

T32m = T32f(Th(3));

T43m = T43f(Th(4));

T54m = T54f(Th(5));

T65m = T65f(Th(6));

T06 =T10m*T21m*T32m*T43m*T54m*T65m;

pe = T06(1:3,4);
  Re = T06(1:3, 1:3);
end
