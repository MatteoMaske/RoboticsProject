%Computation of the Jacobian
function [J] = ur5Jac(Th)
 A = [0, -0.425, -0.3922, 0, 0, 0];
 %Vector of the D distance (expressed in metres)
 D = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
 
 A1 = A(1); A2 = A(2); A3 = A(3); A4 = A(4); A5 = A(5);  A6 = A(6);
 D1 = D(1); D2 = D(2); D3 = D(3); D4 = D(4); D5 = D(5);  D6 = D(6);
 
 th1 = Th(1);
 th2 = Th(2);
 th3 = Th(3);
 th4 = Th(4);
 th5 = Th(5);
 th6 = Th(6);
 
 J1 = [
     D5*(cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5)) + D3*cos(th1) + D4*cos(th1) - A3*cos(th2 + th3)*sin(th1) - A2*cos(th2)*sin(th1) - D5*sin(th2 + th3 + th4)*sin(th1);
     D5*(cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5)) + D3*sin(th1) + D4*sin(th1) + A3*cos(th2 + th3)*cos(th1) + A2*cos(th1)*cos(th2) + D5*sin(th2 + th3 + th4)*cos(th1);
     0;
     0;
     0;
     1];
 J2 = [
     -cos(th1)*(A3*sin(th2 + th3) + A2*sin(th2) + D5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - D5*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4)));
     -sin(th1)*(A3*sin(th2 + th3) + A2*sin(th2) + D5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - D5*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4)));
     A3*cos(th2 + th3) - (D5*sin(th2 + th3 + th4 + th5))/2 + A2*cos(th2) + (D5*sin(th2 + th3 + th4 - th5))/2 + D5*sin(th2 + th3 + th4);
     sin(th1);
     -cos(th1);
     0];
 J3 = [
     cos(th1)*(D5*cos(th2 + th3 + th4) - A3*sin(th2 + th3) + D5*sin(th2 + th3 + th4)*sin(th5));
     sin(th1)*(D5*cos(th2 + th3 + th4) - A3*sin(th2 + th3) + D5*sin(th2 + th3 + th4)*sin(th5));
     A3*cos(th2 + th3) - (D5*sin(th2 + th3 + th4 + th5))/2 + (D5*sin(th2 + th3 + th4 - th5))/2 + D5*sin(th2 + th3 + th4);
     sin(th1);
     -cos(th1);
     0];
 J4 = [
     D5*cos(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5));
     D5*sin(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5));
     D5*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4) - sin(th2 + th3 + th4 + th5)/2);
     sin(th1);
     -cos(th1);
     0];
 J5 = [
     -D5*sin(th1)*sin(th5) - D5*cos(th2 + th3 + th4)*cos(th1)*cos(th5);
     D5*cos(th1)*sin(th5) - D5*cos(th2 + th3 + th4)*cos(th5)*sin(th1);
     -D5*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4 + th5)/2);
     sin(th2 + th3 + th4)*cos(th1);
     sin(th2 + th3 + th4)*sin(th1);
    -cos(th2 + th3 + th4)
    ];
J6 = [
    0;
    0;
    0;
    cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5);
    -cos(th1)*cos(th5) - cos(th2 + th3 + th4)*sin(th1)*sin(th5);
    -sin(th2 + th3 + th4)*sin(th5)
    ];
J = [J1 J2 J3 J4 J5 J6];









  


