 syms A1 A2 A3 A4 A5 A6;
 syms D1 D2 D3 D4 D5 D6;
 assume(A1, 'real');
 assume(A2, 'real');
 assume(A3, 'real');
 assume(A4, 'real');
 assume(A5, 'real');
 assume(A6, 'real');
 assume(D1, 'real');
 assume(D2, 'real');
 assume(D3, 'real');
 assume(D4, 'real');
 assume(D5, 'real');
 assume(D6, 'real');
 
 %Vector of the a distance (expressed in metres)
 %A = [0, -0.425, -0.3922, 0, 0, 0];
 A = [A1 A2 A3 A4 A5 A6];
 
 %Vector of the D distance (expressed in metres)
 %D = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
 D = [D1 D2 D3 D4 D5 D5];
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

syms th1 th2 th3 th4 th5 th6;
assume(th1, 'real');
assume(th2, 'real');
assume(th3, 'real');
assume(th4, 'real');
assume(th5, 'real');
assume(th6, 'real');

T10m = T10f(th1);
T21m = T21f(th2);
T32m = T32f(th3);
T43m = T43f(th4);
T54m = T54f(th5)
T65m = T65f(th6);


z0 = [0;0;1];
p0 = [0; 0; 0];

z1 = T10m(1:3,3);
p1 = T10m(1:3,4);

T20m = T10m*T21m;
z2 = T20m(1:3,3);
p2 = T20m(1:3,4);

T30m = T20m*T32m;
z3 = T30m(1:3,3);
p3 = T30m(1:3,4);

T40m = T30m*T43m;
z4 = T40m(1:3,3);
p4 = T40m(1:3,4);

T50m = T40m*T54m;
z5 = T50m(1:3,3);
p5 = T50m(1:3,4);


T60m = T50m*T65m;
z6 = T60m(1:3,3);
p6 = T60m(1:3,4);

J1 = [cross(z1, p6-p1); z1];
J2 = [cross(z2, p6-p2); z2];
J3 = [cross(z3, p6-p3); z3];
J4 = [cross(z4, p6-p4); z4];
J5 = [cross(z5, p6-p5); z5];
J6 = [cross(z6, p6-p6); z6];

J = [J1 J2 J3 J4 J5 J6];

display(simplify(J1));
display(simplify(J2));
display(simplify(J3));
display(simplify(J4));
display(simplify(J5));
display(simplify(J6));

Js = simplify(J);
displayFormula("Js");
