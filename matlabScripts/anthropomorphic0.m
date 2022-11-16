%Draws an athropomorphic arm
% th1, th2, th3: joint variables
%A : lengths of the links
%lim: limits of the axis
function [pe] = anthropomorphic0(Th, A )

a0 = A.a0; a2 = A.a2, a3 = A.a3;

th1=Th.th1; th2=Th.th2; th3 = Th.th3;

A0b = [eye(3,3), [0;0;a0]; [0 0 0] 1];

A10 = [[cos(th1) 0 sin(th1); sin(th1) 0 -cos(th1); 0 1 0], [0;0;0];
        [0, 0, 0], 1];
A21 = [[cos(th2) -sin(th2) 0; sin(th2) cos(th2) 0; 0 0 1], [a2*cos(th2);a2*sin(th2);0]; 
    [0 0 0] 1];
A32 =  [[cos(th3) -sin(th3) 0; sin(th3) cos(th3) 0; 0 0 1], [a3*cos(th3);a3*sin(th3);0]; 
    [0 0 0] 1];
A30 = A10*A21*A32;
Oe = A30*[0;0;0;1];
pe = [Oe(1:3)];
end