% Point 2 Point motion Plan
% DK: function for direct kinematics
% IK: function for inverse kineamtics
% xEs, phiEs: starting configuration for the end effector
% xEf, phiEf: final configuration for the end effector
% T: duration of the motion paln
% dt: sampling time
function [Th,xE, phiE] = p2pMotionPlan(DK, IK, xEs, phiEs, xEf, phiEf, minT, maxT, dt)

qEs = IK(xEs,eul2rotm(phiEs'));
qEf = IK(xEf, eul2rotm(phiEf'));
qEs = qEs(1,:);
qEf = qEf(1,:);
A = [];
for i = 1:length(qEs(1,:)),
    M = [1, minT, minT^2, minT^3;
        0, 1, 2*minT, 3*minT^2;
        1, maxT, maxT^2, maxT^3;
        0, 1, 2*maxT, 3*maxT^2;];
    b = [qEs(i); 0; qEf(i); 0];
    a =  inv(M)*b;
    A = [A; a'];
end
Th = [];
xE = [];
phiE = [];
for t = minT:dt:maxT,
    th = [t];
    for i = 1:length(qEs),
        q = A(i,1)+A(i,2)*t+A(i,3)*t*t+A(i,4)*t*t*t;
        th = [th q];
    end
    Th = [Th; th];
    [mx, mR] = DK(th(2:7));
    xE = [xE; t, mx'];
    phiE = [phiE; t, rotm2eul(mR)];
end
display('ciao');
