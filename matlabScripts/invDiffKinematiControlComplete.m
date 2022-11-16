% Computation of the controlled values for the complete motion 
% A:parameters
% Jac: jacobian (function of q)
% q: position of the joints
% xe: position of the end effector
% xd: desired position of the end effector
% vd  desired velocity fo the end effector
% phie: RPY (X-Y-Z rotation) int the fixed axis 
%           phi(1) -> X
%           phi(2) -> Y
%           phi(3) -> Z
% phid: Desired RPY
% K: Positive definite matrix to reduce the error
% dotQ: velocity to be appleid at the joints
function [dotQ] = invDiffKinematiControlComplete(Jac,q, xe, xd, vd, phie, phid, phiddot, Kp, Kphi)
    [J] = Jac(q);
    alpha = phie(3);
    beta = phie(2);
    gamma = phie(1);
    T = [cos(beta)*cos(gamma), -sin(gamma), 0;
        cos(beta)*sin(gamma), cos(gamma), 0;
        -sin(beta), 0, 1];
    Ta = [eye(3,3), zeros(3,3);
        zeros(3,3), T];
    Ja = inv(Ta)*J;
    dotQ = inv(Ja)*[(vd+Kp*(xd-xe));(phiddot+Kphi*(phid-phie))];
end