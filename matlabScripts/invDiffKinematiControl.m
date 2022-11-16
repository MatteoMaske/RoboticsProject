% Computation of the controlled values for the translation motion
% A:parameters
% Jac: jacobian (function of q)
% q: position of the joints
% xe: position of the end effector
% xd: desired position of the end effector
% vd  desired velocity fo the end effector
% K: Positive definite matrix to reduce the error
% dotQ: velocity to be appleid at the joints
function [dotQ] = invDiffKinematiControl(Jac,q, xe, xd, vd, K)
    [J] = Jac(q);
    J = J(1:3, 1:6);
    dotQ = pinv(J)*(vd+K*(xd-xe));
end