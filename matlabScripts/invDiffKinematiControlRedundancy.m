% Computation of the controlled values for the translation motion
% A:parameters. We exploit redundancy to attain some secondary goal, which
% is encoded in the solutions q0
% Jac: jacobian (function of q)
% q: position of the joints
% xe: position of the end effector
% xd: desired position of the end effector
% vd  desired velocity fo the end effector
% K: Positive definite matrix to reduce the error
% dotQ: velocity to be appleid at the joints
function [dotQ] = invDiffKinematiControlRedundancy(Jac,q, xe, xd, vd, dotq0, K)
    [J] = Jac(q);
    J = J(1:3, 1:6);

    dotQ = pinv(J)*(vd+K*(xd-xe))+ (eye(6,6)-pinv(J)*J)*dotq0;
end