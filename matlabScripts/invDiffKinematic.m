% Inversion of the Differential kineamtics using Euler integration
% A:parameters
% Jac: jacobian (function of q)
% ve: velocity of end effector (function of time)
% omegae: angular velocity of the end effector
% q0: initial position of the joints
% minT, maxT: minimum and maximum time
% Dt: delta time
function [TH] = invDiffKinematic(Jac, ve, omegae,  TH0, minT, maxT, Dt)

    T = [minT:Dt:maxT];
    L = length(T);
    qk = [TH0];
    q = [qk];
    for t = T(1:L-1),
        [J ] = Jac(qk);
        dotqk = inv(J)*[ve(t);omegae(t)];
        qk1 = qk + dotqk'*Dt;
        q = [q; qk1];
        qk = qk1;
    end
    TH =q
end