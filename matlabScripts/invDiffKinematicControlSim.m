% Simulates using Inverse Differential Kineamtics and Control
% A:parameters
% Jac: jacobian (function of q)
% xd: desired position pf the of end effector (function of time)
% TH0: initial position of the joints
% minT, maxT: minimum and maximum time
% K: positive definite matrix used for control
% Dt: delta time
function [TH] = invDiffKinematicControlSim(direct, Jac, xd, TH0, K, minT, maxT, Dt)

    T = [minT:Dt:maxT];
    L = length(T);
    qk = [TH0];
    q = [qk];
    for t = T(2:L),
        [xe, lost]  = direct(qk);
        vd = (xd(t)-xd(t-Dt))/Dt;
        dotqk = invDiffKinematiControl(Jac,qk, xe, xd(t),  vd, K);
        qk1 = qk + dotqk'*Dt;
        q = [q; qk1];
        qk = qk1;
    end
    TH = q;
end