% Simulates using Inverse Differential Kineamtics and Control
% A:parameters
% Jac: jacobian (function of q)
% xd: desired position pf the of end effector (function of time)
% TH0: initial position of the joints
% minT, maxT: minimum and maximum time
% K: positive definite matrix used for control
% Dt: delta time
function [TH] = invDiffKinematicControlSimComplete(direct, Jac, xd, phid, TH0, Kp, Kphi, minT, maxT, Dt)

    T = [minT:Dt:maxT];
    L = length(T);
    qk = [TH0];
    q = [qk];
    for t = T(2:L),
        [xe, Re]  = direct(qk);
        phie = rotm2eul(Re);
        vd = (xd(t)-xd(t-Dt))/Dt;
        phiddot = (phid(t)-phid(t-Dt))/Dt;
        dotqk = invDiffKinematiControlComplete(Jac,qk, xe, xd(t), vd, phie', phid(t),phiddot, Kp, Kphi);
        qk1 = qk + dotqk'*Dt;
         q = [q; qk1];
        qk = qk1;
    end
    TH = q;
end