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
function [dotQ] = invDiffKinematiControlCompleteAngleAxis(Jac,q, xe, xd, vd, w_R_e, phid, phiddot, Kp, Kphi)
    w_R_d = eul2rotmFDR(phid);
    error_o = computeOrientationErrorW(w_R_e, w_R_d);
    [J] = Jac(q);
    psid = phid(1);%psi
    thetad = phid(2);%theta
    phid = phid(3); %phi
 
    T = [cos(thetad)*cos(phid), -sin(phid), 0;
        cos(thetad)*sin(phid), cos(phid), 0;
        -sin(thetad), 0, 1];

    
    omega_dot = T*phiddot;
    
% to debug    
%     disp('err pos')
%     norm(xd-xe)
%     disp('err orient')
%     norm(phid-phie)
%     disp('max q')
%     max(abs(dotQ))  
   
    dotQ = inv(J+eye(6)*1e-06)*[(vd+Kp*(xd-xe));(omega_dot+Kphi*error_o)];

end