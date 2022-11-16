% Inverse kinematics of the anthr. manipulator
% pWx, pWy, pWz: position of the wrist
% th1, th2, th3: four possible configurations of the angles
% A: geometric parameters of the manipulator
function [th1, th2, th3] = anthropomorphicInverse(pW,A)
a0 = A.a0; a2 = A.a2; a3 = A.a3;
pWx = pW(1); pWy = pW(2); pWz = pW(3);
pWz = pWz - a0;
c3 = (pWx^2+pWy^2+pWz^2-a2^2-a3^2)/(2*a2*a3);
s3p = sqrt(1-c3^2); s3m = -s3p;

th31 = atan2(s3p, c3);
th32 = -th31;

th21 = atan2((a2+a3*c3)*pWz-a3*s3p*sqrt(pWx^2+pWy^2), (a2+a3*c3)*sqrt(pWx^2+pWy^2)+a3*s3p*pWz);
th22 = atan2((a2+a3*c3)*pWz+a3*s3p*sqrt(pWx^2+pWy^2), -(a2+a3*c3)*sqrt(pWx^2+pWy^2)+a3*s3p*pWz);
th23 = atan2((a2+a3*c3)*pWz-a3*s3m*sqrt(pWx^2+pWy^2), (a2+a3*c3)*sqrt(pWx^2+pWy^2)+a3*s3m*pWz);
th24 = atan2((a2+a3*c3)*pWz+a3*s3m*sqrt(pWx^2+pWy^2),-(a2+a3*c3)*sqrt(pWx^2+pWy^2)+a3*s3p*pWz);
th11 = atan2(pWy, pWx);
th12 = atan2(-pWy, -pWx);
th1 = [th11, th11, th12, th12];
th2 = [th21, th23, th22, th24];
th3 = [th31, th32, th31, th32];
end