%homogeneos transformation
function [T] = homogeneousTrans(phi, theta,psi,Ov)
R = eulerRot(phi,theta,psi);

T = [R, Ov;
    zeros(1,3), 1]
end