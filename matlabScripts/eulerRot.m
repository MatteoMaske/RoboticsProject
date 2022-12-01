function [R] = eulerRot(phi, theta, psi)

R = zRot(phi)*yRot(theta)*zRot(psi);

end