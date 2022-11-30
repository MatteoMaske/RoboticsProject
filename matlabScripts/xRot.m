% Function that returns the Rotation about z axis
function [Rx] = xRot(gamma)
    Rx = [
    1, 0, 0;
    0, cos(gamma), -sin(gamma);
    0, sin(gamma); cos(gamma)
   ];
end