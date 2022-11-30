% Function that returns the Rotation about z axis
function [Rz] = zRot(alpha)
    Rz = [
            cos(alpha), - sin(alpha), 0; 
            sin(alpha), cos(alpha), 0; 
            0, 0, 1
          ];
end