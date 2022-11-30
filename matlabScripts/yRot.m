% Function that returns the Rotation about z axis
function [Ry] = yRot(beta)
    Ry = [
            cos(beta), 0 , sin(beta); 
            0, 1, 0; 
            -sin(beta), 0, cos(beta)
          ];
end