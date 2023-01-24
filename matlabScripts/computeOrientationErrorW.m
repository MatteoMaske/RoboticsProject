
function [errorW] = computeOrientationErrorW(w_R_e, w_R_d)
    
    %compute relative orientation 
    e_R_d = w_R_e'*w_R_d;


   %compute the delta_angle
   cos_dtheta = (e_R_d(1,1)+e_R_d(2,2)+e_R_d(3,3) -1)/2;
   sin_dtheta = norm([e_R_d(3,2) -e_R_d(2,3);
                    e_R_d(1,3) -e_R_d(3,1);
                    e_R_d(2,1) -e_R_d(1,2)])*0.5;

   dtheta= atan2(sin_dtheta, cos_dtheta);

   if dtheta == 0 
       errorW = zeros(3,1);
   else

        axis = 1/(2*sin_dtheta)*[e_R_d(3,2)-e_R_d(2,3),
                                e_R_d(1,3)-e_R_d(3,1),
                                e_R_d(2,1)-e_R_d(1,2)];
        errorW = w_R_e * axis * dtheta;
   end

end