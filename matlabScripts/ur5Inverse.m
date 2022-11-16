%Inverse Kineamtics of UR5
%
function [Th] = ur5Inverse(p60, R60)

 %Vector of the a distance (expressed in metres)
 A = [0, -0.425, -0.3922, 0, 0, 0];
 %Vector of the D distance (expressed in metres)
 D = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
 
 T60 = [R60, p60; zeros(1,3), 1];
 
T10f = @(th1)  [
    cos(th1), -sin(th1), 0, 0;
    sin(th1), cos(th1), 0, 0;
    0, 0, 1, D(1);
    0, 0, 0, 1
    ];
T21f = @(th2) [
    cos(th2), -sin(th2), 0, 0;
    0, 0, -1, 0;
    sin(th2), cos(th2), 0, 0;
    0, 0, 0, 1
    ];

T32f = @(th3) [
    cos(th3), -sin(th3), 0, A(2);
    sin(th3), cos(th3), 0, 0;
    0, 0, 1, D(3);
    0, 0, 0, 1
    ];

T43f = @(th4) [
    cos(th4), -sin(th4), 0, A(3);
    sin(th4), cos(th4), 0, 0;
    0, 0, 1, D(4);
    0, 0, 0, 1
    ];

T54f = @(th5) [
    cos(th5), -sin(th5), 0, 0;
    0, 0, -1, -D(5);
    sin(th5), cos(th5), 0, 0;
    0, 0, 0, 1
    ];
T65f =  @(th6) [
    cos(th6), -sin(th6), 0, 0;
    0, 0, 1, D(6);
    -sin(th6), -cos(th6), 0, 0;
    0, 0, 0, 1
    ];


 
 %Finding th1
 p50 = T60*[0;0;-D(6);1];
 th1_1 = real(atan2(p50(2), p50(1)) + acos(D(4)/hypot(p50(2), p50(1))))+pi/2;
 th1_2 = real(atan2(p50(2), p50(1)) - acos(D(4)/hypot(p50(2), p50(1))))+pi/2;
 
 %finding th5
 th5_1 = +real(acos((p60(1)*sin(th1_1) - p60(2)*cos(th1_1)-D(4)) / D(6)));
 th5_2 = -real(acos((p60(1)*sin(th1_1) - p60(2)*cos(th1_1)-D(4)) / D(6)));
 th5_3 = +real(acos((p60(1)*sin(th1_2) - p60(2)*cos(th1_2)-D(4)) / D(6)));
 th5_4 = -real(acos((p60(1)*sin(th1_2) - p60(2)*cos(th1_2)-D(4)) / D(6)));
 

  
  %related to th11 a th51
  T06 = inv(T60);
  Xhat = T06(1:3,1);
  Yhat = T06(1:3,2);
  
  th6_1 = real(atan2(((-Xhat(2)*sin(th1_1)+Yhat(2)*cos(th1_1)))/sin(th5_1), ((Xhat(1)*sin(th1_1)-Yhat(1)*cos(th1_1)))/sin(th5_1)));
  %related to th11 a th52
  th6_2 = real(atan2(((-Xhat(2)*sin(th1_1)+Yhat(2)*cos(th1_1))/sin(th5_2)), ((Xhat(1)*sin(th1_1)-Yhat(1)*cos(th1_1))/sin(th5_2))));
  %related to th12 a th53
  th6_3 = real(atan2(((-Xhat(2)*sin(th1_2)+Yhat(2)*cos(th1_2))/sin(th5_3)), ((Xhat(1)*sin(th1_2)-Yhat(1)*cos(th1_2))/sin(th5_3))));
  %related to th12 a th54
  th6_4 = real(atan2(((-Xhat(2)*sin(th1_2)+Yhat(2)*cos(th1_2))/sin(th5_4)), ((Xhat(1)*sin(th1_2)-Yhat(1)*cos(th1_2))/sin(th5_4))));
  
 
 

 T41m = inv(T10f(th1_1))*T60*inv(T65f(th6_1))*inv(T54f(th5_1));
 p41_1 = T41m(1:3,4);
 p41xz_1 = hypot(p41_1(1), p41_1(3));
 
 
 T41m = inv(T10f(th1_1))*T60*inv(T65f(th6_2))*inv(T54f(th5_2));
 p41_2 = T41m(1:3,4);
 p41xz_2 = hypot(p41_2(1), p41_2(3));
 
 T41m = inv(T10f(th1_2))*T60*inv(T65f(th6_3))*inv(T54f(th5_3));
 p41_3 = T41m(1:3,4);
 p41xz_3 = hypot(p41_3(1), p41_3(3));
 
 T41m = inv(T10f(th1_2))*T60*inv(T65f(th6_4))*inv(T54f(th5_4));
 p41_4 = T41m(1:3,4);
 p41xz_4 = hypot(p41_4(1), p41_4(3));
 
 %Computation of the 8 possible values for th3    
 th3_1 = real(acos((p41xz_1^2-A(2)^2-A(3)^2)/(2*A(2)*A(3))));
 th3_2 = real(acos((p41xz_2^2-A(2)^2-A(3)^2)/(2*A(2)*A(3))));
 th3_3 = real(acos((p41xz_3^2-A(2)^2-A(3)^2)/(2*A(2)*A(3))));
 th3_4 = real(acos((p41xz_4^2-A(2)^2-A(3)^2)/(2*A(2)*A(3))));
 
 th3_5 = -th3_1;
 th3_6 = -th3_2;
 th3_7 = -th3_3;
 th3_8 = -th3_4;
 
 %Computation of eight possible value for th2
 th2_1 = real(atan2(-p41_1(3), -p41_1(1))-asin((-A(3)*sin(th3_1))/p41xz_1));
 th2_2 = real(atan2(-p41_2(3), -p41_2(1))-asin((-A(3)*sin(th3_2))/p41xz_2));
 th2_3 = real(atan2(-p41_3(3), -p41_3(1))-asin((-A(3)*sin(th3_3))/p41xz_3));
 th2_4 = real(atan2(-p41_4(3), -p41_4(1))-asin((-A(3)*sin(th3_4))/p41xz_4));
 
 th2_5 = real(atan2(-p41_1(3), -p41_1(1))-asin((A(3)*sin(th3_1))/p41xz_1));
 th2_6 = real(atan2(-p41_2(3), -p41_2(1))-asin((A(3)*sin(th3_2))/p41xz_2));
 th2_7 = real(atan2(-p41_3(3), -p41_3(1))-asin((A(3)*sin(th3_3))/p41xz_3));
 th2_8 = real(atan2(-p41_4(3), -p41_4(1))-asin((A(3)*sin(th3_4))/p41xz_4));
 
 
 T43m = inv(T32f(th3_1))*inv(T21f(th2_1))*inv(T10f(th1_1))*T60*inv(T65f(th6_1))*inv(T54f(th5_1));
 Xhat43 = T43m(1:3,1);
 th4_1 = real(atan2(Xhat43(2), Xhat43(1)));
 
 T43m = inv(T32f(th3_2))*inv(T21f(th2_2))*inv(T10f(th1_1))*T60*inv(T65f(th6_2))*inv(T54f(th5_2));
 Xhat43 = T43m(1:3,1);
 th4_2 = real(atan2(Xhat43(2), Xhat43(1)));
 
 T43m = inv(T32f(th3_3))*inv(T21f(th2_3))*inv(T10f(th1_2))*T60*inv(T65f(th6_3))*inv(T54f(th5_3));
 Xhat43 = T43m(1:3,1);
 th4_3 = real(atan2(Xhat43(2), Xhat43(1)));
 
 T43m = inv(T32f(th3_4))*inv(T21f(th2_4))*inv(T10f(th1_2))*T60*inv(T65f(th6_4))*inv(T54f(th5_4));
 Xhat43 = T43m(1:3,1);
 th4_4 = real(atan2(Xhat43(2), Xhat43(1)));
 
 T43m = inv(T32f(th3_5))*inv(T21f(th2_5))*inv(T10f(th1_1))*T60*inv(T65f(th6_1))*inv(T54f(th5_1));
 Xhat43 = T43m(1:3,1);
 th4_5 = real(atan2(Xhat43(2), Xhat43(1)));
 
 T43m = inv(T32f(th3_6))*inv(T21f(th2_6))*inv(T10f(th1_1))*T60*inv(T65f(th6_2))*inv(T54f(th5_2));
 Xhat43 = T43m(1:3,1);
 th4_6 = real(atan2(Xhat43(2), Xhat43(1)));
 
 T43m = inv(T32f(th3_7))*inv(T21f(th2_7))*inv(T10f(th1_2))*T60*inv(T65f(th6_3))*inv(T54f(th5_3));
 Xhat43 = T43m(1:3,1);
 th4_7 = real(atan2(Xhat43(2), Xhat43(1)));
 
 T43m = inv(T32f(th3_8))*inv(T21f(th2_8))*inv(T10f(th1_2))*T60*inv(T65f(th6_4))*inv(T54f(th5_4));
 Xhat43 = T43m(1:3,1);
 th4_8 = real(atan2(Xhat43(2), Xhat43(1))) ;
 
 Th = [th1_1 th2_1 th3_1 th4_1 th5_1 th6_1;
     th1_1 th2_2 th3_2 th4_2 th5_2 th6_2;
     th1_2 th2_3 th3_3 th4_3 th5_3 th6_3;
     th1_2 th2_4 th3_4 th4_4 th5_4 th6_4;
     th1_1 th2_5 th3_5 th4_5 th5_1 th6_1;
     th1_1 th2_6 th3_6 th4_6 th5_2 th6_2;
     th1_2 th2_7 th3_7 th4_7 th5_3 th6_3;
     th1_2 th2_8 th3_8 th4_8 th5_4 th6_4;
     ];