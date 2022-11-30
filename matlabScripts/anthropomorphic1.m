%Draws an athropomorphic arm
% th1, th2, th3: joint variables
%A : lengths of the links
%lim: limits of the axis
function [] = anthropomorphic1(Th, A, lim )
figure;
a0 = A.a0; a2 = A.a2, a3 = A.a3;
th1=Th.th1; th2=Th.th2; th3 = Th.th3;

%Define the axis
axs=axes('XLim',[-lim lim],'YLim',[-lim lim],'ZLim',[-lim lim]); 
view(3); 
grid on;
A0b = [eye(3,3), [0;0;a0]; [0 0 0] 1];
A10 = [[cos(th1) 0 sin(th1); sin(th1) 0 -cos(th1); 0 1 0], [0;0;0];
        [0, 0, 0], 1];
A21 = [[cos(th2) -sin(th2) 0; sin(th2) cos(th2) 0; 0 0 1], [a2*cos(th2);a2*sin(th2);0]; 
    [0 0 0] 1];
A32 =  [[cos(th3) -sin(th3) 0; sin(th3) cos(th3) 0; 0 0 1], [a3*cos(th3);a3*sin(th3);0]; 
    [0 0 0] 1];
hb = triad('Parent', axs,'linewidth',3); 
h0 = triad('Parent',hb,'Matrix', A0b,'linewidth', 3);
h1 = triad('Parent', h0, 'Matrix', A10,'linewidth', 3); 
h2 = triad('Parent', h1, 'Matrix', A21, 'linewidth', 3);
h3 = triad('Parent', h2, 'Matrix', A32,'linewidth', 3);
A1b = A0b*A10; A2b = A1b*A21;A3b=A2b*A32;
Ob =[0;0;0]; O1=A1b(1:3,4); O2=A2b(1:3,4); O3 = A3b(1:3,4);  
plot3([Ob(1), O1(1)], [Ob(2), O1(2)], [Ob(3), O1(3)], 'Color', [0, 0,0], 'Parent', axs,'linestyle','--');
plot3([O1(1), O2(1)], [O1(2), O2(2)], [O1(3), O2(3)], 'Color', [0, 0,0], 'Parent', axs,'linestyle','--');
plot3([O2(1), O3(1)], [O2(2), O3(2)], [O2(3), O3(3)], 'Color', [0, 0,0], 'Parent', axs,'linestyle','--');
end